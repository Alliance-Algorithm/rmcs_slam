#include "engine.hpp"
#include "util/logger.hpp"
#include "util/parameter.hpp"

#include <Eigen/Eigen>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

#include <fast_gicp/gicp/fast_gicp.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace rmcs;

class Registration::Impl {
public:
    using PointCloudT = pcl::PointCloud<PointT>;

    void initialize(rclcpp::Node& node) {

        const auto message = util::title_text("rmcs-location registration initializing now");
        rclcpp_info(message.c_str());

        auto p = util::quick_paramtetr_reader(node);

        fast_gicp_engine = std::make_unique<fast_gicp::FastGICP<PointT, PointT>>();
        fast_gicp_engine->setTransformationEpsilon(0.000'001);
        fast_gicp_engine->setEuclideanFitnessEpsilon(0.000'001);

        fast_gicp_engine->setNumThreads(p("registration.threads", int {}));

        fast_gicp_engine->setMaxCorrespondenceDistance(
            p("registration.distance_threshold", double {}));
        rclcpp_info("distance_threshold: %f", fast_gicp_engine->getMaxCorrespondenceDistance());

        outlier_removal_filter = std::make_unique<pcl::StatisticalOutlierRemoval<PointT>>();
        outlier_removal_filter->setMeanK(p("registration.outlier_removal.mean_k", int {}));
        outlier_removal_filter->setStddevMulThresh(
            p("registration.outlier_removal.stddev_mul_thresh", double {}));
        rclcpp_info("outlier_removal: %d %4.2f", outlier_removal_filter->getMeanK(),
            outlier_removal_filter->getStddevMulThresh());

        voxel_grid_filter = std::make_unique<pcl::VoxelGrid<PointT>>();
        voxel_grid_filter->setLeafSize(p("registration.voxel_grid.lx", float {}),
            p("registration.voxel_grid.ly", float {}), p("registration.voxel_grid.lz", float {}));
        rclcpp_info("voxel grid: %4.2f %4.2f %4.2f", voxel_grid_filter->getLeafSize().x(),
            voxel_grid_filter->getLeafSize().y(), voxel_grid_filter->getLeafSize().z());

        scan_angle = p("registration.scan_angle", int {});
        rclcpp_info("scan angle: %d", scan_angle);

        coarse_iterations = p("registration.coarse_iterations", int {});
        rclcpp_info("coarse_iterations: %d", coarse_iterations);

        precise_iterations = p("registration.precise_iterations", int {});
        rclcpp_info("precise_iterations: %d", precise_iterations);

        score_threshold = p("registration.score_threshold", double {});
        rclcpp_info("score_threshold: %4.2f", score_threshold);

        fast_gicp_engine->setMaximumIterations(coarse_iterations);

        aligned = false;
    }

    void register_map(const std::shared_ptr<pcl::PointCloud<PointT>>& map) {
        voxel_grid_filter->setInputCloud(map);
        voxel_grid_filter->filter(*map);
        outlier_removal_filter->setInputCloud(map);
        outlier_removal_filter->filter(*map);

        rclcpp_info("register map, size: %zu", map->size());
        fast_gicp_engine->setInputTarget(map);

        aligned = false;
    }

    void register_scan(const std::shared_ptr<pcl::PointCloud<PointT>>& scan) {
        voxel_grid_filter->setInputCloud(scan);
        voxel_grid_filter->filter(*scan);

        rclcpp_info("register scan, size: %zu", scan->size());
        fast_gicp_engine->setInputSource(scan);

        aligned = false;
    }

    void single_match(const std::shared_ptr<PointCloudT>& align) {
        fast_gicp_engine->align(*align);
        aligned = true;
    }

    void single_match(
        const std::shared_ptr<PointCloudT>& align, const Eigen::Isometry3f& transformation) {
        fast_gicp_engine->align(*align, transformation.matrix());
        aligned = true;
    }

    void full_match(const std::shared_ptr<PointCloudT>& align,
        const Eigen::Isometry3f& transformation = Eigen::Isometry3f::Identity()) {

        // rotate and get score, select the best angle
        double score_min = 1.0;
        int angle_best = 0;

        // set maximum iterations for rough match
        fast_gicp_engine->setMaximumIterations(coarse_iterations);
        rclcpp_info("[coarse_iterations] %d", fast_gicp_engine->getMaximumIterations());

        for (auto n = 1; (scan_angle * n / 2) < 181; n++) {

            const auto angle
                = static_cast<int>(scan_angle * static_cast<int>(n / 2) * std::pow(-1, n));

            auto radian = static_cast<float>(static_cast<float>(angle) / 180 * std::numbers::pi);
            auto rotation = Eigen::AngleAxisf(radian, Eigen::Vector3f::UnitZ());
            auto guess = (rotation * transformation).matrix();

            fast_gicp_engine->align(*align, guess);
            auto score = fast_gicp_engine->getFitnessScore(1.0);

            if (score < score_min) {
                score_min = score;
                angle_best = angle;
            }

            rclcpp_info("[angle] %+4d [score] %-.5lf", angle, score);

            if (score < score_threshold) {
                rclcpp_info("[congratulate] score(%.5f) is enough, break", score);
                break;
            }
        }

        rclcpp_info("[angle_best] %-3d", angle_best);
        rclcpp_info("[score_best] %-3f", score_min);

        // set maximum iterations for detailed match
        fast_gicp_engine->setMaximumIterations(precise_iterations);
        rclcpp_info("[maximum_iterations_detailed] %d", precise_iterations);

        // use the best rotation to align
        auto radian = static_cast<float>(static_cast<float>(angle_best) / 180 * std::numbers::pi);
        auto rotation = Eigen::AngleAxisf(radian, Eigen::Vector3f::UnitZ());
        auto guess = (rotation * transformation).matrix();

        fast_gicp_engine->align(*align, guess);

        rclcpp_info("[score][%d times] %-3f", fast_gicp_engine->getMaximumIterations(),
            fast_gicp_engine->getFitnessScore(1.0));

        aligned = true;
    }

    double fitness_score() const { return fast_gicp_engine->getFitnessScore(1.0); }

    Eigen::Isometry3f transformation() const {
        return Eigen::Isometry3f { fast_gicp_engine->getFinalTransformation() };
    }

private:
    RMCS_INITIALIZE_LOGGER("engine");

    std::unique_ptr<fast_gicp::FastGICP<PointT, PointT>> fast_gicp_engine;
    std::unique_ptr<pcl::StatisticalOutlierRemoval<PointT>> outlier_removal_filter;
    std::unique_ptr<pcl::VoxelGrid<PointT>> voxel_grid_filter;

    bool aligned = false;
    int scan_angle {};
    int precise_iterations {};
    int coarse_iterations {};
    double score_threshold {};
};

void Registration::register_map(const std::shared_ptr<pcl::PointCloud<PointT>>& map) {
    pimpl->register_map(map);
}
void Registration::register_scan(const std::shared_ptr<pcl::PointCloud<PointT>>& scan) {
    pimpl->register_scan(scan);
}

void Registration::full_match(const std::shared_ptr<PointCloudT>& align) {
    pimpl->full_match(align);
}

void Registration::single_match(const std::shared_ptr<PointCloudT>& align) {
    pimpl->single_match(align);
}

void Registration::full_match(const std::shared_ptr<pcl::PointCloud<PointT>>& align,
    const Eigen::Isometry3f& transformation) {
    pimpl->full_match(align, transformation);
}

void Registration::single_match(const std::shared_ptr<pcl::PointCloud<PointT>>& align,
    const Eigen::Isometry3f& transformation) {
    pimpl->single_match(align, transformation);
}

double Registration::fitness_score() const { return pimpl->fitness_score(); }

Eigen::Isometry3f Registration::transformation() const { return pimpl->transformation(); }

Registration::Registration()
    : pimpl(std::make_unique<Impl>()) { }

Registration::~Registration() = default;

void Registration::initialize(rclcpp::Node& node) { pimpl->initialize(node); }
