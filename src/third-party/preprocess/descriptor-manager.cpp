#include "descriptor-manager.hpp"
#include "iris/LidarIris.hpp"
#include "util/logger.hpp"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/logging.hpp>

using namespace rmcs;

struct DescriptorManager::Impl {
public:
    void make_context(const std::shared_ptr<PointCloudT>& pointcloud) {
        generate_key_frames(pointcloud);
        key_descriptors.resize(key_frames.size());

        std::size_t index{0};
        for (const auto& [point, pointcloud] : key_frames) {
            const auto mat           = LidarIris::GetIris(*pointcloud);
            const auto feature       = descriptor_core.GetFeature(mat);
            key_descriptors[index++] = std::make_pair(point, feature);
        }
    }

    std::pair<Eigen::Isometry3f, PointCloudT>
        query(const std::shared_ptr<PointCloudT>& pointcloud) {
        const auto mat     = LidarIris::GetIris(*pointcloud);
        const auto feature = descriptor_core.GetFeature(mat);

        auto bias_min     = int{-1};
        auto point_min    = Eigen::Vector2f{};
        auto distance_min = double{std::numeric_limits<double>::infinity()};
        auto frame_min    = PointCloudT{};

        // #pragma omp parallel for default(none) num_threads(MP_PROC_NUM)
        std::size_t index{0};
        for (const auto& [point, key_feature] : key_descriptors) {
            // TODO: 修改 iris 的接口，裸指针不太能接受
            auto bias     = int{-1};
            auto distance = descriptor_core.Compare(key_feature, feature, &bias);
            if (distance < distance_min)
                std::tie(point_min, bias_min, distance_min, frame_min) =
                    std::tuple{point, bias, distance, *key_frames[index].second};
            index++;
        }
        rclcpp_info(
            "query result: bias[%d] distance[%4.2f] x[%4.2f] y[%4.2f] key size[%zu]", bias_min,
            distance_min, point_min.x(), point_min.y(), frame_min.size());

        auto translation = Eigen::Translation3f{point_min.x(), point_min.y(), 0};
        auto orientation = Eigen::AngleAxisf(
            static_cast<float>(bias_min) / 180.f * static_cast<float>(std::numbers::pi),
            Eigen::Vector3f::UnitZ());

        return {translation * orientation, frame_min};
    }

    void debug() noexcept {
        const auto points = generate_key_points();
        rclcpp_info("total size: %zu\n", points.size());
    }

private:
    LidarIris descriptor_core{4, 18, 1.6, 0.75, 50};
    pcl::KdTreeFLANN<PointT> flann_kd_tree;

    std::vector<std::pair<Eigen::Vector2f, std::shared_ptr<PointCloudT>>> key_frames;
    std::vector<std::pair<Eigen::Vector2f, LidarIris::FeatureDesc>> key_descriptors;

    static constexpr auto x_max = +24.5f;
    static constexpr auto x_min = -03.5f;
    static constexpr auto y_max = +7.5f;
    static constexpr auto y_min = -7.5f;

    static constexpr auto interval_length = 2.0f;
    static constexpr auto keyframe_radius = 8.0f;

private:
    RMCS_INITIALIZE_LOGGER("descriptor-manager");

    void generate_key_frames(const std::shared_ptr<PointCloudT>& pointcloud) {

        rclcpp_info("generate key frames from cloud, size: %zu", pointcloud->size());
        flann_kd_tree.setInputCloud(pointcloud);

        const auto key_points = generate_key_points();
        rclcpp_info("the size of key frames is %zu", key_points.size());

        for (const auto& point : key_points) {
            auto indecis   = pcl::Indices{};
            auto distances = std::vector<float>{};
            flann_kd_tree.radiusSearch(
                PointT(point.x(), point.y(), 0), keyframe_radius, indecis, distances);

            if (indecis.size() < 1'00)
                continue;

            auto search_result = std::make_shared<PointCloudT>(indecis.size(), 1);
            for (const auto index : indecis) {
                search_result->points.push_back(pointcloud->at(index));
            }

            auto transform = Eigen::Affine3f{
                Eigen::Translation3f{-point.x(), -point.y(), 0}
                * Eigen::Quaternionf::Identity()
            };
            auto transformed_result = std::make_shared<PointCloudT>(search_result->size(), 1);
            pcl::transformPointCloud(*search_result, *transformed_result, transform);

            key_frames.emplace_back(Eigen::Vector2f{point.x(), point.y()}, transformed_result);
        }
    }

    static std::vector<Eigen::Vector2f> generate_key_points() {
        auto x     = x_min;
        auto y     = y_min;
        auto x_vec = std::vector<float>{};
        auto y_vec = std::vector<float>{};
        while (x < x_max) {
            x_vec.push_back(x);
            x += interval_length;
        }
        while (y < y_max) {
            y_vec.push_back(y);
            y += interval_length;
        }

        auto points = std::vector<Eigen::Vector2f>{};
        for (const auto x : x_vec)
            for (const auto y : y_vec) {
                points.emplace_back(x, y);
            }
        return points;
    }
};

DescriptorManager::DescriptorManager()
    : pimpl(std::make_unique<Impl>()) {}

DescriptorManager::~DescriptorManager() = default;

void DescriptorManager::debug() const { pimpl->debug(); }

void DescriptorManager::make_context(const std::shared_ptr<PointCloudT>& pointcloud) {
    pimpl->make_context(pointcloud);
}

std::pair<Eigen::Isometry3f, DescriptorManager::PointCloudT>
    DescriptorManager::query(const std::shared_ptr<PointCloudT>& pointcloud) {
    return pimpl->query(pointcloud);
}
