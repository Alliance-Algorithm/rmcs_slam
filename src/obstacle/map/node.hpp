#pragma once

#include "util/logger.hpp"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

namespace rmcs {

struct ObstacleMap {
public:
    /// @note 当 node_height 为 -1 时，UNKNOWN
    struct Node {
        int8_t value { -1 };

        // 高度表，0 -> 100 对应 0 -> 1 m
        std::unordered_set<uint8_t> height_table;

        std::size_t height_table_size() const { //
            return height_table.size();
        }
        void update_height_table(double height) {
            const auto get_height_value = [](double h) {
                return static_cast<uint8_t>(std::clamp(h, 0., 1.) * 100);
            };
            const auto key = get_height_value(height);
            height_table.insert(key);
        }
        double maximum_height_range() const {
            const auto [min, max] = std::minmax_element(height_table.begin(), height_table.end());
            return static_cast<double>(*max - *min) / 100.;
        }
    };
    using NodesMatrix = std::vector<std::vector<Node>>;

    explicit ObstacleMap(std::size_t width)
        : internal_width(width) {
        internal_nodes.resize(width);
        for (auto& nodes : internal_nodes)
            nodes.resize(width, Node {});
    }

    void update_node(std::size_t x, std::size_t y, int8_t v) {
        assert(ax < width() && ay < width());
        internal_nodes[x][y].value = v;
    }

    void ray_cast_with_infinty_unknown() {
        constexpr auto d = [](auto v) -> auto { return static_cast<double>(v); };
        const auto ox    = width() / 2;
        const auto oy    = width() / 2;

        for (std::size_t x = 0; x < width(); x++)
            for (std::size_t y = 0; y < width(); y++) {
                auto& node = data()[x][y];
                if (node.value == -1) continue;

                if (x == ox) {
                    auto [y_min, y_max] = std::minmax(oy, y);
                    for (auto step = y_min; step < y_max; step++) {
                        auto& node = data()[ox][step];
                        if (node.value == -1) node.value = 0;
                    }
                }

                const auto k1 = (d(y) - d(oy)) / (d(x) - d(ox));
                const auto b1 = d(oy) - k1 * d(ox);
                const auto f1 = [=](std::size_t _x) -> double { return k1 * d(_x) + b1; };

                auto [x_min, x_max] = std::minmax(ox, x);
                for (auto step = x_min; step < x_max; step++) {
                    auto& node1 = data()[step][static_cast<std::size_t>(f1(step))];
                    if (node1.value == -1) node1.value = 0;
                }

                const auto k2 = (d(x) - d(ox)) / (d(y) - d(oy));
                const auto b2 = d(ox) - k2 * d(oy);
                const auto f2 = [=](std::size_t _y) -> double { return k2 * d(_y) + b2; };

                auto [y_min, y_max] = std::minmax(oy, y);
                for (auto step = y_min; step < y_max; step++) {
                    auto& node1 = data()[static_cast<std::size_t>(f2(step))][step];
                    if (node1.value == -1) node1.value = 0;
                }
            }
    }

    void ray_cast_with_infinty_avaliable() {
        constexpr auto d = [](auto v) -> auto { return static_cast<double>(v); };

        // 中点
        const auto ox = width() / 2;
        const auto oy = width() / 2;

        const auto iteration_x = [this, d, oy, ox](std::size_t x, std::size_t y) {
            if (x == ox) {
                int error = oy > y ? -1 : +1;
                for (auto step_y = oy; step_y >= 0 && step_y < width(); step_y += error) {
                    auto& node = internal_nodes[ox][step_y];
                    if (node.value > 0) break;
                    node.value = 0;
                }
                return;
            }

            const auto k = (d(y) - d(oy)) / (d(x) - d(ox));
            const auto b = d(oy) - k * d(ox);
            const auto f = [=](std::size_t _x) -> double { return k * d(_x) + b; };

            int error = ox > x ? -1 : +1;
            for (auto step_x = ox; step_x >= 0 && step_x < width(); step_x += error) {
                auto step_y = std::clamp(static_cast<std::size_t>(std::round(f(step_x))),
                    std::size_t { 0 }, width() - 1);
                auto& node  = internal_nodes[step_x][step_y];
                if (node.value > 0) break;
                node.value = 0;
            }
        };

        // 如果都用上述的X迭代的函数做投射，会生成诡异的图案
        // 需要遍历 X 下标时使用 Y 方向的迭代
        const auto iteration_y = [this, d, oy, ox](std::size_t x, std::size_t y) {
            if (y == oy) {
                int error = ox > x ? -1 : +1;
                for (auto step_x = ox; step_x >= 0 && step_x < width(); step_x += error) {
                    auto& node = internal_nodes[step_x][oy];
                    if (node.value > 0) break;
                    node.value = 0;
                }
                return;
            }

            const auto k = (d(x) - d(ox)) / (d(y) - d(oy));
            const auto b = d(ox) - k * d(oy);
            const auto f = [=](std::size_t _y) -> double { return k * d(_y) + b; };

            int error = oy > y ? -1 : +1;
            for (auto step_y = oy; step_y >= 0 && step_y < width(); step_y += error) {
                auto step_x = std::clamp(static_cast<std::size_t>(std::round(f(step_y))),
                    std::size_t { 0 }, width() - 1);
                auto& node  = internal_nodes[step_x][step_y];
                if (node.value > 0) break;
                node.value = 0;
            }
        };

        for (auto x = 0; x < width(); x++) {
            iteration_y(x, 0);
            iteration_y(x, width() - 1);
        }
        for (auto y = 0; y < width(); y++) {
            iteration_x(0, y);
            iteration_x(width() - 1, y);
        }
    }

    void fill_center(std::size_t radius, int8_t v) {
        if (radius >= width() / 2)
            throw util::runtime_error("you set a wrong radius: " + std::to_string(radius)
                + " on ObstacleMap::fill_center");

        const auto center_x = width() / 2;
        const auto center_y = width() / 2;

        const auto minumim_x = center_x - radius;
        const auto minumim_y = center_y - radius;

        const auto maxumim_x = center_x + radius;
        const auto maxumim_y = center_y + radius;

        for (auto x = minumim_x; x < maxumim_x; x++)
            for (auto y = minumim_y; y < maxumim_y; y++) {
                const auto error_x = static_cast<int>(x) - static_cast<int>(center_x);
                const auto error_y = static_cast<int>(y) - static_cast<int>(center_y);
                if (std::pow(error_x, 2) + std::pow(error_y, 2) <= std::pow(radius, 2))
                    this->update_node(x, y, v);
            }
    }

    Node& operator()(std::size_t row, std::size_t col) {
        assert(ros < width && col < width);
        return internal_nodes[row][col];
    }

    NodesMatrix& data() { return internal_nodes; }

    std::size_t width() const { return internal_width; }

private:
    NodesMatrix internal_nodes;
    std::size_t internal_width;
};

} // namespace rmcs
