#pragma once

namespace rmcs::util {

template <typename Orientation>
concept orientation_struct_concept = requires() {
    Orientation{}.w;
    Orientation{}.x;
    Orientation{}.y;
    Orientation{}.z;
};

template <typename Orientation>
concept orientation_wrapper_concept = requires() {
    Orientation{}.w();
    Orientation{}.x();
    Orientation{}.y();
    Orientation{}.z();
};

inline void convert_orientation(
    const orientation_wrapper_concept auto& src, orientation_struct_concept auto& dst) {
    dst.w = src.w();
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}
inline void convert_orientation(
    const orientation_struct_concept auto& src, orientation_wrapper_concept auto& dst) {
    dst.w() = src.w;
    dst.x() = src.x;
    dst.y() = src.y;
    dst.z() = src.z;
}
inline void convert_orientation(
    const orientation_struct_concept auto& src, orientation_struct_concept auto& dst) {
    dst.w = src.w;
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
}
inline void convert_orientation(
    const orientation_wrapper_concept auto& src, orientation_wrapper_concept auto& dst) {
    dst.w() = src.w();
    dst.x() = src.x();
    dst.y() = src.y();
    dst.z() = src.z();
}

template <typename Vector3>
concept vector3_struct_concept = requires {
    Vector3{}.x;
    Vector3{}.y;
    Vector3{}.z;
};

template <typename Vector3>
concept vector3_wrapper_concept = requires {
    Vector3{}.x();
    Vector3{}.y();
    Vector3{}.z();
};

inline void
    convert_vector3(const vector3_wrapper_concept auto& src, vector3_struct_concept auto& dst) {
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
}
inline void
    convert_vector3(const vector3_struct_concept auto& src, vector3_wrapper_concept auto& dst) {
    dst.x() = src.x;
    dst.y() = src.y;
    dst.z() = src.z;
}
inline void
    convert_vector3(const vector3_struct_concept auto& src, vector3_struct_concept auto& dst) {
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
}
inline void
    convert_vector3(const vector3_wrapper_concept auto& src, vector3_wrapper_concept auto& dst) {
    dst.x() = src.x();
    dst.y() = src.y();
    dst.z() = src.z();
}

} // namespace rmcs::util
