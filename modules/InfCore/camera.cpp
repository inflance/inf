/// @file camera.cpp
#include "camera.hpp"
#include <magic_enum/magic_enum.hpp>
#include <cmath>

namespace inf::core {

CameraModel camera_model_from_string(std::string_view name) noexcept {
    return magic_enum::enum_cast<CameraModel>(name).value_or(CameraModel::PINHOLE);
}

std::string_view camera_model_to_string(CameraModel model) noexcept {
    return magic_enum::enum_name(model);
}

namespace {

// 提取焦距 fx, fy
inline std::pair<double, double> extract_focal(CameraModel model, const std::vector<double>& p) {
    switch (model) {
        case CameraModel::SIMPLE_PINHOLE:
        case CameraModel::SIMPLE_RADIAL:
        case CameraModel::RADIAL:
        case CameraModel::SIMPLE_RADIAL_FISHEYE:
        case CameraModel::RADIAL_FISHEYE:
            return {p[0], p[0]};
        default:
            return {p[0], p[1]};
    }
}

// 提取主点 cx, cy
inline std::pair<double, double> extract_principal(CameraModel model, const std::vector<double>& p) {
    switch (model) {
        case CameraModel::SIMPLE_PINHOLE:
        case CameraModel::SIMPLE_RADIAL:
        case CameraModel::RADIAL:
        case CameraModel::SIMPLE_RADIAL_FISHEYE:
        case CameraModel::RADIAL_FISHEYE:
            return {p[1], p[2]};
        default:
            return {p[2], p[3]};
    }
}

// 径向畸变系数
inline double radial_distortion(double r2, const double* k, size_t n) {
    double factor = 1.0, r_pow = r2;
    for (size_t i = 0; i < n; ++i) {
        factor += k[i] * r_pow;
        r_pow *= r2;
    }
    return factor;
}

// 迭代求解反径向畸变
inline std::pair<double, double> undistort_radial(double x_dist, double y_dist,
                                                   const double* k, size_t n, int max_iter = 20) {
    double x = x_dist, y = y_dist;
    for (int i = 0; i < max_iter; ++i) {
        const double r2 = x * x + y * y;
        const double factor = radial_distortion(r2, k, n);
        const double x_new = x_dist / factor, y_new = y_dist / factor;
        if (std::abs(x_new - x) < 1e-10 && std::abs(y_new - y) < 1e-10) break;
        x = x_new; y = y_new;
    }
    return {x, y};
}

// 迭代求解 OpenCV 模型反畸变
inline std::pair<double, double> undistort_opencv(double x_dist, double y_dist,
                                                   double k1, double k2, double p1, double p2,
                                                   int max_iter = 20) {
    double x = x_dist, y = y_dist;
    for (int i = 0; i < max_iter; ++i) {
        const double r2 = x * x + y * y, r4 = r2 * r2;
        const double radial = 1.0 + k1 * r2 + k2 * r4;
        const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        const double x_new = (x_dist - dx) / radial, y_new = (y_dist - dy) / radial;
        if (std::abs(x_new - x) < 1e-10 && std::abs(y_new - y) < 1e-10) break;
        x = x_new; y = y_new;
    }
    return {x, y};
}

}  // namespace

Vec2d Camera::project(const Vec3d& camera_point) const {
    const double x = camera_point.x() / camera_point.z();
    const double y = camera_point.y() / camera_point.z();
    const auto [fx, fy] = extract_focal(model, params);
    const auto [cx, cy] = extract_principal(model, params);

    double u = 0.0, v = 0.0;

    switch (model) {
        case CameraModel::SIMPLE_PINHOLE:
        case CameraModel::PINHOLE: {
            u = fx * x + cx;
            v = fy * y + cy;
            break;
        }
        case CameraModel::SIMPLE_RADIAL: {
            const double k = params[3], r2 = x * x + y * y;
            const double factor = 1.0 + k * r2;
            u = fx * x * factor + cx;
            v = fy * y * factor + cy;
            break;
        }
        case CameraModel::RADIAL: {
            const double k1 = params[3], k2 = params[4], r2 = x * x + y * y;
            const double factor = 1.0 + k1 * r2 + k2 * r2 * r2;
            u = fx * x * factor + cx;
            v = fy * y * factor + cy;
            break;
        }
        case CameraModel::OPENCV: {
            const double k1 = params[4], k2 = params[5], p1 = params[6], p2 = params[7];
            const double r2 = x * x + y * y, r4 = r2 * r2;
            const double radial = 1.0 + k1 * r2 + k2 * r4;
            const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            u = fx * (x * radial + dx) + cx;
            v = fy * (y * radial + dy) + cy;
            break;
        }
        case CameraModel::OPENCV_FISHEYE: {
            const double r = std::sqrt(x * x + y * y), theta = std::atan(r);
            const double theta2 = theta * theta;
            const double k1 = params[4], k2 = params[5], k3 = params[6], k4 = params[7];
            const double theta_d = theta * (1.0 + k1 * theta2 + k2 * theta2 * theta2 +
                                            k3 * theta2 * theta2 * theta2 +
                                            k4 * theta2 * theta2 * theta2 * theta2);
            const double scale = (r > 1e-8) ? (theta_d / r) : 1.0;
            u = fx * x * scale + cx;
            v = fy * y * scale + cy;
            break;
        }
        case CameraModel::FOV: {
            const double omega = params[4], r = std::sqrt(x * x + y * y);
            double scale = 1.0;
            if (std::abs(omega) > 1e-8 && r > 1e-8) {
                scale = std::atan(2.0 * r * std::tan(omega / 2.0)) / (omega * r);
            }
            u = fx * x * scale + cx;
            v = fy * y * scale + cy;
            break;
        }
        case CameraModel::SIMPLE_RADIAL_FISHEYE: {
            const double k = params[3], r = std::sqrt(x * x + y * y), theta = std::atan(r);
            const double theta_d = theta * (1.0 + k * theta * theta);
            const double scale = (r > 1e-8) ? (theta_d / r) : 1.0;
            u = fx * x * scale + cx;
            v = fy * y * scale + cy;
            break;
        }
        case CameraModel::RADIAL_FISHEYE: {
            const double k1 = params[3], k2 = params[4];
            const double r = std::sqrt(x * x + y * y), theta = std::atan(r), theta2 = theta * theta;
            const double theta_d = theta * (1.0 + k1 * theta2 + k2 * theta2 * theta2);
            const double scale = (r > 1e-8) ? (theta_d / r) : 1.0;
            u = fx * x * scale + cx;
            v = fy * y * scale + cy;
            break;
        }
        default: {
            u = fx * x + cx;
            v = fy * y + cy;
            break;
        }
    }
    return {u, v};
}

Vec3d Camera::unproject(const Vec2d& pixel) const {
    const auto [fx, fy] = extract_focal(model, params);
    const auto [cx, cy] = extract_principal(model, params);
    const double x_dist = (pixel.x() - cx) / fx;
    const double y_dist = (pixel.y() - cy) / fy;

    double x = x_dist, y = y_dist;

    switch (model) {
        case CameraModel::SIMPLE_PINHOLE:
        case CameraModel::PINHOLE:
            break;
        case CameraModel::SIMPLE_RADIAL: {
            const double k = params[3];
            std::tie(x, y) = undistort_radial(x_dist, y_dist, &k, 1);
            break;
        }
        case CameraModel::RADIAL: {
            const double k[2] = {params[3], params[4]};
            std::tie(x, y) = undistort_radial(x_dist, y_dist, k, 2);
            break;
        }
        case CameraModel::OPENCV: {
            std::tie(x, y) = undistort_opencv(x_dist, y_dist, params[4], params[5], params[6], params[7]);
            break;
        }
        case CameraModel::OPENCV_FISHEYE: {
            const double r_dist = std::sqrt(x_dist * x_dist + y_dist * y_dist);
            if (r_dist > 1e-8) {
                const double k1 = params[4], k2 = params[5], k3 = params[6], k4 = params[7];
                double theta = r_dist;
                for (int i = 0; i < 20; ++i) {
                    const double theta2 = theta * theta;
                    const double theta_d = theta * (1.0 + k1 * theta2 + k2 * theta2 * theta2 +
                                                    k3 * theta2 * theta2 * theta2 +
                                                    k4 * theta2 * theta2 * theta2 * theta2);
                    const double delta = theta_d - r_dist;
                    if (std::abs(delta) < 1e-10) break;
                    const double d = 1.0 + 3*k1*theta2 + 5*k2*theta2*theta2 +
                                     7*k3*theta2*theta2*theta2 + 9*k4*theta2*theta2*theta2*theta2;
                    theta -= delta / d;
                }
                const double scale = std::tan(theta) / r_dist;
                x = x_dist * scale; y = y_dist * scale;
            } else { x = 0; y = 0; }
            break;
        }
        case CameraModel::FOV: {
            const double omega = params[4], r_dist = std::sqrt(x_dist * x_dist + y_dist * y_dist);
            if (r_dist > 1e-8 && std::abs(omega) > 1e-8) {
                const double scale = std::tan(r_dist * omega) / (2.0 * std::tan(omega / 2.0) * r_dist);
                x = x_dist * scale; y = y_dist * scale;
            }
            break;
        }
        case CameraModel::SIMPLE_RADIAL_FISHEYE:
        case CameraModel::RADIAL_FISHEYE: {
            const double r_dist = std::sqrt(x_dist * x_dist + y_dist * y_dist);
            if (r_dist > 1e-8) {
                const bool simple = (model == CameraModel::SIMPLE_RADIAL_FISHEYE);
                const double k1 = params[3], k2 = simple ? 0.0 : params[4];
                double theta = r_dist;
                for (int i = 0; i < 20; ++i) {
                    const double theta2 = theta * theta;
                    const double theta_d = theta * (1.0 + k1 * theta2 + k2 * theta2 * theta2);
                    const double delta = theta_d - r_dist;
                    if (std::abs(delta) < 1e-10) break;
                    theta -= delta / (1.0 + 3*k1*theta2 + 5*k2*theta2*theta2);
                }
                const double scale = std::tan(theta) / r_dist;
                x = x_dist * scale; y = y_dist * scale;
            } else { x = 0; y = 0; }
            break;
        }
        default:
            break;
    }
    return Vec3d(x, y, 1.0).normalized();
}

Mat3d Camera::intrinsic_matrix() const {
    const auto [fx, fy] = extract_focal(model, params);
    const auto [cx, cy] = extract_principal(model, params);
    Mat3d K = Mat3d::Identity();
    K(0, 0) = fx; K(1, 1) = fy; K(0, 2) = cx; K(1, 2) = cy;
    return K;
}

}  // namespace inf::core
