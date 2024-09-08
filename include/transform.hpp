#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <array>
#include <opencv2/core.hpp>

/* 2D column-major transform
 *
 * Used for mapping 2D values into rotated camera coordinates, as well as
 * mapping values between image coordinates and line fit coordinates
 *
 * Ordering: [xi, xj, xk, yi, yj, yk, Ti, Tj, Tk] where xk, yk, and Tk are only
 * used in support of matrix operations and are typically 0.0, 0.0, and 1.0,
 * respectively;
 */
class Transform2d {
   public:
    Transform2d();
    Transform2d(const cv::Vec4f, const cv::Point2f);
    Transform2d(const std::array<float, 9>);
    // world-to-image or image-to-linefit
    cv::Point2f world_to_local(const cv::Point2f) const;
    // image-to-world or linefit-to-image
    cv::Point2f local_to_world(const cv::Point2f) const;
    void mirror_about_y();
    std::string to_string() const;

   private:
    std::array<float, 9> data;
    std::array<float, 9> inv_data;
    static std::array<float, 9> inv(const std::array<float, 9>);
    static std::array<float, 9> adj(const std::array<float, 9>);
    static cv::Point2f mul(const std::array<float, 9>, const cv::Point2f);
    static float det(const std::array<float, 9>);
};

#endif /* TRANSFORM_HPP */
