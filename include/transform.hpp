#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include <array>
#include <opencv2/core.hpp>

/* Square matrix represented by a 9-element array
 *
 * If use grows beyond transformations, a custom type may be needed with matrix
 * operations transferred to it.
 */
typedef std::array<float, 9> SqMatrix3;

/* 2D column-major transform
 *
 * This is a concatenation of a 2D rotation matrix and a 2D translation vector.
 * Used for mapping 2D values into rotated camera coordinates, as well as
 * mapping values between image coordinates and line fit coordinates.
 *
 * Ordering: [xi, xj, xk, yi, yj, yk, Ti, Tj, Tk] where xk, yk, and Tk are only
 * used in support of matrix operations and are typically 0.0, 0.0, and 1.0,
 * respectively;
 */
class Transform2d {
   public:
    Transform2d();
    Transform2d(const cv::Vec4f, const cv::Point2f);
    Transform2d(const SqMatrix3);
    Transform2d(
        const float,
        const float,
        const float,
        const float,
        const float,
        const float);
    static Transform2d from_rotation(const cv::RotateFlags&);
    static Transform2d from_rotation_translation(
        const cv::RotateFlags&, const float tx, const float ty);
    Transform2d mirror_about_y() const;
    Transform2d mirror_about_x() const;
    Transform2d translate(const float, const float) const;
    Transform2d rotate_ccw_deg(float) const;
    Transform2d rotate_ccw_rad(float) const;
    // world-to-image or image-to-linefit
    cv::Point2f world_to_local(const cv::Point2f) const;
    // image-to-world or linefit-to-image
    cv::Point2f local_to_world(const cv::Point2f) const;
    std::string to_string() const;
    float z_mag() const;

   private:
    SqMatrix3 data;
    SqMatrix3 inv_data;
    static SqMatrix3 inv(const SqMatrix3);
    static SqMatrix3 adj(const SqMatrix3);
    static cv::Point2f mul(const SqMatrix3, const cv::Point2f);
    static float det(const SqMatrix3);
};

#endif /* TRANSFORM_HPP */
