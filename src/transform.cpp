#include "transform.hpp"

/* Default Constructor - Identity Matrix
 */
Transform2d::Transform2d()
    : data({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}),
      inv_data({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) {}

/* Conversion Constructor
 */
Transform2d::Transform2d(const SqMatrix3 matrix)
    : data(matrix), inv_data(Transform2d::inv(matrix)) {}

/* Image-to-Fitted Transform Constructor
 *
 * Constructor used to create a transform for mapping from from image
 * coordinates to "fitted" coordinates and back.  In this context, "world" is
 * image coordinates and "local" is (rejection, projection) coordinates.
 */
Transform2d::Transform2d(const cv::Vec4f line_fit, const cv::Point2f ref_pt)
    : Transform2d() {
    float vx = line_fit[0];
    float vy = line_fit[1];
    float x0 = line_fit[2];
    float y0 = line_fit[3];

    // Create transform based on lineFit where the unit vector becomes the
    // y-axis components
    this->data = {
        // clang-format off
        vy, -vx, 0.0,
        vx, vy, 0.0,
        x0, y0, 1.0,
        // clang-format on
    };
    this->inv_data = Transform2d::inv(this->data);

    // We need to translate the origin of the new system from the lineFit (x0,
    // y0) to the "fitted" ref_pt.

    // Step 1: Transform ref_pt to local system created from lineFit
    cv::Point2f xform_ref_pt = this->world_to_local(ref_pt);
    // Step 2: We only want the component of the transformed ref_pt that lies on
    // our new system's y-axis.
    xform_ref_pt.x = 0.0f;
    // Step 3: We transform the result back to world coordinates and modify the
    // transform matrix's translation components.
    cv::Point2f xform_ref_pt_world = this->local_to_world(xform_ref_pt);
    this->data[6] = xform_ref_pt_world.x;
    this->data[7] = xform_ref_pt_world.y;
    // Now we have a matrix to map to "fitted" coordinates from image
    // coordinates To map from the "fitted" coordinates, we use the inverse
    this->inv_data = Transform2d::inv(this->data);
}

Transform2d::Transform2d(
    const float xi,
    const float xj,
    const float yi,
    const float yj,
    const float Ti,
    const float Tj)
    : Transform2d({xi, xj, 0.0f, yi, yj, 0.0f, Ti, Tj, 1.0f}){};

Transform2d Transform2d::rotate_ccw_deg(const float angle) const {
    return this->rotate_ccw_rad(angle * M_PI / 180.0);
};

Transform2d Transform2d::rotate_ccw_rad(const float angle) const {
    float sin_ = sin(angle);
    float cos_ = cos(angle);
    float xi = this->data[0] * cos_ + this->data[1] * -sin_;
    float xj = this->data[0] * sin_ + this->data[1] * cos_;
    float xk = this->data[2];
    float yi = this->data[3] * cos_ + this->data[4] * -sin_;
    float yj = this->data[3] * sin_ + this->data[4] * cos_;
    float yk = this->data[5];
    float Ti = this->data[6];
    float Tj = this->data[7];
    float Tk = this->data[8];
    return Transform2d(SqMatrix3{xi, xj, xk, yi, yj, yk, Ti, Tj, Tk});
}

Transform2d Transform2d::from_rotation(const cv::RotateFlags& rotate_flag) {
    switch (rotate_flag) {
        case cv::ROTATE_90_CLOCKWISE:
            return Transform2d(SqMatrix3{0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f});
        case cv::ROTATE_180:
            return Transform2d(SqMatrix3{-1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f});
        case cv::ROTATE_90_COUNTERCLOCKWISE:
            return Transform2d(SqMatrix3{0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f});
    }
};

Transform2d Transform2d::from_rotation_translation(
    const cv::RotateFlags& rotate_flag, const float tx, const float ty) {
    switch (rotate_flag) {
        case cv::ROTATE_90_CLOCKWISE:
            return Transform2d(0.0f, -1.0f, 1.0f, 0.0f, tx, ty);
        case cv::ROTATE_180:
            return Transform2d(-1.0f, 0.0f, 0.0f, -1.0f, tx, ty);
        case cv::ROTATE_90_COUNTERCLOCKWISE:
            return Transform2d(0.0f, 1.0f, -1.0f, 0.0f, tx, ty);
    }
};

/* Transform a 2D point from world to local coordinates
 *
 * Useful for transforming from world to camera coordinates, or from camera
 * coordinates to best-fit coordinates.
 *
 * (M^-1) p_world = p_local
 */
cv::Point2f Transform2d::world_to_local(const cv::Point2f pt) const {
    return Transform2d::mul(this->inv_data, pt);
}

/* Transform a 2D point from local to world coordinates
 *
 * Useful for transforming from camera to world coordinates, or from best-fit
 * coordinates to camera coordinates.
 *
 * M * p_local = p_world
 */
cv::Point2f Transform2d::local_to_world(const cv::Point2f pt) const {
    return Transform2d::mul(this->data, pt);
}

/* Create a transform that mirrors the y-axis about the x-axis
 */
Transform2d Transform2d::mirror_about_x() const {
    Transform2d mirrored(this->data);
    mirrored.data[3] = -mirrored.data[3];
    mirrored.data[4] = -mirrored.data[4];
    mirrored.inv_data = Transform2d::inv(mirrored.data);
    return mirrored;
}

/* Create a transform that mirrors the x-axis about the y-axis
 */
Transform2d Transform2d::mirror_about_y() const {
    Transform2d mirrored(this->data);
    mirrored.data[0] = -mirrored.data[0];
    mirrored.data[1] = -mirrored.data[1];
    mirrored.inv_data = Transform2d::inv(mirrored.data);
    return mirrored;
}

/* Create a transform offset with a translation but no rotation
 */
Transform2d Transform2d::translate(const float tx, const float ty) const {
    Transform2d mirrored(this->data);
    mirrored.data[6] = tx;
    mirrored.data[7] = ty;
    mirrored.inv_data = Transform2d::inv(mirrored.data);
    return mirrored;
}

/* Determinant of a 3x3 matrix
 *
 * Useful when finding the inverse.  Uses the rule of Sarrus.
 */
float Transform2d::det(const SqMatrix3 matrix) {
    return matrix[0] * matrix[4] * matrix[8] +
           matrix[3] * matrix[7] * matrix[2] +
           matrix[6] * matrix[1] * matrix[6] -
           matrix[0] * matrix[7] * matrix[5] -
           matrix[3] * matrix[1] * matrix[8] -
           matrix[6] * matrix[4] * matrix[2];
}

/* Calculate the z-axis magnitude
 *
 * Cross product of the x-axis and y-axis.
 */
float Transform2d::z_mag() const {
    return this->data[0] * this->data[4] - this->data[1] * this->data[3];
}

/* Adjoint of a 3x3 matrix
 *
 * Transpose of the cofactor matrix.  This is useful when finding the inverse.
 */
SqMatrix3 Transform2d::adj(const SqMatrix3 matrix) {
    float xi = matrix[4] * matrix[8] - matrix[7] * matrix[5];
    float xj = matrix[6] * matrix[5] - matrix[3] * matrix[8];
    float xk = matrix[3] * matrix[7] - matrix[6] * matrix[4];

    float yi = matrix[7] * matrix[2] - matrix[1] * matrix[8];
    float yj = matrix[0] * matrix[8] - matrix[6] * matrix[2];
    float yk = matrix[6] * matrix[1] - matrix[0] * matrix[7];

    float Ti = matrix[1] * matrix[5] - matrix[4] * matrix[2];
    float Tj = matrix[3] * matrix[2] - matrix[0] * matrix[5];
    float Tk = matrix[0] * matrix[4] - matrix[3] * matrix[1];

    return {xi, yi, Ti, xj, yj, Tj, xk, yk, Tk};
}

/* Multiply a 3x3 matrix and a 2-element column vector
 *
 * This is specific to 2D transformations.  The value 1.0 is added to index 3 of
 * the 2-element vector for application of both rotation and translation in a
 * single calculation.
 */
cv::Point2f Transform2d::mul(
    const std::array<float, 9> matrix, const cv::Point2f pt) {
    return {
        matrix[0] * pt.x + matrix[3] * pt.y + matrix[6],
        matrix[1] * pt.x + matrix[4] * pt.y + matrix[7],
    };
}

/* Inverse of a 3x3 matrix
 *
 * M^-1 = adj(M) / det(A)
 */
SqMatrix3 Transform2d::inv(const std::array<float, 9> matrix) {
    std::array<float, 9> adj = Transform2d::adj(matrix);
    float inv_det = 1.0f / Transform2d::det(matrix);
    std::array<float, 9> inverse = {0.0f};
    for (size_t i = 0; i < inverse.size(); ++i) {
        inverse[i] = adj[i] * inv_det;
    }
    float w = inverse[8];
    // Have to re-normalize
    for (size_t i = 0; i < inverse.size(); ++i) {
        inverse[i] /= w;
    }
    return inverse;
}

/* Serialize as a string
 *
 * Useful for debugging.  If serializing to file or some other transfer stream,
 * this should be modified to utilize a standard format.
 */
std::string Transform2d::to_string() const {
    std::stringstream ss;
    ss << "Data\n"
       << "[\n"
       << "  x_axis: [" << this->data[0] << ", " << this->data[1] << ", "
       << this->data[2] << "],\n"
       << "  y_axis: [" << this->data[3] << ", " << this->data[4] << ", "
       << this->data[5] << "],\n"
       << "  T_axis: [" << this->data[6] << ", " << this->data[7] << ", "
       << this->data[8] << "],\n"
       << "]";
    ss << "INVERSE\n"
       << "[\n"
       << "  x_axis: [" << this->inv_data[0] << ", " << this->inv_data[1]
       << ", " << this->inv_data[2] << "],\n"
       << "  y_axis: [" << this->inv_data[3] << ", " << this->inv_data[4]
       << ", " << this->inv_data[5] << "],\n"
       << "  T_axis: [" << this->inv_data[6] << ", " << this->inv_data[7]
       << ", " << this->inv_data[8] << "],\n"
       << "]";
    return ss.str();
}
