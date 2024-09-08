#include "transform.hpp"

void Transform2d::mirror_about_y() {
    this->data[0] = -this->data[0];
    this->data[1] = -this->data[1];
    this->inv_data = Transform2d::inv(this->data);
}

/* Default Constructor - Identity Matrix
 */
Transform2d::Transform2d()
    : data({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}),
      inv_data({1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) {}

/* Conversion Constructor
 */
Transform2d::Transform2d(const std::array<float, 9> matrix)
    : data(matrix), inv_data(Transform2d::inv(matrix)) {}

/* Image-to-Fitted Transform Constructor
 *
 * Constructor used to create a transform for mapping from from image
 * coordinates to "fitted" coordinates and back.  In this context, "world" is
 * image coordinates and "local" is (rejection, projection) coordinates.
 */
Transform2d::Transform2d(
    const cv::Vec4f line_fit, const cv::Point2f ref_pt)
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

    // We need to translate the origin of the new system from the lineFit (x0,
    // y0) to the "fitted" ref_pt.

    // Step 1: Transform ref_pt to local system created from lineFit
    cv::Point2f xform_ref_pt = Transform2d::mul(this->data, ref_pt);
    // Step 2: We only want the component of the transformed ref_pt that lies on
    // our new system's y-axis.
    xform_ref_pt.x = 0.0f;
    // Step 3: We transform the result back to world coordinates and modify the
    // transform matrix's translation components.
    cv::Point2f xform_ref_pt_world =
        Transform2d::mul(Transform2d::inv(this->data), xform_ref_pt);
    this->data[6] = xform_ref_pt_world.x;
    this->data[7] = xform_ref_pt_world.y;
    // Now we have a matrix to map to "fitted" coordinates from image
    // coordinates To map from the "fitted" coordinates, we use the inverse
    this->inv_data = Transform2d::inv(this->data);
}

// (M^-1) p_world = p_local
cv::Point2f Transform2d::world_to_local(cv::Point2f pt) const {
    return Transform2d::mul(this->inv_data, pt);
}

// M * p_local = p_world
cv::Point2f Transform2d::local_to_world(const cv::Point2f pt) const {
    return Transform2d::mul(this->data, pt);
}

float Transform2d::det(const std::array<float, 9> matrix) {
    return matrix[0] * matrix[4] * matrix[8] +
           matrix[3] * matrix[7] * matrix[2] +
           matrix[6] * matrix[1] * matrix[6] -
           matrix[0] * matrix[7] * matrix[5] -
           matrix[3] * matrix[1] * matrix[8] -
           matrix[6] * matrix[4] * matrix[2];
}

// Adjoint : Transpose of the cofactor matrix
std::array<float, 9> Transform2d::adj(
    const std::array<float, 9> matrix) {
    float xi = matrix[4] * matrix[8] - matrix[7] * matrix[5];
    float xj = matrix[6] * matrix[5] - matrix[3] * matrix[8];
    float xk = matrix[3] * matrix[7] - matrix[6] * matrix[4];

    float yi = matrix[7] * matrix[2] - matrix[1] * matrix[8];
    float yj = matrix[0] * matrix[8] - matrix[6] * matrix[2];
    float yk = matrix[6] * matrix[1] - matrix[0] * matrix[7];

    float Ti = matrix[1] * matrix[5] - matrix[4] * matrix[2];
    float Tj = matrix[3] * matrix[2] - matrix[0] * matrix[5];
    float Tk = matrix[0] * matrix[4] - matrix[3] * matrix[1];

    return std::array<float, 9>{xi, yi, Ti, xj, yj, Tj, xk, yk, Tk};
}

cv::Point2f Transform2d::mul(
    const std::array<float, 9> matrix, const cv::Point2f pt) {
    return cv::Point2f{
        matrix[0] * pt.x + matrix[3] * pt.y + matrix[6],
        matrix[1] * pt.x + matrix[4] * pt.y + matrix[7],
    };
}

std::array<float, 9> Transform2d::inv(
    const std::array<float, 9> matrix) {
    std::array<float, 9> adj = Transform2d::adj(matrix);
    float inv_det = 1.0f / Transform2d::det(matrix);
    std::array<float, 9> inverse = {0.0f};
    for (size_t i = 0; i < inverse.size(); ++i) {
        inverse[i] = adj[i] * inv_det;
    }
    float w = inverse[8];
    for (size_t i = 0; i < inverse.size(); ++i) {
        inverse[i] /= w;
    }
    return inverse;
}

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
