#ifndef __GLM_CONVERSION_H__
#define __GLM_CONVERSION_H__

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Core>


inline bool operator==(const glm::vec4& lhs, const Eigen::Vector4f& rhs) {
    return lhs.x == rhs.x() && lhs.y == rhs.y() && lhs.z == rhs.z();
}
inline bool operator==(const Eigen::Vector4f& lhs, const glm::vec4& rhs) {
    return operator==(rhs, lhs);
}

inline bool operator!=(const glm::vec4& lhs, const Eigen::Vector4f& rhs) {
    return !operator==(lhs, rhs);
}
inline bool operator!=(const Eigen::Vector4f& lhs, const glm::vec4& rhs) {
    return operator!=(rhs, lhs);
}

// naming scheme
// <a><null | b><c><d>(...)
//
// a: E if conversion from glm to Eigen       G if conversion from Eigen to glm
// b: M if the conversion is a matrix, empty if the conversion is a vector
// c: number of components
// d: type (f: float, i: int, d: double)

#define E3i(glm_vector) Eigen::RowVector3i((glm_vector).x, (glm_vector).y, (glm_vector).z)
#define E3f(glm_vector) Eigen::RowVector3f((glm_vector).x, (glm_vector).y, (glm_vector).z)
#define E3d(glm_vector) Eigen::RowVector3d((glm_vector).x, (glm_vector).y, (glm_vector).z)
#define E2d(glm_vector) Eigen::RowVector2d((glm_vector).x, (glm_vector).y)

#define E4f(glm_vector) Eigen::RowVector4f((glm_vector).x, (glm_vector).y, (glm_vector).z, (glm_vector).w)

#define G2f(eigen_vector) glm::vec2((eigen_vector).x(), (eigen_vector).y())
#define G3f(eigen_vector) glm::vec3((eigen_vector).x(), (eigen_vector).y(), (eigen_vector).z())
#define G3i(eigen_vector) glm::ivec3((eigen_vector).x(), (eigen_vector).y(), (eigen_vector).z())
#define G4f(eigen_vector) glm::vec4((eigen_vector).x(), (eigen_vector).y(), (eigen_vector).z(), (eigen_vector).w())
#define GM3f(eigen_matrix) glm::mat3(                                                    \
    (eigen_matrix).row(0)[0], (eigen_matrix).row(1)[0], (eigen_matrix).row(2)[0],        \
    (eigen_matrix).row(0)[1], (eigen_matrix).row(1)[1], (eigen_matrix).row(2)[1],        \
    (eigen_matrix).row(0)[2], (eigen_matrix).row(1)[2], (eigen_matrix).row(2)[2])

#define GM4f(eigen_matrix) glm::mat4(                                                    \
    (eigen_matrix).row(0)[0], (eigen_matrix).row(1)[0], (eigen_matrix).row(2)[0], (eigen_matrix).row(3)[0], \
    (eigen_matrix).row(0)[1], (eigen_matrix).row(1)[1], (eigen_matrix).row(2)[1], (eigen_matrix).row(3)[1], \
    (eigen_matrix).row(0)[2], (eigen_matrix).row(1)[2], (eigen_matrix).row(2)[2], (eigen_matrix).row(3)[2], \
    (eigen_matrix).row(0)[3], (eigen_matrix).row(1)[3], (eigen_matrix).row(2)[3], (eigen_matrix).row(3)[3])

#endif // __GLM_CONVERSION__H__
