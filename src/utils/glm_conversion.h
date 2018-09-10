#ifndef __GLM_CONVERSION_H__
#define __GLM_CONVERSION_H__

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Core>


bool operator==(const glm::vec4& lhs, const Eigen::Vector4f& rhs) {
    return lhs.x == rhs.x() && lhs.y == rhs.y() && lhs.z == rhs.z();
}
bool operator==(const Eigen::Vector4f& lhs, const glm::vec4& rhs) {
    return operator==(rhs, lhs);
}

bool operator!=(const glm::vec4& lhs, const Eigen::Vector4f& rhs) {
    return !operator==(lhs, rhs);
}
bool operator!=(const Eigen::Vector4f& lhs, const glm::vec4& rhs) {
    return operator!=(rhs, lhs);
}

// naming scheme
// <a><null | b><c>(...)
//
// a: E if conversion from glm to Eigen       G if conversion from Eigen to glm
// b: M if the conversion is a matrix, empty if the conversion is a vector
// c: number of components

#define E3i(glm_vector) Eigen::RowVector3i((glm_vector).x, (glm_vector).y, (glm_vector).z)

#define G3(eigen_vector) glm::make_vec3((eigen_vector).data())
#define G4(eigen_vector) glm::make_vec4((eigen_vector).data())
#define GM4(eigen_matrix) glm::make_mat4((eigen_matrix).data())

#endif // __GLM_CONVERSION__H__
