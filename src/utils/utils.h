#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Core>
#include <vector>
#include <array>


void tet_mesh_faces(const Eigen::MatrixXi& TT, Eigen::MatrixXi& TF, bool flip=false);

int load_tet_file(const std::string& tet, Eigen::MatrixXd& TV, Eigen::MatrixXi& TF, Eigen::MatrixXi& TT);


// Compute heat diffusion
void diffusion_distances(const Eigen::MatrixXd& TV,
                         const Eigen::MatrixXi& TT,
                         const std::vector<std::array<int, 2>>& endpoints,
                         Eigen::VectorXd& isovals);


// Compute approximate geodesic distance
void geodesic_distances(const Eigen::MatrixXd& TV,
                        const Eigen::MatrixXi& TT,
                        const std::vector<std::array<int, 2>>& endpoints,
                        Eigen::VectorXd& isovals);


void edge_endpoints(const Eigen::MatrixXd& V,
                    const Eigen::MatrixXi& F,
                    Eigen::MatrixXd& V1,
                    Eigen::MatrixXd& V2);


// Scale a vector so its values lie between zero and one
void scale_zero_one(const Eigen::VectorXd& V, Eigen::VectorXd& V_scaled);


void scale_zero_one(Eigen::VectorXd& V);


// Check if the point pt is in the tet at ID tet
bool point_in_tet(const Eigen::MatrixXd& TV,
                  const Eigen::MatrixXi& TT,
                  const Eigen::RowVector3d& pt,
                  int tet);


// Return the index of the tet containing the point p or -1 if the vertex is in no tets
int containing_tet(const Eigen::MatrixXd& TV,
                   const Eigen::MatrixXi& TT,
                   const Eigen::RowVector3d& p);


// Return the index of the closest vertex to p
int nearest_vertex(const Eigen::MatrixXd& TV, const Eigen::RowVector3d& p);


template <typename T>
T clamp(T val, T vmin, T vmax) {
  return std::min(vmax, std::max(val, vmin));
}


#endif // UTILS_H
