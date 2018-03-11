#include <Eigen/Core>

#include <string>


#ifndef TETRAHEDRALIZE_H
#define TETRAHEDRALIZE_H

bool tetrahedralize_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                         int facet_angle, double facet_size,
                         double facet_distance,
                         int cell_radius_edge_ratio,
                         double cell_size,
                         Eigen::MatrixXd& TV,
                         Eigen::MatrixXi& TF,
                         Eigen::MatrixXi& TT);

#endif // TETRAHEDRALIZE_H
