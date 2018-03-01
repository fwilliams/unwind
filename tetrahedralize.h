#include <Eigen/Core>

#include <string>


#ifndef TETRAHEDRALIZE_H
#define TETRAHEDRALIZE_H

bool tetrahedralize_mesh(const std::string& fname,
                         int facet_angle, double facet_size,
                         double facet_distance,
                         int cell_radius_edge_ratio, double cell_size,
                         Eigen::MatrixXd& TV,
                         Eigen::MatrixXi& TF,
                         Eigen::MatrixXi& TT);

#endif // TETRAHEDRALIZE_H
