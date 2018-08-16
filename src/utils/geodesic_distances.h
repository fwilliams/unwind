#ifndef GEODESIC_DISTANCES_H
#define GEODESIC_DISTANCES_H

#include <Eigen/Core>
#include <vector>

// Compute approximate geodesic distance
void geodesic_distances(const Eigen::MatrixXd& TV,
                        const Eigen::MatrixXi& TT,
                        const std::vector<std::array<int, 2>>& endpoints,
                        Eigen::VectorXd& isovals,
                        bool normalized=true);


#endif // GEODESIC_DISTANCES_H
