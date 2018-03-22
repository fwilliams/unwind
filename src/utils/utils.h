#ifndef UTILS_H
#define UTILS_H

#include <igl/edges.h>
#include <igl/barycentric_coordinates.h>

#include <Eigen/Core>

#include <numeric>

#include "datfile.h"


void edge_endpoints(const Eigen::MatrixXd& V,
                    const Eigen::MatrixXi& F,
                    Eigen::MatrixXd& V1,
                    Eigen::MatrixXd& V2) {
  Eigen::MatrixXi E;
  igl::edges(F, E);

  V1.resize(E.rows(), 3);
  V2.resize(E.rows(), 3);
  for (int i = 0; i < E.rows(); i++) {
    V1.row(i) = V.row(E(i, 0));
    V2.row(i) = V.row(E(i, 1));
  }
}


// Scale a vector so its values lie between zero and one
void scale_zero_one(const Eigen::VectorXd& V, Eigen::VectorXd& V_scaled) {
  using namespace Eigen;

  const double v_min = V.minCoeff();
  const double v_max = V.maxCoeff();
  const double v_spread = v_max - v_min;
  if (v_spread == 0.0) {
    V_scaled.resize(V.size());
    V_scaled.setZero();
  } else {
    V_scaled = (V - v_min * VectorXd::Ones(V.size())) / v_spread;
  }
}

void scale_zero_one(Eigen::VectorXd& V) {
  scale_zero_one(V, V);
}


// Check if the point pt is in the tet at ID tet
bool point_in_tet(const Eigen::MatrixXd& TV,
                  const Eigen::MatrixXi& TT,
                  const Eigen::RowVector3d& pt,
                  int tet) {
  using namespace  Eigen;

  auto sgn = [](double val) -> int {
    return (double(0) < val) - (val < double(0));
  };

  Matrix4d D0, D1, D2, D3, D4;
  RowVector3d v1 = TV.row(TT(tet, 0)), v2 = TV.row(TT(tet, 1));
  RowVector3d v3 = TV.row(TT(tet, 2)), v4 = TV.row(TT(tet, 3));

  D0 << v1[0], v1[1], v1[2], 1,
        v2[0], v2[1], v2[2], 1,
        v3[0], v3[1], v3[2], 1,
        v4[0], v4[1], v4[2], 1;

  RowVector4d pt_row(pt[0], pt[1], pt[2], 1);
  D1 = D0;
  D1.row(0) = pt_row;

  D2 = D0;
  D2.row(1) = pt_row;

  D3 = D0;
  D3.row(2) = pt_row;

  D4 = D0;
  D4.row(3) = pt_row;

  const double det0 = D0.determinant();
  assert(det0 != 0);
  const double det1 = D1.determinant();
  const double det2 = D2.determinant();
  const double det3 = D3.determinant();
  const double det4 = D4.determinant();

  return sgn(det1) == sgn(det2) && sgn(det1) == sgn(det3) && sgn(det1) == sgn(det4);
}


// Return the index of the tet containing the point p or -1 if the vertex is in no tets
int containing_tet(const Eigen::MatrixXd& TV,
                   const Eigen::MatrixXi& TT,
                   const Eigen::RowVector3d& p) {
  for (int i = 0; i < TT.rows(); i++) {
    if (point_in_tet(TV, TT, p, i)) {
      return i;
    }
  }
  return -1;
}


// Return the index of the closest vertex to p
int nearest_vertex(const Eigen::MatrixXd& TV,
                   const Eigen::RowVector3d& p) {
  int idx = -1;
  double min_norm = std::numeric_limits<double>::infinity();
  for (int k = 0; k < TV.rows(); k++) {
    double norm = (TV.row(k) - p).norm();
    if (norm < min_norm) {
      idx = k;
      min_norm = norm;
    }
  }
  return idx;
}


void rasterize_tets(const Eigen::MatrixXd& TV, const Eigen::MatrixXi&TT,
                    const Eigen::RowVector3i& grid_size,
                    const Eigen::RowVector3d& cell_size,
                    const Eigen::RowVector3d& origin,
                    const Eigen::MatrixXd& TTC,
                    const Eigen::VectorXd& inV,
                    Eigen::VectorXd& outV) {
  using namespace Eigen;
  using namespace std;

  outV.resize(grid_size[0]*grid_size[1]*grid_size[2]);
  outV.setZero(grid_size[0]*grid_size[1]*grid_size[2]);

  const RowVector3d bb_min = TV.colwise().minCoeff();
  const RowVector3d bb_max = TV.colwise().maxCoeff();
  const RowVector3d bb_dims = bb_max - bb_min;

  for (int i = 0; i < TT.rows(); i++) {
    // These are in [0, 1]
    const RowVector3d v1 = (TV.row(TT(i, 0)) - bb_min).array() / bb_dims.array();
    const RowVector3d v2 = (TV.row(TT(i, 1)) - bb_min).array() / bb_dims.array();
    const RowVector3d v3 = (TV.row(TT(i, 2)) - bb_min).array() / bb_dims.array();
    const RowVector3d v4 = (TV.row(TT(i, 3)) - bb_min).array() / bb_dims.array();

    // Determine the grid cell each tet vertex belongs to
    const RowVector3i g1 = v1.cwiseProduct(grid_size.cast<double>()).cast<int>();
    const RowVector3i g2 = v2.cwiseProduct(grid_size.cast<double>()).cast<int>();
    const RowVector3i g3 = v3.cwiseProduct(grid_size.cast<double>()).cast<int>();
    const RowVector3i g4 = v4.cwiseProduct(grid_size.cast<double>()).cast<int>();

    const int min_x = min(g1[0], min(g2[0], min(g3[0], g4[0])));
    const int max_x = max(g1[0], max(g2[0], max(g3[0], g4[0])));

    const int min_y = min(g1[1], min(g2[1], min(g3[1], g4[1])));
    const int max_y = max(g1[1], max(g2[1], max(g3[1], g4[1])));

    const int min_z = min(g1[2], min(g2[2], min(g3[2], g4[2])));
    const int max_z = max(g1[2], max(g2[2], max(g3[2], g4[2])));

//    cout << g1 << " - " << g2 << " - " << g3 << " - " << g4 << endl;

    for (int z = min_z; z < max_z; z++) {
      for (int y = min_y; y < max_y; y++) {
        for (int x = min_x; x < max_x; x++) {
          const RowVector3d ctr((x+0.5)/grid_size[0], (y+0.5)/grid_size[1], (z+0.5)/grid_size[2]);
          RowVector4d bctr;

          igl::barycentric_coordinates(ctr, v1, v2, v3, v4, bctr);
          // If the center is inside the tet
          if (bctr[0] >= 0 && bctr[0] <= 1 &&
              bctr[1] >= 0 && bctr[1] <= 1 &&
              bctr[2] >= 0 && bctr[2] <= 1 &&
              bctr[3] >= 0 && bctr[3] <= 1) {
            const RowVector3d tc1 = TTC.row(TT(i, 0));
            const RowVector3d tc2 = TTC.row(TT(i, 0));
            const RowVector3d tc3 = TTC.row(TT(i, 0));
            const RowVector3d tc4 = TTC.row(TT(i, 0));

            const RowVector3d texcoord = tc1*bctr[0] + tc2*bctr[1] + tc3*bctr[2] + tc4*bctr[3];

            const int in_idx = z*(grid_size[0]*grid_size[1]) + y*grid_size[0] + x;
            const int out_idx = int(texcoord[2])*(grid_size[0]*grid_size[1]) + int(texcoord[1])*grid_size[0] + int(texcoord[0]);
            outV[in_idx] = inV[out_idx];
//            cout << "outV[" << in_idx << "] = inV[" << out_idx << "] => " << inV[out_idx] << endl;
          }
        }
      }
    }
  }
}

#endif // UTILS_H
