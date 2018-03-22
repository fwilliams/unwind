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

template <typename T>
T clamp(T val, T vmin, T vmax) {
  return std::min(vmax, std::max(val, vmin));
}


#endif // UTILS_H
