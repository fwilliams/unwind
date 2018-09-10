#include "geodesic_distances.h"
#include "utils.h"

#include <igl/grad.h>
#include <igl/cotmatrix.h>
#include <igl/harmonic.h>

#include <Eigen/CholmodSupport>

// Scale a vector so its values lie between zero and one
static void scale_zero_one(const Eigen::VectorXd& V, Eigen::VectorXd& V_scaled) {
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


static void scale_zero_one(Eigen::VectorXd& V) {
  scale_zero_one(V, V);
}


// Compute heat diffusion
static void diffusion_distances(const Eigen::MatrixXd& TV,
                         const Eigen::MatrixXi& TT,
                         const std::vector<std::pair<int, int>>& endpoints,
                         Eigen::VectorXd& isovals) {
  using namespace std;
  using namespace Eigen;

  MatrixXi constraint_indices;
  MatrixXd constraint_values;
  constraint_indices.resize(2*endpoints.size(), 1);
  constraint_values.resize(2*endpoints.size(), 1);

  int ccount = 0;
  for (int i = 0; i < endpoints.size(); i++) {
    auto ep = endpoints[i];
    constraint_indices(ccount, 0) = ep.second;
    constraint_indices(ccount + 1, 0) = ep.first;
    constraint_values(ccount, 0) = 1.0;
    constraint_values(ccount+1, 0) = 0.0;
    ccount += 2;
  }

  igl::harmonic(TV, TT, constraint_indices,
                constraint_values, 1, isovals);

  scale_zero_one(isovals);
}


// Compute approximate geodesic distance
void geodesic_distances(const Eigen::MatrixXd& TV,
                        const Eigen::MatrixXi& TT,
                        const std::vector<std::pair<int, int>>& endpoints,
                        Eigen::VectorXd& isovals,
                        bool normalized) {
  using namespace std;
  using namespace Eigen;

  typedef SparseMatrix<double> SparseMatrixXd;

  // Discrete Gradient operator
  SparseMatrixXd G;
  igl::grad(TV, TT, G);

  SimplicialLDLT<SparseMatrixXd> solver;

  diffusion_distances(TV, TT, endpoints, isovals);

  VectorXd g = G*isovals;
  Map<MatrixXd> V(g.data(), TT.rows(), 3);
  V.rowwise().normalize();

  solver.compute(G.transpose()*G);
  isovals = solver.solve(G.transpose()*g);
  if (normalized) {
    scale_zero_one(isovals);
  }
}
