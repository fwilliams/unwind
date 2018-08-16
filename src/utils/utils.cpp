#include "utils.h"

#include <igl/edges.h>
#include <igl/barycentric_coordinates.h>

#include <array>
#include <numeric>
#include <fstream>
#include <iostream>


void split_mesh_components(const Eigen::MatrixXi& TT, const Eigen::VectorXi& components, std::vector<Eigen::MatrixXi>& out) {
  const int num_components = components.maxCoeff() + 1;

  for (int c = 0; c < num_components; c++) {
    int count = 0;
    Eigen::MatrixXi TTcomp(TT.rows(), 4);
    for (int i = 0; i < TT.rows(); i++) {
      const int t1 = TT(i, 0), t2 = TT(i, 1), t3 = TT(i, 2), t4 = TT(i, 3);
      const int comp = components[t1];
      assert(components[t1] == components[t2] && components[t1] == components[t3] && components[t1] == components[t4]);
      if (!(components[t1] == components[t2] && components[t1] == components[t3] && components[t1] == components[t4])) {
          std::cerr << "T1: " << components[t1] << " " << components[t2] << " " << components[t3] << " " << components[t4];
          std::cerr.flush();
      }
      if (comp == c) {
        TTcomp.row(count) = Eigen::RowVector4i(t1, t2, t3, t4);
        count += 1;
      }
    }
    TTcomp.conservativeResize(count, 4);
    out.push_back(TTcomp);
  }
}


void tet_mesh_faces(const Eigen::MatrixXi& TT, Eigen::MatrixXi& TF, bool flip) {
  using namespace std;

  vector<array<int, 4>> tris_sorted;

  vector<array<int, 4>> tris;
  for (int i = 0; i < TT.rows(); i++) {
    const int e1 = TT(i, 0);
    const int e2 = TT(i, 1);
    const int e3 = TT(i, 2);
    const int e4 = TT(i, 3);
    array<int, 4> t1, t2, t3, t4;
    if (!flip) {
      t1 = array<int, 4>{{ e1, e2, e3, INT_MAX }};
      t2 = array<int, 4>{{ e1, e3, e4, INT_MAX }};
      t3 = array<int, 4>{{ e2, e4, e3, INT_MAX }};
      t4 = array<int, 4>{{ e1, e4, e2, INT_MAX }};
    } else {
      t1 = array<int, 4>{{ e1, e3, e2, INT_MAX }};
      t2 = array<int, 4>{{ e1, e4, e3, INT_MAX }};
      t3 = array<int, 4>{{ e2, e3, e4, INT_MAX }};
      t4 = array<int, 4>{{ e1, e2, e4, INT_MAX }};
    }
    tris.push_back(t1);
    tris.push_back(t2);
    tris.push_back(t3);
    tris.push_back(t4);
    t1[3] = tris_sorted.size();
    t2[3] = tris_sorted.size()+1;
    t3[3] = tris_sorted.size()+2;
    t4[3] = tris_sorted.size()+3;
    sort(t1.begin(), t1.end());
    sort(t2.begin(), t2.end());
    sort(t3.begin(), t3.end());
    sort(t4.begin(), t4.end());
    tris_sorted.push_back(t1);
    tris_sorted.push_back(t2);
    tris_sorted.push_back(t3);
    tris_sorted.push_back(t4);
  }

  int fcount = 0;
  TF.resize(tris_sorted.size(), 3);
  sort(tris_sorted.begin(), tris_sorted.end());
  for (int i = 0; i < TF.rows();) {
    int v1 = tris_sorted[i][0], v2 = tris_sorted[i][1], v3 = tris_sorted[i][2];
    int tid = tris_sorted[i][3];
    int count = 0;
    while (i < TF.rows() && v1 == tris_sorted[i][0] && v2 == tris_sorted[i][1] && v3 == tris_sorted[i][2]) {
      i += 1;
      count += 1;
    }
    if (count == 1) {
      TF.row(fcount++) = Eigen::RowVector3i(tris[tid][0], tris[tid][1], tris[tid][2]);
    }
  }

  TF.conservativeResize(fcount, 3);
}


void load_tet_file(const std::string& tet, Eigen::MatrixXd& TV, Eigen::MatrixXi& TF, Eigen::MatrixXi& TT) {
  using namespace std;
  using namespace Eigen;

  ifstream ifs(tet);

  int nv = 0;
  int nt = 0;
  string header;
  ifs >> header;
  ifs >> nv;
  ifs >> nt;

  TV.resize(nv, 3);
  TT.resize(nt, 4);
  for (int i = 0; i < nv; i++) {
    double x, y, z;
    ifs >> x >> y >> z;
    TV.row(i) = RowVector3d(x, y, z);
  }
  for (int i = 0; i < nt; i++) {
    int a = -1, b = -1, c = -1, d = -1;
    ifs >> a >> b >> c >> d;
    TT.row(i) = RowVector4i(a, b, c, d);
  }

  tet_mesh_faces(TT, TF, true /*flip*/);
}

bool load_rawfile(const std::string& rawfilename, const Eigen::RowVector3i& dims, Eigen::VectorXd& out, bool normalize) {
    const size_t num_bytes = dims[0] * dims[1] * dims[2];

    char* data = new char[num_bytes];
    std::ifstream rawfile(rawfilename, std::ifstream::binary);

    if (!rawfile.good()) {
        std::cerr << "ERROR: RawFile '" << rawfilename << "' does not exist." << std::endl;
        return false;
    }

    rawfile.read(data, num_bytes);
    if (!rawfile) {
        std::cerr << "ERROR: Only read " << rawfile.gcount() <<
            " bytes from Raw File '" << rawfilename <<
            "' but expected to read " << num_bytes <<
            " bytes." << std::endl;
        return false;
    }
    rawfile.close();

    out.resize(num_bytes);
    for (int i = 0; i < num_bytes; i++) {
        out[i] = static_cast<double>(data[i]);
        if (normalize) {
            static_assert(sizeof(char) == sizeof(std::uint8_t), "Your system is fucked"); // This is dumb but why not
            out[i] /= 255.0;
        }
    }

    return true;
}

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

