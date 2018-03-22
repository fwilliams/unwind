#include <igl/opengl/glfw/Viewer.h>
#include <igl/colormap.h>

#include <fstream>
#include <string>
#include <exception>

#include "datfile.h"
#include "yixin_loader.h"
#include "utils.h"


using namespace std;
using namespace Eigen;

typedef igl::opengl::glfw::Viewer Viewer;


void assrt(bool expr, const string& msg=string("")) {
  if (!expr) {
    cerr << msg << endl << flush;
    throw runtime_error(msg);
  }
}

void draw_grid(Viewer& viewer, const DatFile& f, const VectorXd& tvs, int ds_factor, double thresh = 1e-10) {
  const int w = f.w+2, h = f.h+2, d = f.d+2;
  MatrixXd out_texpos;
  VectorXd out_texvals;

  out_texpos.resize(tvs.rows(), 3);
  out_texvals.resize(tvs.rows());
  out_texpos.setZero();
  out_texvals.setZero();

  int num_pos = 0;
  for (int z = 0; z < d; z += ds_factor) {
    for (int y = 0; y < h; y += ds_factor) {
      for (int x = 0; x < w; x += ds_factor) {

        int avg_count = 0;
        double texval = 0.0;
        for (int i = z; i < min(d, z+ds_factor); i++) {
          for (int j = y; j < min(h, y+ds_factor); j++) {
            for (int k = x; k < min(w, x+ds_factor); k++) {
              const int idx = i*w*h + j*w + k;
              texval += tvs[idx];
              avg_count += 1;
            }
          }
        }

        texval /= avg_count;
        if(fabs(texval) > thresh) {
          out_texvals[num_pos] = texval;
          out_texpos.row(num_pos) = RowVector3d(x, y, z);
          num_pos += 1;
        }
        texval = 0.0;
      }
    }
  }

  cout << num_pos << endl;
  out_texpos.conservativeResize(num_pos, 3);
  out_texvals.conservativeResize(num_pos);
  cout << out_texvals.maxCoeff() << " " << out_texvals.minCoeff() << endl;

  MatrixXd drawTexColor;
  viewer.core.align_camera_center(out_texpos);
  viewer.data().point_size = 2.0;
  out_texvals /= out_texvals.maxCoeff();
  igl::colormap(igl::COLOR_MAP_TYPE_JET, out_texvals, false, drawTexColor);
  viewer.data().add_points(out_texpos, drawTexColor);
}


void load_texture(DatFile& f, VectorXd& out_texvals) {
  const int w = f.w, h = f.h, d = f.d;
  vector<unsigned char> texdata(w*h*d);
  ifstream ifs(f.m_directory + string("/") + f.m_texture_filename, ios::binary);
  assrt(ifs.good());
  ifs.read((char*)texdata.data(), texdata.size());
  assrt(ifs.good());
  out_texvals.resize((w+2)*(h+2)*(d+2));
  cout << texdata.size() << endl;

  int count = 0;
  int tex_count = 0;
  for (int z = 0; z < d+2; z++) {
    for (int y = 0; y < h+2; y++) {
      for (int x = 0; x < w+2; x++) {
        if (x == 0 || y == 0 || z == 0 || x == w+1 || y == h+1 || z == d+1) {
          out_texvals[count++] = 0.0;
        } else {
          out_texvals[count++] = texdata[tex_count++];
        }
      }
    }
  }
  assrt(count == out_texvals.rows(), "bad count");
}

template <typename T>
T clamp(T val, T vmin, T vmax) {
  return min(vmax, max(val, vmin));
}

void rasterize_tet_mesh(const Eigen::MatrixXd& TV,              // #V x 3  Tet mesh vertex positions
                        const Eigen::MatrixXi& TT,              // #T x 4  Tet mesh tet indices
                        const Eigen::MatrixXd& texcoords,       // #V x 3  Texture coordinates of each tet mesh vertex
                        const Eigen::RowVector3i& in_tex_size,  // Dimensions of the input texture
                        const Eigen::RowVector3i& out_tex_size, // Dimensions of the output texture
                        const Eigen::VectorXd& in_texture,      // Input texture
                        Eigen::VectorXd& out_texture) {         // Outut texture

  const int out_w = out_tex_size[0], out_h = out_tex_size[1], out_d = out_tex_size[2];
  const int in_w = in_tex_size[0], in_h = in_tex_size[1], in_d = out_tex_size[2];

  out_texture.resize(out_w*out_h*out_d);
  out_texture.setZero();

  const RowVector3d bb_min = TV.colwise().minCoeff();
  const RowVector3d bb_max = TV.colwise().maxCoeff();
  const RowVector3d bb_size = bb_max - bb_min;

  for (int t = 0; t < TT.rows(); t++) {
    // The 4 tet vertices in the normalized bounding box space
    const RowVector3d v1 = (TV.row(TT(t, 0)) - bb_min).array() / bb_size.array();
    const RowVector3d v2 = (TV.row(TT(t, 1)) - bb_min).array() / bb_size.array();
    const RowVector3d v3 = (TV.row(TT(t, 2)) - bb_min).array() / bb_size.array();
    const RowVector3d v4 = (TV.row(TT(t, 3)) - bb_min).array() / bb_size.array();

    // Grid indices of each tet vertex in the output raster grid
    const RowVector3i g1 = v1.cwiseProduct(RowVector3d(out_w-1, out_h-1, out_d-1)).cast<int>();
    const RowVector3i g2 = v2.cwiseProduct(RowVector3d(out_w-1, out_h-1, out_d-1)).cast<int>();
    const RowVector3i g3 = v3.cwiseProduct(RowVector3d(out_w-1, out_h-1, out_d-1)).cast<int>();
    const RowVector3i g4 = v4.cwiseProduct(RowVector3d(out_w-1, out_h-1, out_d-1)).cast<int>();

    const int min_x = min(g1[0], min(g2[0], min(g3[0], g4[0])));
    const int max_x = max(g1[0], max(g2[0], max(g3[0], g4[0])));

    const int min_y = min(g1[1], min(g2[1], min(g3[1], g4[1])));
    const int max_y = max(g1[1], max(g2[1], max(g3[1], g4[1])));

    const int min_z = min(g1[2], min(g2[2], min(g3[2], g4[2])));
    const int max_z = max(g1[2], max(g2[2], max(g3[2], g4[2])));

    assrt(min_x >= 0);
    assrt(min_y >= 0);
    assrt(min_z >= 0);
    assrt(max_x <= out_w-1);
    assrt(max_y <= out_h-1);
    assrt(max_z <= out_d-1);

    for (int z = min_z; z <= max_z; z++) {
      for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
          const RowVector3d ctr((x+0.5)/out_w, (y+0.5)/out_h, (z+0.5)/out_d);
          RowVector4d bctr;
          igl::barycentric_coordinates(ctr, v1, v2, v3, v4, bctr);

          if (bctr[0] >= 0 && bctr[0] <= 1 &&
              bctr[1] >= 0 && bctr[1] <= 1 &&
              bctr[2] >= 0 && bctr[2] <= 1 &&
              bctr[3] >= 0 && bctr[3] <= 1) {

            const RowVector3d tc1 = texcoords.row(TT(t, 0));
            const RowVector3d tc2 = texcoords.row(TT(t, 1));
            const RowVector3d tc3 = texcoords.row(TT(t, 2));
            const RowVector3d tc4 = texcoords.row(TT(t, 3));

            const RowVector3d tcbctr = bctr[0]*tc1 + bctr[1]*tc2 + bctr[2]*tc3 + bctr[3]*tc4;
            const RowVector3i tc = tcbctr.cwiseProduct(RowVector3d(in_w, in_h, in_d)).cast<int>();
            const RowVector3i tc_clamped(clamp(tc[0], 0, in_w-1), clamp(tc[1], 0, in_h-1), clamp(tc[2], 0, in_d-1));

            const int out_idx = z*(out_w*out_h) + y*out_w + x;
            const int in_idx = tc_clamped[2]*in_w*in_h + tc_clamped[1]*in_w + tc_clamped[0];
            out_texture[out_idx] = in_texture[in_idx];
          }
        }
      }
    }
  }
}

int main(int argc, char *argv[]) {
  using namespace std;

  MatrixXd TV, TC;
  MatrixXi TF, TT;

  VectorXd in_texture;
  VectorXd out_texture;

  Viewer viewer;
  DatFile f("data/p-tapinosoma.dat");

  const int w = f.w, h = f.h, d = f.d;


  load_yixin_tetmesh(f.m_directory + string("/") + f.m_basename + string("_.msh"), TV, TF, TT);
  TC = TV;
  TC.col(0) /= (w+2);
  TC.col(1) /= (h+2);
  TC.col(2) /= (d+2);

  load_texture(f, in_texture);
  rasterize_tet_mesh(TV, TT, TC, RowVector3i(w+2, h+2, d+2), RowVector3i(w+2, h+2, d+2), in_texture, out_texture);
  draw_grid(viewer, f, out_texture, 4, 30.0 /* thresh */);


//  const int raster_w = w+2, raster_h = h+2, raster_d = d+2;

//  VectorXd out_texture(raster_w*raster_h*raster_d);
//  out_texture.setZero();

//  const RowVector3d bb_min = TV.colwise().minCoeff();
//  const RowVector3d bb_max = TV.colwise().maxCoeff();
//  const RowVector3d bb_size = bb_max - bb_min;

//  for (int t = 0; t < TT.rows(); t++) {
//    // The 4 tet vertices in the normalized bounding box space
//    const RowVector3d v1 = (TV.row(TT(t, 0)) - bb_min).array() / bb_size.array();
//    const RowVector3d v2 = (TV.row(TT(t, 1)) - bb_min).array() / bb_size.array();
//    const RowVector3d v3 = (TV.row(TT(t, 2)) - bb_min).array() / bb_size.array();
//    const RowVector3d v4 = (TV.row(TT(t, 3)) - bb_min).array() / bb_size.array();

//    // Grid indices of each tet vertex in the output raster grid
//    const RowVector3i g1 = v1.cwiseProduct(RowVector3d(raster_w-1, raster_h-1, raster_d-1)).cast<int>();
//    const RowVector3i g2 = v2.cwiseProduct(RowVector3d(raster_w-1, raster_h-1, raster_d-1)).cast<int>();
//    const RowVector3i g3 = v3.cwiseProduct(RowVector3d(raster_w-1, raster_h-1, raster_d-1)).cast<int>();
//    const RowVector3i g4 = v4.cwiseProduct(RowVector3d(raster_w-1, raster_h-1, raster_d-1)).cast<int>();

//    const int min_x = min(g1[0], min(g2[0], min(g3[0], g4[0])));
//    const int max_x = max(g1[0], max(g2[0], max(g3[0], g4[0])));

//    const int min_y = min(g1[1], min(g2[1], min(g3[1], g4[1])));
//    const int max_y = max(g1[1], max(g2[1], max(g3[1], g4[1])));

//    const int min_z = min(g1[2], min(g2[2], min(g3[2], g4[2])));
//    const int max_z = max(g1[2], max(g2[2], max(g3[2], g4[2])));

//    assrt(min_x >= 0);
//    assrt(min_y >= 0);
//    assrt(min_z >= 0);
//    assrt(max_x <= raster_w-1);
//    assrt(max_y <= raster_h-1);
//    assrt(max_z <= raster_d-1);

//    for (int z = min_z; z <= max_z; z++) {
//      for (int y = min_y; y <= max_y; y++) {
//        for (int x = min_x; x <= max_x; x++) {
//          const RowVector3d ctr((x+0.5)/raster_w, (y+0.5)/raster_h, (z+0.5)/raster_d);
//          RowVector4d bctr;
//          igl::barycentric_coordinates(ctr, v1, v2, v3, v4, bctr);

//          if (bctr[0] >= 0 && bctr[0] <= 1 &&
//              bctr[1] >= 0 && bctr[1] <= 1 &&
//              bctr[2] >= 0 && bctr[2] <= 1 &&
//              bctr[3] >= 0 && bctr[3] <= 1) {

//            const RowVector3d tc1 = TC.row(TT(t, 0));
//            const RowVector3d tc2 = TC.row(TT(t, 1));
//            const RowVector3d tc3 = TC.row(TT(t, 2));
//            const RowVector3d tc4 = TC.row(TT(t, 3));

//            RowVector3d tcbctr = bctr[0]*tc1 + bctr[1]*tc2 + bctr[2]*tc3 + bctr[3]*tc4;
//            tcbctr = RowVector3d(clamp(tcbctr[0], 0.0, 1.0), clamp(tcbctr[1], 0.0, 1.0), clamp(tcbctr[2], 0.0, 1.0));
//            RowVector3i tc = tcbctr.cwiseProduct(RowVector3d(w+2, h+2, d+2)).cast<int>();

//            const int out_idx = z*(raster_w*raster_h) + y*raster_w + x;
//            const int in_idx = tc[2]*(w+2)*(h+2) + tc[1]*(w+2) + tc[0];
//            out_texture[out_idx] = in_texture[in_idx];
//          }
//        }
//      }
//    }
//  }
  return viewer.launch();
}
