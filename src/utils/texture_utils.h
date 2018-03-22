#ifndef TEXTURE_UTILS_H
#define TEXTURE_UTILS_H

#include <fstream>
#include <string>
#include <vector>

#include "datfile.h"
#include "utils.h"




void sample_volume_texture(const Eigen::RowVector3i tex_size,
                           const Eigen::VectorXd& texture,
                           int downsample_factor,
                           double thresh,
                           bool rescale,
                           Eigen::MatrixXd& out_pos,
                           Eigen::VectorXd& out_vals) {
  using namespace std;
  using namespace Eigen;

  const int w = tex_size[0], h = tex_size[1], d = tex_size[2];

  out_pos.resize(texture.rows(), 3);
  out_vals.resize(texture.rows());
  out_pos.setZero();
  out_vals.setZero();

  int num_pos = 0;
  for (int z = 0; z < d; z += downsample_factor) {
    for (int y = 0; y < h; y += downsample_factor) {
      for (int x = 0; x < w; x += downsample_factor) {

        int avg_count = 0;
        double texval = 0.0;
        for (int i = z; i < min(d, z+downsample_factor); i++) {
          for (int j = y; j < min(h, y+downsample_factor); j++) {
            for (int k = x; k < min(w, x+downsample_factor); k++) {
              const int idx = i*w*h + j*w + k;
              texval += texture[idx];
              avg_count += 1;
            }
          }
        }

        texval /= avg_count;
        if(fabs(texval) > thresh) {
          out_vals[num_pos] = texval;
          out_pos.row(num_pos) = RowVector3d(x, y, z);
          num_pos += 1;
        }
        texval = 0.0;
      }
    }
  }

  out_pos.conservativeResize(num_pos, 3);
  out_vals.conservativeResize(num_pos);

  if (rescale) {
    out_vals /= out_vals.maxCoeff();
  }
}


bool read_texture(const DatFile& f, int zero_pad, Eigen::RowVector3i& out_tex_size, Eigen::VectorXd& out_texture) {
  using namespace std;
  using namespace Eigen;

  const int w = f.w, h = f.h, d = f.d;
  const string tex_file_path = f.m_directory + string("/") + f.m_texture_filename;

  vector<unsigned char> texdata(w*h*d);

  ifstream ifs(tex_file_path, ios::binary);
  if (!ifs.good()) {
    cerr << "ERROR: Failed to load texture file " << tex_file_path << endl;
    return false;
  }
  ifs.read((char*)texdata.data(), texdata.size());
  if (!ifs.good()) {
    cerr << "ERROR: Failed to read " << texdata.size() << " bytes from texture file " << tex_file_path << ". Actually read " << ifs.gcount() << " bytes instead." << endl;
    return false;
  }

  out_tex_size = RowVector3i(w+2*zero_pad, h+2*zero_pad, d+2*zero_pad);
  out_texture.resize(out_tex_size[0]*out_tex_size[1]*out_tex_size[2]);

  int count = 0;
  int tex_count = 0;
  for (int z = 0; z < d+2*zero_pad; z++) {
    for (int y = 0; y < h+2*zero_pad; y++) {
      for (int x = 0; x < w+2*zero_pad; x++) {
        if (x < zero_pad || y < zero_pad || z < zero_pad || x > w+zero_pad-1 || y == h+zero_pad-1 || z == d+zero_pad-1) {
          out_texture[count++] = 0.0;
        } else {
          out_texture[count++] = texdata[tex_count++];
        }
      }
    }
  }

  assert(count == out_texture.rows());


  return true;
}

void write_texture(const std::string& out_filename, const Eigen::VectorXd& texture) {
  using namespace std;

  ofstream ofs(out_filename, ios::out | ios::binary);
  vector<unsigned char> outdata;
  outdata.reserve(texture.rows());
  for (int i = 0; i < texture.rows(); i++) {
    outdata.push_back((unsigned char)(texture[i]));
  }

  ofs.write((char*)outdata.data(), outdata.size());
}

void rasterize_tet_mesh(const Eigen::MatrixXd& TV,              // #V x 3  Tet mesh vertex positions
                        const Eigen::MatrixXi& TT,              // #T x 4  Tet mesh tet indices
                        const Eigen::MatrixXd& texcoords,       // #V x 3  Texture coordinates of each tet mesh vertex
                        const Eigen::RowVector3i& in_tex_size,  // Dimensions of the input texture
                        const Eigen::RowVector3i& out_tex_size, // Dimensions of the output texture
                        const Eigen::VectorXd& in_texture,      // Input texture
                        Eigen::VectorXd& out_texture) {         // Outut texture

  using namespace std;
  using namespace Eigen;

  const int out_w = out_tex_size[0], out_h = out_tex_size[1], out_d = out_tex_size[2];
  const int in_w = in_tex_size[0], in_h = in_tex_size[1], in_d = in_tex_size[2];

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

    assert(min_x >= 0);
    assert(min_y >= 0);
    assert(min_z >= 0);
    assert(max_x <= out_w-1);
    assert(max_y <= out_h-1);
    assert(max_z <= out_d-1);

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

#endif // TEXTURE_UTILS_H
