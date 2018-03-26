#ifndef TEXTURE_UTILS_H
#define TEXTURE_UTILS_H

#include <string>

#include "datfile.h"




void sample_volume_texture(const Eigen::RowVector3i tex_size,
                           const Eigen::VectorXd& texture,
                           int downsample_factor,
                           double thresh,
                           bool rescale,
                           Eigen::MatrixXd& out_pos,
                           Eigen::VectorXd& out_vals);

bool read_texture(const DatFile& f, int zero_pad, Eigen::RowVector3i& out_tex_size, Eigen::VectorXd& out_texture);

void write_texture(const std::string& out_filename, const Eigen::VectorXd& texture);

void rasterize_tet_mesh(const Eigen::MatrixXd& TV,              // #V x 3  Tet mesh vertex positions
                        const Eigen::MatrixXi& TT,              // #T x 4  Tet mesh tet indices
                        const Eigen::MatrixXd& texcoords,       // #V x 3  Texture coordinates of each tet mesh vertex
                        const Eigen::RowVector3i& in_tex_size,  // Dimensions of the input texture
                        const Eigen::RowVector3i& out_tex_size, // Dimensions of the output texture
                        const Eigen::VectorXd& in_texture,      // Input texture
                        Eigen::VectorXd& out_texture);          // Outut texture

#endif // TEXTURE_UTILS_H
