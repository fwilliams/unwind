
#include <igl/opengl/glfw/Viewer.h>
#include <igl/colormap.h>

#include "texture_utils.h"
#include "yixin_loader.h"


typedef igl::opengl::glfw::Viewer Viewer;

int main(int argc, char *argv[]) {
  using namespace std;
  using namespace Eigen;

  DatFile f("data/p-tapinosoma.dat");

  MatrixXd TV, TC;
  MatrixXi TF, TT;

  VectorXd in_texture;
  VectorXd out_texture;

  RowVector3i in_tex_size;
  RowVector3i out_tex_size;

  load_texture(f, 1, in_tex_size, in_texture);
  out_tex_size = in_tex_size;

  Viewer viewer;

  load_yixin_tetmesh(f.m_directory + string("/") + f.m_basename + string("_.msh"), TV, TF, TT);
  TC = TV;
  TC.col(0) /= in_tex_size[0];
  TC.col(1) /= in_tex_size[1];
  TC.col(2) /= in_tex_size[2];

  rasterize_tet_mesh(TV, TT, TC, in_tex_size, out_tex_size, in_texture, out_texture);


  MatrixXd pt_positions, pt_colors;
  VectorXd pt_vals;

  sample_volume_texture(out_tex_size, out_texture, 4 /* downsample_factor */, 10.0 /* thresh */, true /* rescale */, pt_positions, pt_vals);
  igl::colormap(igl::COLOR_MAP_TYPE_MAGMA, pt_vals, false, pt_colors);

  viewer.data().add_points(pt_positions, pt_colors);
  viewer.core.align_camera_center(pt_positions);
  viewer.data().point_size = 2.0;
  return viewer.launch();
}
