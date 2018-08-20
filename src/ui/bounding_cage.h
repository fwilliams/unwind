#include <Eigen/Core>
#include <memory>
#include <vector>


#ifndef BOUNDING_CAGE_H
#define BOUNDING_CAGE_H


class BoundingCage {
public:
  struct BoundingCageNode {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;
    Eigen::MatrixXd N;

    int level = -1;

    int start = -1;
    int end = -1;

    std::shared_ptr<BoundingCageNode> left;
    std::shared_ptr<BoundingCageNode> right;

    bool is_leaf() const { return !left && !right; }
  };

  BoundingCage();

  const std::vector<std::shared_ptr<BoundingCage::BoundingCageNode>>& components() const { return cage_components; }

  const Eigen::MatrixXd& skeleton_vertices() const { return SV; }
  const Eigen::MatrixXd& smooth_skeleton_vertices() const { return SV_smooth; }

  void set_skeleton_vertices(const Eigen::MatrixXd& SV, unsigned smoothing_iters=0);

private:
  Eigen::MatrixXd SV;
  Eigen::MatrixXd SV_smooth;
  std::shared_ptr<BoundingCageNode> root;
  std::vector<std::shared_ptr<BoundingCageNode>> cage_components;

  bool make_bounding_cage_r(std::shared_ptr<BoundingCageNode> root);
  bool skeleton_in_cage(const Eigen::MatrixXd& CC, const Eigen::MatrixXd& CN, int start, int end);
  std::shared_ptr<BoundingCageNode> make_bounding_cage_component(int v1, int v2, int level);

  Eigen::MatrixXd polygon_for_vertex(int vid, const Eigen::MatrixXd& P) {}

  void smooth_skeleton(int num_iters);

  std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> plane_for_vertex(int vid, double radius);
};

#endif // BOUNDING_CAGE_H
