#include <Eigen/Core>
#include <memory>
#include <vector>

#include "state.h"


#ifndef BOUNDING_CAGE_H
#define BOUNDING_CAGE_H


class BoundingCage {
public:
  BoundingCage(State& state);

private:
  struct BoundingCageNode {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd C;
    Eigen::MatrixXd N;

    int level;

    int start;
    int end;

    std::shared_ptr<BoundingCageNode> left;
    std::shared_ptr<BoundingCageNode> right;
  };

  std::vector<std::shared_ptr<BoundingCageNode>> cage_components;

  bool make_bounding_cage_r(std::shared_ptr<BoundingCageNode> root);
  bool skeleton_in_cage(const Eigen::MatrixXd& CC, const Eigen::MatrixXd& CN, int start, int end);
  std::shared_ptr<BoundingCageNode> make_bounding_cage_component(int v1, int v2, int level);

  Eigen::MatrixXd polygon_for_vertex(int vid, const Eigen::MatrixXd& P) {}

  std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> plane_for_vertex(int vid, double radius);

  State& state;
};

#endif // BOUNDING_CAGE_H
