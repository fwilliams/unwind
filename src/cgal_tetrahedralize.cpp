#include "cgal_tetrahedralize.h"
#include "cgal_tetmesh.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/refine_mesh_3.h>

#include <igl/copyleft/cgal/mesh_to_polyhedron.h>

// IO
#include <CGAL/IO/Polyhedron_iostream.h>

// Domain
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, K> Mesh_domain;

// Use Parallelism or not
#ifdef CGAL_CONCURRENT_MESH_3
typedef CGAL::Parallel_tag Concurrency_tag;
#else
 typedef CGAL::Sequential_tag Concurrency_tag;
#endif

// Triangulation
typedef CGAL::Mesh_triangulation_3<Mesh_domain, CGAL::Kernel_traits<Mesh_domain>::Kernel, Concurrency_tag>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;

// Criteria
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;

// To avoid verbose function and named parameters call
using namespace CGAL::parameters;


bool tetrahedralize_mesh_internal(const Polyhedron& polyhedron,
                                  int fa, double fs,
                                  double fd,
                                  int cr,
                                  double cs,
                                  Eigen::MatrixXd& TV,
                                  Eigen::MatrixXi& TF,
                                  Eigen::MatrixXi& TT)
{

  // Create domain
  std::cout << "creating domain... ";
  Mesh_domain domain(polyhedron);
  std::cout << "done" << std::endl;

  // Mesh criteria (no cell_size set)
  std::cout << "gengerating mesh... ";
  if (cs > 0)
  {
    Mesh_criteria criteria(facet_angle=fa, facet_size=fs, facet_distance=fd, cell_radius_edge_ratio=cr, cell_size=cs);
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, no_perturb(), no_exude());
    tetmesh_from_c3t3(c3t3, TV, TF, TT);
  }
  else
  {
    Mesh_criteria criteria(facet_angle=fa, facet_size=fs, facet_distance=fd, cell_radius_edge_ratio=cr);
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, no_perturb(), no_exude());
    tetmesh_from_c3t3(c3t3, TV, TF, TT);
  }
  std::cout << "done" << std::endl;

  std::cout << "success" << std::endl;

  return 0;
}


bool tetrahedralize_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                         int facet_angle, double facet_size,
                         double facet_distance,
                         int cell_radius_edge_ratio,
                         double cell_size,
                         Eigen::MatrixXd& TV,
                         Eigen::MatrixXi& TF,
                         Eigen::MatrixXi& TT)
{

  // To avoid verbose function and named parameters call
  using namespace CGAL::parameters;
  using namespace std;

  cout << "Tetrahedralizing mesh. Please wait 9000 years to an eternity..." << endl;
  Polyhedron polyhedron;
  igl::copyleft::cgal::mesh_to_polyhedron(V, F, polyhedron);

  return tetrahedralize_mesh_internal(polyhedron, facet_angle, facet_size,
                                      facet_distance, cell_radius_edge_ratio, cell_size,
                                      TV, TF, TT);
}
