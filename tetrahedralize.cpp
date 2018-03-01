#include "tetrahedralize.h"
#include "cgal_tetmesh.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/OFF_reader.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/IO/scan_OFF.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/refine_mesh_3.h>
#include <CGAL/Delaunay_triangulation_3.h>

#include <string>
#include <iostream>
#include <fstream>


// Domain
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, K> Mesh_domain;

// TODO: Make this multithreaded
typedef CGAL::Parallel_tag Concurrency_tag;

// Triangulation
typedef CGAL::Mesh_triangulation_3<Mesh_domain, CGAL::Default,Concurrency_tag>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;

// Criteria
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;



bool tetrahedralize_mesh(const std::string& fname,
                         int facet_angle, double facet_size,
                         double facet_distance,
                         int cell_radius_edge_ratio, double cell_size,
                         Eigen::MatrixXd& TV,
                         Eigen::MatrixXi& TF,
                         Eigen::MatrixXi& TT) {

    // To avoid verbose function and named parameters call
    using namespace CGAL::parameters;
    using namespace std;

    cout << "Tetrahedralizing mesh. Please wait 9000 years to an eternity..." << endl;
    Polyhedron polyhedron;

    std::ifstream input(fname);
    CGAL::scan_OFF(input, polyhedron, true /* verbose */);
    if(input.fail()){
        cerr << "Error: Cannot read file " <<  fname << endl;
        return false;
    }
    input.close();

    if (!CGAL::is_triangle_mesh(polyhedron)){
        cerr << "Input geometry is not triangulated." << endl;
        return false;
    }
    // Create domain
    Mesh_domain domain(polyhedron);

    // Mesh criteria (no cell_size set)
    Mesh_criteria criteria(facet_angle=facet_angle,
                           facet_size=facet_size,
                           facet_distance=facet_distance,
                           cell_radius_edge_ratio=cell_radius_edge_ratio,
                           cell_size=cell_size);

    // Mesh generation
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, no_perturb(), no_exude());

    load_tetmesh_c3t3(c3t3, TV, TF, TT);
    return true;
}
