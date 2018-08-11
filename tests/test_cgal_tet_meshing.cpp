#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/refine_mesh_3.h>
// IO
#include <CGAL/IO/Polyhedron_iostream.h>
// Domain
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::Polyhedral_mesh_domain_3<Polyhedron, K> Mesh_domain;
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

int main(int argc, char*argv[]){
    if(argc!=3) {
        std::cerr <<"Usage: " << argv[0] << " input.off output.mesh" <<std::endl;
        return 1;
    }

    // Create input polyhedron
    Polyhedron polyhedron;
    std::ifstream input(argv[1]);
    input >> polyhedron;
    if(input.bad()){
        std::cerr << "Error: Cannot read file " << argv[1] << std::endl;
        return EXIT_FAILURE;
    }
    input.close();

    // Create domain
    std::cout<<"creating domain...";
    Mesh_domain domain(polyhedron);
    std::cout<<"done"<<std::endl;

    // Mesh criteria (no cell_size set)
    //std::cout<<"set criteria...";
    //Mesh_criteria criteria(facet_angle=25, facet_size=0.2, facet_distance=0.01, cell_radius_edge_ratio=3);
    Mesh_criteria criteria(facet_angle=5, facet_size=2, facet_distance=0.1, cell_radius_edge_ratio=3);
    //Mesh_criteria criteria;
    //std::cout<<"done"<<std::endl;
    //howto set the criteria to make the test more fear?

    // Mesh generation
    std::cout<<"mesh gengerating...";
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, no_perturb(), no_exude());
    std::cout<<"done"<<std::endl;

    // Output
    std::ofstream medit_file(argv[2]);
    c3t3.output_to_medit(medit_file);
    medit_file.close();
    std::cout<<"success"<<std::endl;

    return 0;
}
