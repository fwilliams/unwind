#include <CGAL/Triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>

#include <unordered_map>
#include <iostream>


#ifndef CGAL_TETMESH_H
#define CGAL_TETMESH_H

template <typename GT, typename Tds_, typename Lds_>
void load_tetmesh(const CGAL::Triangulation_3<GT, Tds_, Lds_>& tr,
                  Eigen::MatrixXd& TV, Eigen::MatrixXi& TF, Eigen::MatrixXi& TT) {
    using namespace std;
    typedef typename CGAL::Triangulation_3<GT, Tds_, Lds_> Tr;
    typedef typename Tr::Finite_vertices_iterator Vertex_iterator;
    typedef typename Tr::Finite_facets_iterator Facet_iterator;
    typedef typename Tr::Finite_cells_iterator Cell_iterator;
    typedef typename Tr::Vertex_handle Vertex_handle;
    typedef typename Tr::Point Point_3;

    cout << "Loading tet mesh with "
         << tr.number_of_vertices() << " vertices, "
         << tr.number_of_finite_facets() << " facets, and "
         << tr.number_of_finite_cells() << " tets." << endl;

    TV.resize(tr.number_of_vertices(), 3);
    TF.resize(tr.number_of_finite_facets(), 3);
    TT.resize(tr.number_of_finite_cells(), 4);

    // Map vertex handles to integer indexes in the vertex matrix, TV
    std::unordered_map<Vertex_handle, int> V;

    // Load vertices
    int v_idx = 0;
    for (Vertex_iterator vit = tr.finite_vertices_begin(); vit != tr.finite_vertices_end(); ++vit) {
        V[vit] = v_idx;
        Point_3 p = vit->point();

        TV.row(v_idx) = Eigen::RowVector3d(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
        v_idx += 1;
    }

    // Load boundary triangles
    int tri_idx = 0;
    for (Facet_iterator fit = tr.finite_facets_begin(); fit != tr.finite_facets_end(); ++fit) {
        int col_idx = 0;
        for (int i = 0; i < 4; i++) {
            if (i != fit->second) {
                const Vertex_handle& vh = (*fit).first->vertex(i);
                TF(tri_idx, col_idx++) = V[vh];
            }
        }
        tri_idx += 1;
    }

    // Load Tets
    int tet_idx = 0;
    for (Cell_iterator cit = tr.finite_cells_begin(); cit != tr.finite_cells_end(); ++cit)  {
        for (int i=0; i < 4; i++) {
            TT(tet_idx, i) = V[cit->vertex(i)];
        }
        tet_idx += 1;
    }

    cout << "Done!" << endl;
}

template <typename Tr>
void load_tetmesh_c3t3(const CGAL::Mesh_complex_3_in_triangulation_3<Tr>& c3t3,
                       Eigen::MatrixXd& TV, Eigen::MatrixXi& TF, Eigen::MatrixXi& TT) {
    using namespace std;
    typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3T3;
    typedef typename C3T3::Facets_in_complex_iterator Facet_iterator;
    typedef typename C3T3::Cells_in_complex_iterator Cell_iterator;
    typedef typename Tr::Finite_vertices_iterator Vertex_iterator;
    typedef typename Tr::Vertex_handle Vertex_handle;
    typedef typename Tr::Point Point_3;

    const Tr& tr = c3t3.triangulation();

    cout << "Loading tet mesh with "
         << tr.number_of_vertices() << " vertices, "
         << c3t3.number_of_facets_in_complex() << " facets, and "
         << c3t3.number_of_cells_in_complex() << " tets." << endl;

    TV.resize(tr.number_of_vertices(), 3);
    TF.resize(c3t3.number_of_facets_in_complex(), 3);
    TT.resize(c3t3.number_of_cells_in_complex(), 4);

    // Map vertex handles to integer indexes in the vertex matrix, TV
    std::unordered_map<Vertex_handle, int> V;

    // Load vertices
    int v_idx = 0;
    for (Vertex_iterator vit = tr.finite_vertices_begin(); vit != tr.finite_vertices_end(); ++vit) {
        V[vit] = v_idx;
        Point_3 p = vit->point();

        TV.row(v_idx) = Eigen::RowVector3d(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
        v_idx += 1;
    }

    // Load boundary triangles
    int tri_idx = 0;
    for (Facet_iterator fit = c3t3.facets_in_complex_begin(); fit != c3t3.facets_in_complex_end(); ++fit) {
        int col_idx = 0;
        for (int i = 0; i < 4; i++) {
            if (i != fit->second) {
                const Vertex_handle& vh = (*fit).first->vertex(i);
                TF(tri_idx, col_idx++) = V[vh];
            }
        }
        tri_idx += 1;
    }

    // Load tets
    int tet_idx = 0;
    for (Cell_iterator cit = c3t3.cells_in_complex_begin(); cit != c3t3.cells_in_complex_end(); ++cit)  {
        for (int i=0; i < 4; i++) {
            TT(tet_idx, i) = V[cit->vertex(i)];
        }
        tet_idx += 1;
    }

    cout << "Done!" << endl;
}

#endif // CGAL_TETMESH_H
