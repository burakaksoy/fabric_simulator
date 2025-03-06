#ifndef COMMON_H
#define COMMON_H

#include <math.h>
#include <cmath>
#include <numeric>
#include <vector>
#include <unordered_set>
#include <unordered_map> 
#include <algorithm>    // std::sort, std::find

#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <scipy/spatial.h>

#include <string>
#include <iostream>

#include <float.h>

#include <omp.h>


#define USE_DOUBLE // comment out if you would like to use float.

#ifdef USE_DOUBLE
typedef double Real;
#else
typedef float Real;
#endif // USE_DOUBLE



namespace pbd_object
{

struct Mesh
{
    // EXPLANATIONS OF THE DATA ROWS OF AN .OBJ FILE
    
    // vertices (vertex data):
    // The numbers following "v" are the x, y, and z coordinates of a vertex in 3D space.

    // face_tri_ids (face data):
    // Lines beginning with "f "define the faces of the geometry, using the vertices defined earlier.
    // Each set of numbers in a face definition refers to a vertex index and its corresponding texture coordinate index, separated by a slash (/). The indices start from 1.
    // For instance, "f 1/1 2/2 3/3" defines a face using vertices 1, 2, and 3, with their corresponding texture coordinates 1, 2, and 3.
    // These faces are typically triangles in OBJ files, but can also be other polygons.

    // tex_coords (texture coordinate data):
    // Lines beginning with "vt" represent texture coordinates, used to map a 2D texture onto a 3D model.
    // The numbers following "vt" are the u and v coordinates (similar to x and y) in the texture space, typically ranging from 0.0 to 1.0.

    // normals (vertex normal data):
    // stores the normal vectors for each vertex.
    // Lines beginning with vn represents the normal vector of a vertex, with x, y, and z components.
    // Normal vectors are essential for lighting calculations, as they indicate the direction a surface is facing.

    std::string name; // Name of the mesh, used for identification.

    // Eigen::MatrixX3d vertices;
    Eigen::Matrix<Real,Eigen::Dynamic,3> vertices; // for vertex data (v)
    Eigen::MatrixX3i face_tri_ids; // for face data (f)
    Eigen::Matrix<Real, Eigen::Dynamic, 2> tex_coords; // for texture coordinates (vt), 
    Eigen::Matrix<Real, Eigen::Dynamic, 3> normals; // for normal vectors on each vertex (vn) 
};

} // namespace pbd_object


#endif /* !COMMON_H */