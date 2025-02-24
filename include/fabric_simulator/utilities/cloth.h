/*
 * Author: Burak Aksoy
 */

#ifndef CLOTH_H
#define CLOTH_H

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
#endif

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



class Cloth
{
public:
    Cloth();
    Cloth(const Mesh &mesh, 
          const Real &stretching_compliance,
          const Real &bending_compliance,
          const Real &density,
          const Real &global_damp_coeff_v,
          const Eigen::Matrix<Real,1,3> &gravity);
    ~Cloth();

    void preSolve(const Real &dt, const Eigen::Matrix<Real,1,3> &gravity);
    void solve(const Real &dt);
    void postSolve(const Real &dt);

    void changeParticleDynamicity(const int &particle, const bool &is_dynamic);

    void hangFromCorners(const int &num_corners, std::vector<int>& custom_static_particles_);
    void setStaticParticles(const std::vector<int> &particles);
    void setDynamicParticles(const std::vector<int> &particles);

    const bool isStaticParticle(const int &particle) const;
    const bool isDynamicParticle(const int &particle) const;

    void setStretchingCompliance(const Real &stretching_compliance);
    void setBendingCompliance(const Real &bending_compliance);

    const Real getStretchingCompliance();
    const Real getBendingCompliance();

    const Real getDensity();
    const Eigen::Matrix<Real,1,3> getGravity();

    int attachNearest(const Eigen::Matrix<Real,1,3> &pos);
    void updateAttachedPose(const int &id, const Eigen::Matrix<Real,1,3> &pos);

    void updateAttachedVelocity(const int &id, const Eigen::Matrix<Real,1,3> &vel);

    void attachNearestWithRadius(const Eigen::Matrix<Real,1,3> &pos, const Real &r, 
                                    std::vector<int> &ids,
                                    std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                                    bool is_attach=true);

    void attachWithinRadius(const Eigen::Matrix<Real,1,3> &pos, const Real &r, 
                                    std::vector<int> &ids,
                                    std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                                    bool is_attach=true);

    void attachWithinRadius(const Eigen::Matrix<Real,1,3> &pos, const Real &r, 
                                    std::vector<int> &ids,
                                    std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                                    std::unordered_set<int> &sticked_ids,
                                    bool is_attach=true);

    void updateAttachedPoses(const std::vector<int> &ids,
                                const Eigen::Matrix<Real,1,3> &pos,
                                const std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                                const Eigen::Quaternion<Real> &cur_orient,
                                const Eigen::Quaternion<Real> &init_orient);

    Eigen::MatrixX2i *getStretchingIdsPtr();
    Eigen::MatrixX4i *getBendingIdsPtr();

    Eigen::MatrixX3i *getFaceTriIdsPtr();

    Eigen::Matrix<Real,1,Eigen::Dynamic> *getStretchingLengthsPtr();
    Eigen::Matrix<Real,1,Eigen::Dynamic> *getBendingLengthsPtr();

    Eigen::Matrix<Real,Eigen::Dynamic,3> *getPosPtr();
    Eigen::Matrix<Real,Eigen::Dynamic,3> *getPrevPosPtr();
    Eigen::Matrix<Real,Eigen::Dynamic,3> *getVelPtr();
    Eigen::Matrix<Real,Eigen::Dynamic,3> *getForPtr();
    Eigen::Matrix<Real,1,Eigen::Dynamic> *getInvMassPtr();

    std::vector<int> *getAttachedIdsPtr();

    void resetForces();

    void resetLambdas();

private:
    // Functions
    void initPhysics(const Eigen::MatrixX3i &face_tri_ids);
    Eigen::RowVectorXi findTriNeighbors(const Eigen::MatrixX3i &face_tri_ids);

    int findNearestPositionVectorId(const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>& matrix, const Eigen::Matrix<Real,3,1>& pos);

    void findPositionVectorsAndIdsInSphere(const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic> &particlePoses, 
                                       const Real &radius, 
                                       std::vector<int> &ids, 
                                       std::vector<Eigen::Matrix<Real,1,3>> &rel_poses);

    void solveStretching(const Real &compliance, const Real &dt);
    void solveBending(const Real &compliance, const Real &dt);

    // Variables
    Mesh mesh_;

    Eigen::Matrix<Real,1,3> gravity_;

    int num_particles_;

    Eigen::Matrix<Real,Eigen::Dynamic,3> pos_;
    Eigen::Matrix<Real,Eigen::Dynamic,3> prev_pos_;
    Eigen::Matrix<Real,Eigen::Dynamic,3> rest_pos_;
    Eigen::Matrix<Real,Eigen::Dynamic,3> vel_;
    Eigen::Matrix<Real,Eigen::Dynamic,3> for_;
    
    Eigen::Matrix<Real,1,Eigen::Dynamic> inv_mass_;
    std::vector<bool> is_dynamic_; // vector that holds the data whether the particle is dynamic or not

    Real density_; // fabric mass per meter square (kg/m^2)
    Eigen::Matrix<Real,1,3> grads_; // gradient vector of the each constraint for each particle

    Eigen::MatrixX2i stretching_ids_;
    Eigen::MatrixX4i bending_ids_;
    Eigen::Matrix<Real,1,Eigen::Dynamic> stretching_lengths_;
    Eigen::Matrix<Real,1,Eigen::Dynamic> bending_lengths_;

    std::vector<Real> Lambda_stretching_;
    std::vector<Real> Lambda_bending_;

    Real stretching_compliance_;
    Real bending_compliance_;

    Real global_damp_coeff_v_; 

    // May not be necessary?
    std::vector<int> attached_ids_; // ids of robot attached particles

};

} // namespace pbd_object

#endif /* !CLOTH_H */