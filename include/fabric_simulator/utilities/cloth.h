/*
 * Author: Burak Aksoy
 */

#ifndef CLOTH_H
#define CLOTH_H

#include <math.h>
#include <cmath>
#include <numeric>
#include <vector>
#include <unordered_map> 
#include <algorithm>    // std::sort

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
    std::string name;
    // Eigen::MatrixX3d vertices;
    Eigen::Matrix<Real,Eigen::Dynamic,3> vertices;
    Eigen::MatrixX3i face_tri_ids;
    
};

class Cloth
{
public:
    Cloth();
    Cloth(const Mesh &mesh, 
          const Real &stretching_compliance,
          const Real &bending_compliance,
          const Real &density,
          const Real &global_damp_coeff_v);
    ~Cloth();

    void preSolve(const Real &dt, const Eigen::Matrix<Real,1,3> &gravity);
    void solve(const Real &dt);
    void postSolve(const Real &dt);

    void hangFromCorners(const int &num_corners);

    int attachNearest(const Eigen::Matrix<Real,1,3> &pos);
    void updateAttachedPose(const int &id, const Eigen::Matrix<Real,1,3> &pos);

    Eigen::MatrixX2i *getStretchingIdsPtr();
    Eigen::MatrixX4i *getBendingIdsPtr();

    Eigen::Matrix<Real,1,Eigen::Dynamic> *getStretchingLengthsPtr();
    Eigen::Matrix<Real,1,Eigen::Dynamic> *getBendingLengthsPtr();

    Eigen::Matrix<Real,Eigen::Dynamic,3> *getPosPtr();
    Eigen::Matrix<Real,Eigen::Dynamic,3> *getVelPtr();
    Eigen::Matrix<Real,Eigen::Dynamic,3> *getForPtr();

    std::vector<int> *getAttachedIdsPtr();

    void resetForces();

private:
    // Functions
    void initPhysics(const Eigen::MatrixX3i &face_tri_ids);
    Eigen::RowVectorXi findTriNeighbors(const Eigen::MatrixX3i &face_tri_ids);

    int findNearestPositionVectorId(const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>& matrix, const Eigen::Matrix<Real,3,1>& pos);
    void solveStretching(const Real &compliance, const Real &dt);
    void solveBending(const Real &compliance, const Real &dt);

    // Variables
    Mesh mesh_;
    int num_particles_;

    Eigen::Matrix<Real,Eigen::Dynamic,3> pos_;
    Eigen::Matrix<Real,Eigen::Dynamic,3> prev_pos_;
    Eigen::Matrix<Real,Eigen::Dynamic,3> rest_pos_;
    Eigen::Matrix<Real,Eigen::Dynamic,3> vel_;
    Eigen::Matrix<Real,Eigen::Dynamic,3> for_;
    
    Eigen::Matrix<Real,1,Eigen::Dynamic> inv_mass_;
    Real density_; // fabric mass per meter square (kg/m^2)
    Eigen::Matrix<Real,1,3> grads_;

    Eigen::MatrixX2i stretching_ids_;
    Eigen::MatrixX4i bending_ids_;
    Eigen::Matrix<Real,1,Eigen::Dynamic> stretching_lengths_;
    Eigen::Matrix<Real,1,Eigen::Dynamic> bending_lengths_;

    Real stretching_compliance_;
    Real bending_compliance_;

    Real global_damp_coeff_v_; 

    // May not be necessary?
    // Eigen::RowVectorXi attached_ids_; // ids of robot attached particles
    std::vector<int> attached_ids_; // ids of robot attached particles

    // int grab_id_;
    // Real grab_inv_mass_;
    
    // to debug:
    // int only_once;
};

} // namespace pbd_object

#endif /* !CLOTH_H */