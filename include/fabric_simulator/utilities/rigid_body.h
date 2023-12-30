/*
 * Author: Burak Aksoy
 * DO NOT USE YET, THIS FILE IS JUST A TEMPLATE FOR POTENTIAL FUTURE IMPLEMENT
 * CURRENTLY, ALL RIGID BODIES ARE ASSUMED TO BE STATIC IN THE SIMULATION
 */

#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <math.h>
#include <cmath>
#include <numeric>
#include <vector>
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
    std::string name;
    // Eigen::MatrixX3d vertices;
    Eigen::Matrix<Real,Eigen::Dynamic,3> vertices;
    Eigen::MatrixX3i face_tri_ids;
    
};

class RigidBody
{
public:
    RigidBody();
    RigidBody(const Mesh &mesh, 
          const );
    ~RigidBody();

    void preSolve(const Real &dt, const Eigen::Matrix<Real,1,3> &gravity);
    void solve(const Real &dt);
    void postSolve(const Real &dt);


    Eigen::MatrixX3i *getFaceTriIdsPtr();

    Eigen::Matrix<Real,Eigen::Dynamic,3> *getPosPtr();
    Eigen::Matrix<Real,Eigen::Dynamic,3> *getVelPtr();
    Eigen::Matrix<Real,Eigen::Dynamic,3> *getForPtr();

    void resetForces();

private:
    // Functions
    void initPhysics(const Eigen::MatrixX3i &face_tri_ids);
    Eigen::RowVectorXi findTriNeighbors(const Eigen::MatrixX3i &face_tri_ids);

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


    Real global_damp_coeff_v_; 

};

} // namespace pbd_object

#endif /* !RIGID_BODY_H */