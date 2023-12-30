/*
 * Author: Burak Aksoy
 * DO NOT USE YET, THIS FILE IS JUST A TEMPLATE FOR POTENTIAL FUTURE IMPLEMENT
 * CURRENTLY, ALL RIGID BODIES ARE ASSUMED TO BE STATIC IN THE SIMULATION
 */

#include "fabric_simulator/utilities/rigid_body.h"

using namespace pbd_object;

RigidBody::RigidBody(){

}

RigidBody::RigidBody(const Mesh &mesh, 
             const Real &stretching_compliance, 
             const Real &bending_compliance, 
             const Real &density,
             const Real &global_damp_coeff_v):
    mesh_(mesh),
    stretching_compliance_(stretching_compliance),
    bending_compliance_(bending_compliance),
    density_(density),
    global_damp_coeff_v_(global_damp_coeff_v)
{
    num_particles_ = mesh_.vertices.rows();
    std::cout << "num particles: " << num_particles_ << std::endl;
    
    pos_ = mesh_.vertices;
    prev_pos_ = mesh_.vertices;
    rest_pos_ = mesh_.vertices;
    vel_ = Eigen::Matrix<Real,Eigen::Dynamic,3>::Zero(num_particles_,3);
    inv_mass_ = Eigen::Matrix<Real,1,Eigen::Dynamic>::Zero(num_particles_);
    
    for_ = Eigen::Matrix<Real,Eigen::Dynamic,3>::Zero(num_particles_,3);
    
    grads_ = Eigen::Matrix<Real,1,3>::Zero();

    // std::cout << "pos_:\n" << pos_ << std::endl;
    // std::cout << "prev_pos_:\n" << prev_pos_ << std::endl;
    // std::cout << "vel_:\n" << vel_ << std::endl;
    // std::cout << "inv_mass_:\n" << inv_mass_ << std::endl;

    

    initPhysics(mesh_.face_tri_ids);
}

RigidBody::~RigidBody(){

}

void RigidBody::initPhysics(const Eigen::MatrixX3i &face_tri_ids){
    int num_tris = face_tri_ids.rows();

    Real total_area = 0.0;

    for (int i = 0; i < num_tris; i++) {
        int id0 = face_tri_ids(i,0);
        int id1 = face_tri_ids(i,1);
        int id2 = face_tri_ids(i,2);

        Eigen::Matrix<Real,1,3> e0 = pos_.row(id1) - pos_.row(id0);
        Eigen::Matrix<Real,1,3> e1 = pos_.row(id2) - pos_.row(id0);
        Eigen::Matrix<Real,1,3> c = e0.cross(e1);

        Real A = 0.5 * c.norm();  // area
        
        Real mass = A * density_;
    
        if (mass > 0.0) {
            Real p_mass = (mass / 3.0); // divide by 3 because we have 3 vertices per triangle

            Real p_0_mass = ( inv_mass_(id0) > 0.0 ) ? 1.0/inv_mass_(id0) : 0.0;
            Real p_1_mass = ( inv_mass_(id1) > 0.0 ) ? 1.0/inv_mass_(id1) : 0.0;
            Real p_2_mass = ( inv_mass_(id2) > 0.0 ) ? 1.0/inv_mass_(id2) : 0.0;

            p_0_mass += p_mass;
            p_1_mass += p_mass;
            p_2_mass += p_mass;

            inv_mass_(id0) = 1.0/p_0_mass;
            inv_mass_(id1) = 1.0/p_1_mass;
            inv_mass_(id2) = 1.0/p_2_mass;
        }

        total_area += A;
    }

    // std::cout << "inv_mass_:\n" << inv_mass_ << std::endl;
    // std::cout << "particle masses:\n" << inv_mass_.cwiseInverse() << " kg." << std::endl;
    std::cout << "Total fabric area:\n" << total_area << " m^2." << std::endl;
    std::cout << "Total fabric mass:\n" << inv_mass_.cwiseInverse().sum() << " kg." << std::endl;

    

    // std::cout << "stretching_lengths_:\n" << stretching_lengths_ << std::endl;
    // std::cout << "bending_lengths_:\n" << bending_lengths_ << std::endl;

}

void RigidBody::preSolve(const Real &dt, const Eigen::Matrix<Real,1,3> &gravity){
    #pragma omp parallel default(shared)
    {
        // Semi implicit euler (position)
        // #pragma omp for schedule(static) 
        #pragma omp parallel for 
        for (int i = 0; i< num_particles_; i++){
            if (inv_mass_(i) > 0){
                vel_.row(i) += gravity*dt;
                prev_pos_.row(i) = pos_.row(i);
                pos_.row(i) += vel_.row(i)*dt;

                // Prevent going below ground
                Real z = pos_(i,2);
                if (z < 0.){
                    pos_.row(i) = prev_pos_.row(i) ;
                    pos_(i,2) = 0.0;
                }
            }
        }
    }
}

void RigidBody::solve(const Real &dt){
    // solveStretching(stretching_compliance_,dt);
    // solveBending(bending_compliance_,dt);
}

void RigidBody::postSolve(const Real &dt){
    // Update velocities
    #pragma omp parallel default(shared)
    {
        // Update linear velocities
        // #pragma omp for schedule(static) 
        #pragma omp parallel for 
        for (int i = 0; i< num_particles_; i++){
            if (inv_mass_(i) != 0){
                vel_.row(i) = (pos_.row(i) - prev_pos_.row(i))/dt;
            }
        }
    }

    // Create an artificial global damping
    #pragma omp parallel default(shared)
    {
        // Damp linear velocities
        // #pragma omp for schedule(static) 
        #pragma omp parallel for 
        for (int i = 0; i< num_particles_; i++){
            if (inv_mass_(i) != 0){
                vel_.row(i) -= std::min(1.0, (global_damp_coeff_v_/std::sqrt(num_particles_))*dt*inv_mass_(i)) * vel_.row(i);
                // divide damping coeff by square root of num_particles_ to get rid of the fabric area dependent damping response
            }
        }
    }
}

void RigidBody::resetForces(){
    // Also Reset accumulated forces for the next iteration
    for_.setZero();
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *RigidBody::getPosPtr(){
    return &pos_;
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *RigidBody::getVelPtr(){
    return &vel_;
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *RigidBody::getForPtr(){
    return &for_;
}

Eigen::MatrixX3i *RigidBody::getFaceTriIdsPtr(){
    return &mesh_.face_tri_ids;
}
