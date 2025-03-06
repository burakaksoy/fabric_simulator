/*
 * Author: Burak Aksoy
 */

#ifndef CLOTH_H
#define CLOTH_H


#include "fabric_simulator/Common.h"

// #include "fabric_simulator/utilities/collision_handler.h"

namespace utilities {
    class CollisionHandler; // forward declare
}

namespace pbd_object
{

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

    void changeParticleDynamicity(const int &particle, const bool &is_dynamic, const Eigen::Matrix<Real,1,3>* pos = nullptr);

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

    void attachToRigidBodySurfaceWithinRadius(const Eigen::Matrix<Real,1,3> &centerWorld,
                                                const Real radius,
                                                std::vector<int> &ids,
                                                std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                                                std::unordered_set<int> &sticked_ids,
                                                bool is_attach,
                                                utilities::CollisionHandler *collisionHandler,
                                                Real snapDistanceThreshold);

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