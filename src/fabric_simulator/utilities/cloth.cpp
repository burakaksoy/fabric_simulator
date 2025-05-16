/*
 * Author: Burak Aksoy
 */

#include "fabric_simulator/utilities/cloth.h"
#include "fabric_simulator/utilities/collision_handler.h"

using namespace pbd_object;

Cloth::Cloth(){

}

Cloth::Cloth(const Mesh &mesh, 
             const Real &stretching_compliance, 
             const Real &bending_compliance, 
             const Real &density,
             const Real &global_damp_coeff_v,
             const Eigen::Matrix<Real,1,3> &gravity):
    mesh_(mesh),
    stretching_compliance_(stretching_compliance),
    bending_compliance_(bending_compliance),
    density_(density),
    global_damp_coeff_v_(global_damp_coeff_v),
    gravity_(gravity)
{
    num_particles_ = mesh_.vertices.rows();
    std::cout << "num particles: " << num_particles_ << std::endl;
    
    pos_ = mesh_.vertices;
    prev_pos_ = mesh_.vertices;
    rest_pos_ = mesh_.vertices;
    vel_ = Eigen::Matrix<Real,Eigen::Dynamic,3>::Zero(num_particles_,3);
    inv_mass_ = Eigen::Matrix<Real,1,Eigen::Dynamic>::Zero(num_particles_);
    is_dynamic_.assign(num_particles_, true); // initially assume all particles are dynamic
    for_ = Eigen::Matrix<Real,Eigen::Dynamic,3>::Zero(num_particles_,3);
    
    grads_ = Eigen::Matrix<Real,1,3>::Zero();

    // std::cout << "pos_:\n" << pos_ << std::endl;
    // std::cout << "prev_pos_:\n" << prev_pos_ << std::endl;
    // std::cout << "vel_:\n" << vel_ << std::endl;
    // std::cout << "inv_mass_:\n" << inv_mass_ << std::endl;
    
    // Set stretching and bending constraints
    Eigen::RowVectorXi neighbors = findTriNeighbors(mesh_.face_tri_ids);
    int num_tris = mesh_.face_tri_ids.rows();

    std::vector<Eigen::RowVector2i> edge_ids;
    std::vector<Eigen::RowVector4i> tri_pair_ids;
    for (int i = 0; i < num_tris; i++) {
        for (int j = 0; j < 3; j++) {
            int global_edge_nr = 3 * i + j;
            // ids of particle creating that global_edge:
            int id0 = mesh_.face_tri_ids(i,j);
            int id1 = mesh_.face_tri_ids(i,(j + 1) % 3);

            // Each edge only once
            int n = neighbors[global_edge_nr]; // returns -1 if open edge, or positive global edge number of its pair
            if (n < 0 || id0 < id1) {
                Eigen::RowVector2i ids(id0, id1);
                edge_ids.push_back(ids);
            }

            // Tri pair
            if (n >= 0 && id0 < id1) {
                int ni = n / 3;
                int nj = n % 3;
                int id2 = mesh_.face_tri_ids(i,(j + 2) % 3);
                int id3 = mesh_.face_tri_ids(ni,(nj + 2) % 3);
                Eigen::RowVector4i ids(id0, id1, id2, id3);
                tri_pair_ids.push_back(ids); // simple bending constraint needs only id2 and id3, but id0 and id1 also added for a "future" implementation
            }
        }
    }

    // Eigen::Map<Eigen::MatrixX2i> edge_ids_mat((int *)edge_ids.data(), edge_ids.size(), 2);
    Eigen::MatrixXi edge_ids_mat(edge_ids.size(),2);
    for (int i = 0; i < edge_ids.size(); i++) {
        edge_ids_mat.row(i) = edge_ids[i];
    }

    // Eigen::Map<Eigen::MatrixX4i> tri_pair_ids_mat((int *)tri_pair_ids.data(), tri_pair_ids.size(), 4);
    Eigen::MatrixXi tri_pair_ids_mat(tri_pair_ids.size(),4);
    for (int i = 0; i < tri_pair_ids.size(); i++) {
        tri_pair_ids_mat.row(i) = tri_pair_ids[i];
    }

    stretching_ids_ = edge_ids_mat;
    bending_ids_ = tri_pair_ids_mat;

    // std::cout << "stretching_ids_:\n" << stretching_ids_ << std::endl;
    // std::cout << "bending_ids_:\n" << bending_ids_ << std::endl;
    std::cout << "num edges: " << stretching_ids_.rows() << std::endl;

    stretching_lengths_ = Eigen::Matrix<Real,1,Eigen::Dynamic>::Zero(stretching_ids_.rows()); //assigned at initPhysics
    bending_lengths_ = Eigen::Matrix<Real,1,Eigen::Dynamic>::Zero(bending_ids_.rows()); //assigned at initPhysics

    // Is lambda_ necessary? Answer: When using more than 1 steps, yes as explained in the XPBD paper (2016); Not necessary for 1 step and when small substeps method (2019) is used.
    Lambda_stretching_.assign(stretching_ids_.rows(),0.0); 
    Lambda_bending_.assign(bending_ids_.rows(),0.0); 

    // attached_ids_  //Initially empty vector of integers to store the ids of attached (fixed) particles.

    initPhysics(mesh_.face_tri_ids);
}

Cloth::~Cloth(){

}

void Cloth::initPhysics(const Eigen::MatrixX3i &face_tri_ids){
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

    

    for (int i = 0; i < stretching_lengths_.size(); i++){
        int id0 = stretching_ids_(i,0);
        int id1 = stretching_ids_(i,1);
        stretching_lengths_(i) = (pos_.row(id1)-pos_.row(id0)).norm();
    }

    for (int i = 0; i < bending_lengths_.size(); i++){
        int id0 = bending_ids_(i,2);
        int id1 = bending_ids_(i,3);
        bending_lengths_(i) = (pos_.row(id1)-pos_.row(id0)).norm();
    }

    // std::cout << "stretching_lengths_:\n" << stretching_lengths_ << std::endl;
    // std::cout << "bending_lengths_:\n" << bending_lengths_ << std::endl;

}

Eigen::RowVectorXi Cloth::findTriNeighbors(const Eigen::MatrixX3i &face_tri_ids){
    int num_tris = face_tri_ids.rows();

    // create common edges
    std::vector<std::unordered_map<std::string, int>> edges;
    for (int i = 0; i < num_tris; i++) {
        for (int j = 0; j < 3; j++) {
            int id0 = face_tri_ids(i,j);
            int id1 = face_tri_ids(i,(j + 1) % 3);
            edges.push_back({
                {"id0", std::min(id0, id1)},
                {"id1", std::max(id0, id1)},
                {"edgeNr", 3 * i + j}
            });
        }
    }

    // sort so common edges are next to each other
    std::sort(edges.begin(), edges.end(), [](const std::unordered_map<std::string, int> &a, const std::unordered_map<std::string, int> &b) {
        return a.at("id0") < b.at("id0") || (a.at("id0") == b.at("id0") && a.at("id1") < b.at("id1"));
    });

    // find matching edges
    const int fill_value = -1;
    Eigen::RowVectorXi neighbors = Eigen::RowVectorXi::Constant(3*num_tris, fill_value);

    int nr = 0;
    while (nr < edges.size()) {
        std::unordered_map<std::string, int> e0 = edges[nr]; // unordered_map
        nr++;
        if (nr < edges.size()) {
            std::unordered_map<std::string, int> e1 = edges[nr]; // unordered_map
            if (e0.at("id0") == e1.at("id0") && e0.at("id1") == e1.at("id1")) {
                neighbors(e0.at("edgeNr")) = e1.at("edgeNr");
                neighbors(e1.at("edgeNr")) = e0.at("edgeNr");
            }
        }
    }
    
    return neighbors;
}


// Find the nearest 3D position vector row id in the given matrix
int Cloth::findNearestPositionVectorId(const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>& matrix, 
                                        const Eigen::Matrix<Real,3,1>& pos) 
{
    int nearestId = -1;
    Real minDistance = std::numeric_limits<Real>::max();
    for (int i = 0; i < matrix.rows(); ++i) 
    {
        Eigen::Matrix<Real,3,1> currentPos = matrix.row(i);
        Real currentDistance = (currentPos - pos).norm();
        if (currentDistance < minDistance) {
            nearestId = i;
            minDistance = currentDistance;
        }
    }
    return nearestId;
}

void Cloth::findPositionVectorsAndIdsInSphere(const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic> &particlePoses, 
                                                const Real &radius, 
                                                std::vector<int> &ids, 
                                                std::vector<Eigen::Matrix<Real,1,3>> &rel_poses) {
    const int first_id = ids[0];
    const Eigen::Matrix<Real,3,1> center = particlePoses.row(first_id);

    for (int i = 0; i < particlePoses.rows(); ++i) {
        if (i != first_id){
            Eigen::Matrix<Real,3,1> currentPos = particlePoses.row(i).transpose();
            Eigen::Matrix<Real,1,3> relPos = (currentPos - center); 
            Real currentDistance = relPos.norm();
            if (currentDistance < radius) {
                // Only add the particle if it is not already in the list
                if (std::find(ids.begin(), ids.end(), i) == ids.end()) {
                    ids.push_back(i);
                    rel_poses.push_back(relPos);
                }
            }
        }
    }
}

void Cloth::solveStretching(const Real &compliance, const Real &dt){
    Real alpha = compliance / (dt*dt);

    for (int i = 0; i < stretching_lengths_.size(); i++){
        int &id0 = stretching_ids_(i,0);
        int &id1 = stretching_ids_(i,1);

        
        Real &w0 = inv_mass_(id0);
        Real &w1 = inv_mass_(id1);
        Real w = w0 + w1;
        
        if (w == 0){
            continue;
        }

        const bool &isDynamic0 = is_dynamic_[id0];
        const bool &isDynamic1 = is_dynamic_[id1];

        if (!isDynamic0 && !isDynamic1){
            continue;
        }

        // grads_ stores the gradient of the constraint function
        grads_ = pos_.row(id0) - pos_.row(id1);
        Real len = grads_.norm();

        // If the length is too small, set the gradient to zero
        if (len <= static_cast<Real>(1e-13)){
            grads_ = Eigen::Matrix<Real,1,3>::Zero();
        } else {
            grads_ = grads_ / len;
        }

        Real &rest_len = stretching_lengths_(i);
        Real C = len - rest_len; // constraint
        Real K = w+alpha;
        Real &lambda = Lambda_stretching_[i];
        
        // Initialize delta_lambda to zero
        Real delta_lambda = 0.0;
        // Only update lambda if K is not too small
        if (std::fabs(K) > static_cast<Real>(1e-13)){
            delta_lambda = -(C + lambda*alpha) / K; 
        }

        // update lambda
        lambda += delta_lambda; 

        // Position corrections
        if (isDynamic0){
            pos_.row(id0) += delta_lambda*w0*grads_;
        }
        if (isDynamic1){
            pos_.row(id1) += -delta_lambda*w1*grads_;
        }
        
        // Force calculation
        Real f = (lambda / (dt*dt)); 
        for_.row(id0) += f*grads_;
        for_.row(id1) += -f*grads_;
    }
}

void Cloth::solveBending(const Real &compliance, const Real &dt){
    Real alpha = compliance / (dt*dt);

    for (int i = 0; i < bending_lengths_.size(); i++){
        int &id0 = bending_ids_(i,2);
        int &id1 = bending_ids_(i,3);

        Real &w0 = inv_mass_(id0);
        Real &w1 = inv_mass_(id1);
        Real w = w0 + w1;

        if (w == 0){
            continue;
        }

        const bool &isDynamic0 = is_dynamic_[id0];
        const bool &isDynamic1 = is_dynamic_[id1];

        if (!isDynamic0 && !isDynamic1){
            continue;
        }

        // grads_ stores the gradient of the constraint function
        grads_ = pos_.row(id0) - pos_.row(id1);
        Real len = grads_.norm();

        // If the length is too small, set the gradient to zero
        if (len <= static_cast<Real>(1e-13)){
            grads_ = Eigen::Matrix<Real,1,3>::Zero();
        } else {
            grads_ = grads_ / len;
        }

        Real &rest_len = bending_lengths_(i);
        Real C = len - rest_len; // constraint
        Real K = w+alpha;
        Real &lambda = Lambda_bending_[i];
    
        // Initialize delta_lambda to zero
        Real delta_lambda = 0.0;
        // Only update lambda if K is not too small
        if (std::fabs(K) > static_cast<Real>(1e-13)){
            delta_lambda = -(C + lambda*alpha) / K; 
        }

        // update lambda
        lambda += delta_lambda; 

        // Position corrections
        if (isDynamic0){
            pos_.row(id0) += delta_lambda*w0*grads_;
        }
        if (isDynamic1){
            pos_.row(id1) += -delta_lambda*w1*grads_;
        }

        // Force calculation
        Real f = (lambda / (dt*dt)); 
        for_.row(id0) += f*grads_;
        for_.row(id1) += -f*grads_;
    }
}

void Cloth::hangFromCorners(const int &num_corners, std::vector<int>& custom_static_particles_){
    // if num_corners = 0: Do not fix any corners, free fall
    // if num_corners = 1: Fix from 1 corners
    // if num_corners = 2: Fix from 2 corners
    // if num_corners = 3: Fix from 3 corners
    // if num_corners = 4: Fix all corners
    // if num_corners = else: Fix all corners

    Real min_x = std::numeric_limits<Real>::infinity();
    Real max_x = -std::numeric_limits<Real>::infinity();
    Real min_y = std::numeric_limits<Real>::infinity();
    Real max_y = -std::numeric_limits<Real>::infinity();

    for (int i = 0; i < num_particles_; i++) {
        min_x = std::min(min_x, pos_(i,0));
        max_x = std::max(max_x, pos_(i,0));
        min_y = std::min(min_y, pos_(i,1));
        max_y = std::max(max_y, pos_(i,1));
    }

    Real eps = 0.0001;

    for (int i = 0; i < num_particles_; i++) {
        Real x = pos_(i,0);
        Real y = pos_(i,1);

        switch(num_corners) {
            case 0:
                std::cout << "Did not virtually hang from any corners." << std::endl;
                return;
                // break;
            case 1:
                if (y > max_y - eps && x > max_x - eps) {
                    std::cout << "id: " << i << " is virtually hang as corner 1." << std::endl;
                    changeParticleDynamicity(i,false);

                    // Check if value exists in the custom_static_particles_ vector, if not, add it
                    if (std::find(custom_static_particles_.begin(), custom_static_particles_.end(), i) == custom_static_particles_.end()) {
                        custom_static_particles_.push_back(i);
                    }
                }
                break;
            case 2:
                if (y > max_y - eps && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2." << std::endl;
                    changeParticleDynamicity(i,false);

                    // Check if value exists in the custom_static_particles_ vector, if not, add it
                    if (std::find(custom_static_particles_.begin(), custom_static_particles_.end(), i) == custom_static_particles_.end()) {
                        custom_static_particles_.push_back(i);
                    }
                }
                break;
            case 3:
                if ((y > max_y - eps && x < min_x + eps) || (y > max_y - eps && x > max_x - eps) || (y < min_y + eps && x > max_x - eps) ) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2 or 3." << std::endl;
                    changeParticleDynamicity(i,false);

                    // Check if value exists in the custom_static_particles_ vector, if not, add it
                    if (std::find(custom_static_particles_.begin(), custom_static_particles_.end(), i) == custom_static_particles_.end()) {
                        custom_static_particles_.push_back(i);
                    }
                }
                break;
            case 4:
                if ((y < min_y + eps || y > max_y - eps  ) && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2 or 3 or 4." << std::endl;
                    changeParticleDynamicity(i,false);

                    // Check if value exists in the custom_static_particles_ vector, if not, add it
                    if (std::find(custom_static_particles_.begin(), custom_static_particles_.end(), i) == custom_static_particles_.end()) {
                        custom_static_particles_.push_back(i);
                    }
                }
                break;
            default:
                if ((y < min_y + eps || y > max_y - eps  ) && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2 or 3 or 4." << std::endl;
                    changeParticleDynamicity(i,false);

                    // Check if value exists in the custom_static_particles_ vector, if not, add it
                    if (std::find(custom_static_particles_.begin(), custom_static_particles_.end(), i) == custom_static_particles_.end()) {
                        custom_static_particles_.push_back(i);
                    }
                }
                break;
        }
    }
}

void Cloth::changeParticleDynamicity(const int &particle, const bool &is_dynamic, const Eigen::Matrix<Real,1,3>* pos){
    if(particle < 0 || particle >= is_dynamic_.size()) {
        throw std::out_of_range("Index out of bounds");
    }
    if (is_dynamic_[particle] != is_dynamic){
        is_dynamic_[particle] = is_dynamic;

        if (!is_dynamic){
            // Update Velocity to Zero when making the particle static
            updateAttachedVelocity(particle, Eigen::Matrix<Real,1,3>::Zero());
            // Check if pos is provided
            if (pos) {
                updateAttachedPose(particle, *pos);
            }
        }
    }
}

void Cloth::setStaticParticles(const std::vector<int> &particles){
    for (const int& i : particles)
    {
        changeParticleDynamicity(i, false);
    }
}

void Cloth::setDynamicParticles(const std::vector<int> &particles){
    for (const int& i : particles)
    {
        changeParticleDynamicity(i, true);
    }
}

const bool Cloth::isStaticParticle(const int &particle) const{
    if(particle < 0 || particle >= is_dynamic_.size()) {
        throw std::out_of_range("Index out of bounds");
    }
    return !is_dynamic_[particle];
}

const bool Cloth::isDynamicParticle(const int &particle) const{
    if(particle < 0 || particle >= is_dynamic_.size()) {
        throw std::out_of_range("Index out of bounds");
    }
    return is_dynamic_[particle];
}

void Cloth::setStretchingCompliance(const Real &stretching_compliance){
    stretching_compliance_ = stretching_compliance;
}

void Cloth::setBendingCompliance(const Real &bending_compliance){
    bending_compliance_ = bending_compliance;
}

const Real Cloth::getStretchingCompliance(){
    return stretching_compliance_;
}

const Real Cloth::getBendingCompliance(){
    return bending_compliance_;
}

const Real Cloth::getDensity(){
    return density_;
}

const Eigen::Matrix<Real,1,3> Cloth::getGravity(){
    return gravity_;
}

void Cloth::preSolve(const Real &dt, const Eigen::Matrix<Real,1,3> &gravity){
    #pragma omp parallel default(shared)
    {
        // Semi implicit euler (position)
        // #pragma omp for schedule(static) 
        #pragma omp parallel for 
        for (int i = 0; i< num_particles_; i++){
            if (is_dynamic_[i]){
                vel_.row(i) += gravity*dt;
            }
                prev_pos_.row(i) = pos_.row(i);
                pos_.row(i) += vel_.row(i)*dt;
        }
    }
}

void Cloth::solve(const Real &dt){
    solveStretching(stretching_compliance_,dt);
    solveBending(bending_compliance_,dt);
}

void Cloth::postSolve(const Real &dt){
    // Update velocities
    #pragma omp parallel default(shared)
    {
        // Update linear velocities
        // #pragma omp for schedule(static) 
        #pragma omp parallel for 
        for (int i = 0; i< num_particles_; i++){
            if (is_dynamic_[i]){
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
            if (is_dynamic_[i]){
                vel_.row(i) -= std::min(1.0, (global_damp_coeff_v_/std::sqrt(num_particles_))*dt*inv_mass_(i)) * vel_.row(i);
                // divide damping coeff by square root of num_particles_ to get rid of the fabric area dependent damping response
            }
        }
    }
}

void Cloth::resetForces(){
    // Also Reset accumulated forces for the next iteration
    for_.setZero();
}

void Cloth::resetLambdas(){
    // Reset the lambdas for the next iteration
    // for (auto& lambda : Lambda_stretching_) {
    //     lambda.setZero();
    // }
    Lambda_stretching_.assign(stretching_ids_.rows(),0.0); 
    Lambda_bending_.assign(bending_ids_.rows(),0.0); 
}

int Cloth::attachNearest(const Eigen::Matrix<Real,1,3> &pos){
    int id = findNearestPositionVectorId(pos_,pos);
    // Make that particle stationary
    if (id >= 0){
        changeParticleDynamicity(id,false);
    }
    return id;
}

void Cloth::updateAttachedPose(const int &id, const Eigen::Matrix<Real,1,3> &pos){
    pos_.row(id) = pos;
}


void Cloth::attachNearestWithRadius(const Eigen::Matrix<Real,1,3> &pos, const Real &r, 
                                    std::vector<int> &ids,
                                    std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                                    bool is_attach){
    // Attaches the particles that is inside a sphere defined with radius r parallel with center pos
    // Edits:
    // std::vector<int> ids;
    // std::vector<Eigen::Matrix<Real,1,3>> rel_poses; 

    int id = findNearestPositionVectorId(pos_,pos);
    if (id != -1)
    {
        // Only push back if id is not already in the vector
        if (std::find(ids.begin(), ids.end(), id) == ids.end()) {
            ids.push_back(id);
            rel_poses.push_back(Eigen::Matrix<Real,1,3>(0.0, 0.0, 0.0));
        }

        // if (r <= 0), Only attach the nearest to given pose
        if (r > 0){ // Attach all within the specified circle
            // Find all the ids within the radius and their relative poses
            findPositionVectorsAndIdsInSphere(pos_, r, ids, rel_poses);
        }

        if (is_attach){
            // Make those particles stationary
            setStaticParticles(ids);
        }
        else{
            // Make those particles dynamic
            setDynamicParticles(ids);
        }
    }
}

// void Cloth::attachWithinRadius(const Eigen::Matrix<Real,1,3> &pos, const Real &r, 
//                                     std::vector<int> &ids,
//                                     std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
//                                     bool is_attach){
//     // Attaches the particles that is inside a sphere defined with radius r parallel with center pos
//     // Edits:
//     // std::vector<int> ids;
//     // std::vector<Eigen::Matrix<Real,1,3>> rel_poses; 

//     // if (r <= 0), Only attach the nearest to given pose
//     if (r > 0){ // Attach all within the specified circle
//         // Find all the ids within the radius around the given pos and their relative poses
//         // findPositionVectorsAndIdsInSphere(pos_, r, ids, rel_poses);
//         for (int i = 0; i < pos_.rows(); i++) {
//             Eigen::Matrix<Real,1,3> currentPos = pos_.row(i).transpose();
//             Eigen::Matrix<Real,1,3> relPos = (currentPos - pos); 
//             Real currentDistance = relPos.norm();
//             if (currentDistance < r) {
//                 // Only add the particle if it is not already in the list
//                 if (std::find(ids.begin(), ids.end(), i) == ids.end()) {
//                     ids.push_back(i);
//                     rel_poses.push_back(relPos);
//                 }
//             }
//         }
//     }

//     if (is_attach){
//         // Make those particles stationary
//         setStaticParticles(ids);
//     }
//     else{
//         // Make those particles dynamic
//         setDynamicParticles(ids);
//     }
// }

void Cloth::attachWithinRadius(const Eigen::Matrix<Real,1,3> &pos,
                               const Real &r,
                               std::vector<int> &ids,
                               std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                               bool is_attach)
{
    bool nearest_only = false;

    ids.clear();
    rel_poses.clear();

    int  best_id   = -1;
    Real best_dist = std::numeric_limits<Real>::max();
    Eigen::Matrix<Real,1,3> best_rel = Eigen::Matrix<Real,1,3>::Zero();

    const Real r2 = r*r;          // save a sqrt

    for (int i = 0; i < pos_.rows(); ++i)
    {
        // ---- keep everything as 1×3 row vectors ----
        Eigen::Matrix<Real,1,3> cur  = pos_.row(i);   // 1×3
        Eigen::Matrix<Real,1,3> rel  = cur - pos;     // 1×3  (OK)
        Real dist2 = rel.squaredNorm();

        // inside sphere ?  (if r<=0 => accept every particle, we'll filter later)
        if (r <= Real(0) || dist2 < r2)
        {
            if (nearest_only)
            {
                if (dist2 < best_dist)
                {
                    best_dist = dist2;
                    best_id   = i;
                    best_rel  = rel;
                }
            }
            else
            {
                ids.push_back(i);
                rel_poses.push_back(rel);
            }
        }
    }

    if (nearest_only && best_id != -1)
    {
        ids.push_back(best_id);
        rel_poses.push_back(best_rel);
    }

    // ------------------------------------------------
    if (is_attach)
        setStaticParticles(ids);
    else
        setDynamicParticles(ids);
}

void Cloth::attachWithinRadius(const Eigen::Matrix<Real,1,3> &pos, const Real &r, 
                                    std::vector<int> &ids,
                                    std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                                    std::unordered_set<int> &sticked_ids,
                                    bool is_attach){
    // Attaches/Detaches the particles that is inside a sphere defined with radius r parallel with center pos


    // if (r <= 0), Only attach the nearest to given pose
    if (r > 0){ // Attach all within the specified circle
        // Find all the ids within the radius around the given pos and their relative poses
        // findPositionVectorsAndIdsInSphere(pos_, r, ids, rel_poses);
        for (int i = 0; i < pos_.rows(); i++) {
            Eigen::Matrix<Real,1,3> currentPos = pos_.row(i).transpose();
            Eigen::Matrix<Real,1,3> relPos = (currentPos - pos); 
            Real currentDistance = relPos.norm();
            if (currentDistance < r) {
                // Only add the particle if it is not already in the list
                if (std::find(ids.begin(), ids.end(), i) == ids.end()) {
                    ids.push_back(i);
                    rel_poses.push_back(relPos);
                }
            }
        }
    }

    // Add attached ids to the sticked_ids unordered set
    if (is_attach) {
        for (int i = 0; i < ids.size(); ++i) {
            // If the id is not already in the set, add it
            // otherwise, remove it from the ids and rel_poses vectors
            if (sticked_ids.find(ids[i]) == sticked_ids.end()) {
                sticked_ids.insert(ids[i]);
            } else {
                ids.erase(ids.begin() + i);
                rel_poses.erase(rel_poses.begin() + i);
                i--;
            }
        }
    } else { // Detach
        for (int i = 0; i < ids.size(); ++i) {
            // If the id is not already in the set, remove it from the ids and rel_poses vectors
            // otherwise, remove it from the sticked_ids set
            if (sticked_ids.find(ids[i]) == sticked_ids.end()) {
                ids.erase(ids.begin() + i);
                rel_poses.erase(rel_poses.begin() + i);
                i--;
            } else {
                sticked_ids.erase(ids[i]);
            }
        }
    }

    if (is_attach){
        // Make those particles stationary
        setStaticParticles(ids);
    }
    else{ // Detach
        // Make those particles dynamic
        setDynamicParticles(ids);
    }
}

void Cloth::attachToRigidBodySurfaceWithinRadius(const Eigen::Matrix<Real,1,3> &centerWorld,
                                                    const Real radius,
                                                    std::vector<int> &ids,
                                                    std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                                                    std::unordered_set<int> &sticked_ids,
                                                    bool is_attach,
                                                    utilities::CollisionHandler *collisionHandler,
                                                    Real snapDistanceThreshold)
{
    // 1) Gather candidate indices
    ids.clear();
    rel_poses.clear();

    for (int i = 0; i < pos_.rows(); ++i)
    {
        // skip if already attached and is_attach is true, etc.
        if (!is_attach && sticked_ids.find(i) == sticked_ids.end())
            continue;
        // Distance check
        Eigen::Matrix<Real,1,3> curPos = pos_.row(i);
        Real dist = (curPos - centerWorld).norm();
        if (dist <= radius)
        {
            ids.push_back(i);
            // we'll fill rel_poses after we know the final projected position
            rel_poses.emplace_back(Eigen::Matrix<Real,1,3>::Zero());
        }
    }

    // 2) Depending on attach or detach
    if (is_attach)
    {
        // For each candidate in 'ids', project onto the nearest surface
        for (size_t k = 0; k < ids.size(); ++k)
        {
            int pid = ids[k];
            Eigen::Matrix<Real,1,3> oldPos = pos_.row(pid);

            utilities::CollisionHandler::NearestSurfaceData proj;
            bool ok = collisionHandler->projectPointOnRigidBodySurface(oldPos.transpose(), proj);
            if (ok && proj.distance < snapDistanceThreshold)
            {
                // Move cloth particle to that surface
                pos_.row(pid) = proj.closestPointWorld.transpose();

                // record relative pose if needed:
                // relPose = (particleWorldPos - centerWorld)
                rel_poses[k] = pos_.row(pid) - centerWorld;

                // if not already in sticked_ids, add it
                if (sticked_ids.find(pid) == sticked_ids.end())
                {
                    sticked_ids.insert(pid);
                }
            }
            else
            {
                // Could not project or too far => optionally skip or do something else
                // e.g. remove from the final ids so we don't freeze it
                ids[k] = -1; // mark invalid
            }
        }

        // Erase any invalid ones
        // or rebuild the vectors ignoring indices == -1
        std::vector<int> valid_ids;
        std::vector<Eigen::Matrix<Real,1,3>> valid_relposes;
        for (size_t k=0; k<ids.size(); ++k)
        {
            if (ids[k] >= 0)
            {
                valid_ids.push_back(ids[k]);
                valid_relposes.push_back(rel_poses[k]);
            }
        }
        ids = valid_ids;
        rel_poses = valid_relposes;

        // Finally, freeze them
        setStaticParticles(ids);
    }
    else
    {
        // Detach the ones that are currently in 'sticked_ids' and in 'ids'
        // then set them dynamic
        std::vector<int> toDetach;
        for (int pid : ids)
        {
            if (sticked_ids.find(pid) != sticked_ids.end())
            {
                sticked_ids.erase(pid);
                toDetach.push_back(pid);
            }
        }
        setDynamicParticles(toDetach);
    }
}

void Cloth::updateAttachedPoses(const std::vector<int> &ids,
                                const Eigen::Matrix<Real,1,3> &pos,
                                const std::vector<Eigen::Matrix<Real,1,3>> &rel_poses,
                                const Eigen::Quaternion<Real> &cur_orient,
                                const Eigen::Quaternion<Real> &init_orient){
    // ids: 
    // pos: new pose of the first id
    // rel_poses: 
    
    Eigen::Matrix<Real,3,3> R_cur  =  cur_orient.normalized().toRotationMatrix();
    Eigen::Matrix<Real,3,3> R_init = init_orient.normalized().toRotationMatrix();
    Eigen::Matrix<Real,3,3> R_ic = R_init.transpose()*R_cur; // init to current

    for (int i = 0; i < ids.size(); ++i) {
        updateAttachedPose(ids[i], pos+(R_ic*rel_poses[i].transpose()).transpose());
    }
}

void Cloth::updateAttachedVelocity(const int &id, 
                                    const Eigen::Matrix<Real,1,3> &vel){
    vel_.row(id) = vel;
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *Cloth::getPosPtr(){
    return &pos_;
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *Cloth::getPrevPosPtr(){
    return &prev_pos_;
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *Cloth::getVelPtr(){
    return &vel_;
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *Cloth::getForPtr(){
    return &for_;
}

Eigen::Matrix<Real,1,Eigen::Dynamic> *Cloth::getInvMassPtr(){
    return &inv_mass_;
}

Eigen::Matrix<Real,1,Eigen::Dynamic> *Cloth::getStretchingLengthsPtr(){
    return &stretching_lengths_;
}

Eigen::Matrix<Real,1,Eigen::Dynamic> *Cloth::getBendingLengthsPtr(){
    return &bending_lengths_;
}

Eigen::MatrixX2i *Cloth::getStretchingIdsPtr(){
    return &stretching_ids_;
}

Eigen::MatrixX4i *Cloth::getBendingIdsPtr(){
    return &bending_ids_;
}

Eigen::MatrixX3i *Cloth::getFaceTriIdsPtr(){
    return &mesh_.face_tri_ids;
}

std::vector<int> *Cloth::getAttachedIdsPtr(){
    return &attached_ids_;
}