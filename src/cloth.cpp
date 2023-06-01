/*
 * Author: Burak Aksoy
 */

#include "fabric_simulator/utilities/cloth.h"

using namespace pbd_object;

Cloth::Cloth(){

}

Cloth::Cloth(const Mesh &mesh, 
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

    // attached_ids_  //Initially empty vector of integers to store the ids of attached (fixed) particles.

    // Not necessary? 
    // only_once 

    initPhysics(mesh_.face_tri_ids);
}

Cloth::~Cloth(){

}

void Cloth::initPhysics(const Eigen::MatrixX3i &face_tri_ids){
    int num_tris = face_tri_ids.rows();

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
    }

    // std::cout << "inv_mass_:\n" << inv_mass_ << std::endl;
    // std::cout << "particle masses:\n" << inv_mass_.cwiseInverse() << " kg." << std::endl;
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
int Cloth::findNearestPositionVectorId(const Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>& matrix, const Eigen::Matrix<Real,3,1>& pos) {
  int nearestId = -1;
  Real minDistance = std::numeric_limits<Real>::max();
  for (int i = 0; i < matrix.rows(); ++i) {
    Eigen::Matrix<Real,3,1> currentPos = matrix.row(i);
    Real currentDistance = (currentPos - pos).norm();
    if (currentDistance < minDistance) {
      nearestId = i;
      minDistance = currentDistance;
    }
  }
  return nearestId;
}

void Cloth::solveStretching(const Real &compliance, const Real &dt){
    Real alpha = compliance / (dt*dt);

    for (int i = 0; i < stretching_lengths_.size(); i++){
        int id0 = stretching_ids_(i,0);
        int id1 = stretching_ids_(i,1);

        Real w0 = inv_mass_(id0);
        Real w1 = inv_mass_(id1);
        Real w = w0 + w1;

        if (w == 0){
            continue;
        }

        grads_ = pos_.row(id0) - pos_.row(id1);

        Real len = grads_.norm();
        if (len <= static_cast<Real>(1e-13)){
            continue;
        }

        grads_ = grads_ / len;

        // std::cout << "grads_: " << grads_ << std::endl;

        Real rest_len = stretching_lengths_(i);
        Real C = len - rest_len;
        Real K = w+alpha;
        
        if (std::fabs(K) <= static_cast<Real>(1e-13)){
            continue;
        } 

        Real s = -C / K; // lambda

        // Position corrections
        pos_.row(id0) += s*w0*grads_;
        pos_.row(id1) += -s*w1*grads_;
        

        // Force calculation
        Real f = (s / (dt*dt)); 
        for_.row(id0) += f*grads_;
        for_.row(id1) += -f*grads_;
    }
}

void Cloth::solveBending(const Real &compliance, const Real &dt){
    Real alpha = compliance / (dt*dt);

    for (int i = 0; i < bending_lengths_.size(); i++){
        int id0 = bending_ids_(i,2);
        int id1 = bending_ids_(i,3);

        Real w0 = inv_mass_(id0);
        Real w1 = inv_mass_(id1);
        Real w = w0 + w1;

        if (w == 0){
            continue;
        }

        grads_ = pos_.row(id0) - pos_.row(id1);

        Real len = grads_.norm();
        if (len < static_cast<Real>(1e-13)){
            continue;
        }

        grads_ = grads_ / len;

        Real rest_len = bending_lengths_(i);
        Real C = len - rest_len;
        Real K = w+alpha;
    
        if (std::fabs(K) <= static_cast<Real>(1e-13)){
            continue;
        } 

        Real s = -C / K; // lambda

        // Position corrections
        pos_.row(id0) += s*w0*grads_;
        pos_.row(id1) += -s*w1*grads_;
    }
}

void Cloth::hangFromCorners(const int &num_corners){
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
                    inv_mass_(i) = 0.0;
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
            case 2:
                if (y > max_y - eps && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2." << std::endl;
                    inv_mass_(i) = 0.0;
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
            case 3:
                if ((y > max_y - eps && x < min_x + eps) || (y > max_y - eps && x > max_x - eps) || (y < min_y + eps && x > max_x - eps) ) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2 or 3." << std::endl;
                    inv_mass_(i) = 0.0;
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
            case 4:
                if ((y < min_y + eps || y > max_y - eps  ) && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2 or 3 or 4." << std::endl;
                    inv_mass_(i) = 0.0;
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
            default:
                if ((y < min_y + eps || y > max_y - eps  ) && (x < min_x + eps || x > max_x - eps)) {
                    std::cout << "id: " << i << " is virtually hang as corners 1 or 2 or 3 or 4." << std::endl;
                    inv_mass_(i) = 0.0;
                    attached_ids_.push_back(i); // add fixed particle id to the attached_ids_ vector
                }
                break;
        }
    }
}

void Cloth::preSolve(const Real &dt, const Eigen::Matrix<Real,1,3> &gravity){
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

void Cloth::resetForces(){
    // Also Reset accumulated forces for the next iteration
    for_.setZero();
}

int Cloth::attachNearest(const Eigen::Matrix<Real,1,3> &pos){
    int id = findNearestPositionVectorId(pos_,pos);
    // Make that particle stationary
    if (id >= 0){
        inv_mass_(id) = 0.0;
        attached_ids_.push_back(id); // add fixed particle id to the attached_ids_ vector
    }
    return id;
}

void Cloth::updateAttachedPose(const int &id, const Eigen::Matrix<Real,1,3> &pos){
    pos_.row(id) = pos;
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *Cloth::getPosPtr(){
    return &pos_;
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *Cloth::getVelPtr(){
    return &vel_;
}

Eigen::Matrix<Real,Eigen::Dynamic,3> *Cloth::getForPtr(){
    return &for_;
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

std::vector<int> *Cloth::getAttachedIdsPtr(){
    return &attached_ids_;
}