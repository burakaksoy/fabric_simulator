/*
 * Author: Burak Aksoy
 */

#ifndef FABRIC_SIMULATOR_H
#define FABRIC_SIMULATOR_H

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>

#include <fabric_simulator/MinDistanceDataArray.h> 
#include <fabric_simulator/ChangeParticleDynamicity.h> 
#include <fabric_simulator/SegmentStateArray.h> 
#include <fabric_simulator/FixNearestFabricParticleRequest.h> 
#include <fabric_simulator/AttachExternalOdomFrameRequest.h> 

#include <std_srvs/Empty.h>
#include <fabric_simulator/SetParticleDynamicity.h> // service
#include <fabric_simulator/SetFabricStretchingCompliance.h> // service
#include <fabric_simulator/GetFabricStretchingCompliance.h> // service
#include <fabric_simulator/SetFabricBendingCompliance.h> // service
#include <fabric_simulator/GetFabricBendingCompliance.h> // service
#include <fabric_simulator/EnableCollisionHandling.h> // service
#include <fabric_simulator/FixNearestFabricParticle.h> // service
#include <fabric_simulator/AttachExternalOdomFrame.h> // service

#include <time.h>
#include <math.h>
#include <numeric>
#include <vector>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <scipy/spatial.h>

// #include <thread>
// #include <mutex>
#include <boost/thread/recursive_mutex.hpp>

#include <string>
#include <iostream>

#include <fstream>
#include <sstream>

#include "fabric_simulator/utilities/cloth.h"
#include "fabric_simulator/utilities/rigid_body_scene_loader.h"
#include "fabric_simulator/utilities/collision_handler.h"

#include <memory> // needed for std::unique_ptr and std::make_unique

namespace fabric_simulator
{

struct ExternalOdomAttachment
{
    // The topic on which we listen for Odometry
    std::string odom_topic;
    
    // The subscription to that odom topic
    ros::Subscriber odom_sub;
    
    // The wrench-stamped publisher and the frame to use for that wrench
    std::string wrench_frame_id;
    ros::Publisher wrench_pub;
    
    // Attachment data
    bool is_attached{false};          // whether or not the fabric is actually attached
    int attached_id{-1};              // single attached particle if you do single ID attach
    std::vector<int> attached_ids;    // if you do multi-ID attach
    std::vector<Eigen::Matrix<Real,1,3>> attached_rel_poses;
    Eigen::Matrix<Real,1,3> attached_force{0,0,0};
    // TODO: Add torque as well
    Eigen::Quaterniond attached_orient{1,0,0,0}; // last known orientation, for example
};

class FabricSimulator
{
    // friend class utilities::CollisionHandler; // Allow CollisionHandler to access all private data of the fabric simulator
public:
    FabricSimulator(ros::NodeHandle &nh, ros::NodeHandle &nh_local, boost::recursive_mutex &mtx);
    ~FabricSimulator();

    pbd_object::Cloth& getFabric() { return fabric_; }
    std::vector<utilities::RigidBodySceneLoader::RigidBodyData>& getRigidBodies() { return rigid_bodies_; }

private:
    // Functions ---------------------------------
    
    // Create a mesh structure with face triangle ids and vertice for particle initial poses
    // face triangle ids; vector with n elements with 3d integer rowvectors. each row is 3 particle id of a triangle
    // vertice: vector with m elements with 3d row vectors. each row is 3D xyz position of a particle.
    pbd_object::Mesh createMeshRectangular(const std::string &name, 
                                        const Real &fabric_x, 
                                        const Real &fabric_y, 
                                        const Real &fabric_z, 
                                        const Real &fabric_res);

    pbd_object::Mesh loadMesh(const std::string &name, const std::string &path, const Real &fabric_z);

    pbd_object::Mesh transformMesh(const pbd_object::Mesh &mesh, 
                                   const std::vector<Real> &translation,
                                   const std::vector<Real> &rotationAxis,
                                   const Real &rotationAngle,
                                   const std::vector<Real> &scale);

    pbd_object::Mesh transformMesh(const pbd_object::Mesh &mesh, 
                                   const Eigen::Matrix<Real, 1, 3> &translation,
                                   const Eigen::Quaternion<Real> &rotation,
                                   const Eigen::Matrix<Real, 1, 3> &scale);

    void readAttachedRobotForces();

    // Creates the markers to publish
    void createRvizPointsMarker(const Eigen::Matrix<Real,Eigen::Dynamic,3> *poses, 
                                visualization_msgs::Marker &marker);
    void createRvizWireframeMarker(const Eigen::Matrix<Real,Eigen::Dynamic,3> *poses, 
                                const Eigen::MatrixX2i *ids, 
                                visualization_msgs::Marker &marker);
    void createRvizTrianglesMarker(const Eigen::Matrix<Real,Eigen::Dynamic,3> *poses, 
                                    const Eigen::MatrixX3i *face_tri_ids, 
                                    visualization_msgs::Marker &marker);

    void publishFaceTriIds(const Eigen::MatrixX3i *ids);

    void createMeshAndWireframeMarkers(const Eigen::Matrix<Real,Eigen::Dynamic,3> *vertices, 
                                        const Eigen::MatrixX3i *face_tri_ids, 
                                        visualization_msgs::Marker &meshMarker, 
                                        visualization_msgs::Marker &wireframeMarker);
    void setupMeshMarker(visualization_msgs::Marker &marker);
    void setupWireframeMarker(visualization_msgs::Marker &marker);

    void setupMinDistLineMarker(visualization_msgs::Marker &marker);

    void drawRvizMesh_from_resource(const std::string &mesh_resource); // This is not used however put here as a future reference

    // Timer callback functions
    void simulate(const ros::TimerEvent& e);
    void render(const ros::TimerEvent& e);

    void renderMarkers(const Eigen::Matrix<Real,Eigen::Dynamic,3>* pos_ptr,
                        const Eigen::MatrixX2i* stretching_ids_ptr,
                        const Eigen::MatrixX3i* face_tri_ids_ptr,
                        ros::Publisher& publisher,
                        int marker_id);

    void publishWrenchesOnExternalOdoms(const ros::TimerEvent& e);
    void publishZeroWrenches();
    void renderRigidBodies(const ros::TimerEvent& e);

    void publishMinDistancesToRigidBodies(const ros::TimerEvent& e);
    void publishFabricState(const ros::TimerEvent& e);

    void publishMinDistLineMarkers(const std::vector<std::vector<utilities::CollisionHandler::MinDistanceData>>& min_distances_mt);

    // Odometry callback functions
    void odometryCb_01(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void odometryCb_02(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void odometryCb_03(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void odometryCb_04(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void odometryCb_custom_static_particles(const nav_msgs::Odometry::ConstPtr& odom_msg, const int& id);
    void cmdVelCb_custom_static_particles(const geometry_msgs::Twist::ConstPtr& twist_msg, const int& id);

    void odometryCb_external(const nav_msgs::Odometry::ConstPtr& odom_msg, const std::string& topic);

    // Change Dynamicity callback function
    void changeParticleDynamicityCb(const fabric_simulator::ChangeParticleDynamicity::ConstPtr& change_particle_dynamicity_msg);
    bool updateParticleDynamicityCommon(int particle_id, bool is_dynamic);

    void fixNearestFabricParticleRequestCb(const fabric_simulator::FixNearestFabricParticleRequest::ConstPtr& fix_nearest_fabric_particle_request_msg);
    bool fixNearestFabricParticleCommon(bool is_fix, const geometry_msgs::PoseStamped &pose);

    void attachExternalOdomFrameRequestCb(const fabric_simulator::AttachExternalOdomFrameRequest::ConstPtr& attach_externa_odom_frame_request_msg);
    bool attachExternalOdomFrameCommon(const std::string & odom_topic, bool is_attach);

    // Change Stretching Compliance and Bending Compliance callback functions
    void changeStretchingComplianceCb(const std_msgs::Float32::ConstPtr& stretching_compliance_msg);
    void changeBendingComplianceCb(const std_msgs::Float32::ConstPtr& bending_compliance_msg);

    // Service functions
    //   void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }
    bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void reset();
    void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

    bool setParticleDynamicityCallback(fabric_simulator::SetParticleDynamicity::Request &req,
        fabric_simulator::SetParticleDynamicity::Response &res);

    bool fixNearestFabricParticleCallback(fabric_simulator::FixNearestFabricParticle::Request &req, 
        fabric_simulator::FixNearestFabricParticle::Response &res);
    
    bool attachExternalOdomFrameCallback(fabric_simulator::AttachExternalOdomFrame::Request &req, 
        fabric_simulator::AttachExternalOdomFrame::Response &res);

    // Create setStretchingCompliance service callback and setBendingCompliance service callback
    bool setFabricStretchingComplianceCallback(fabric_simulator::SetFabricStretchingCompliance::Request &req,
            fabric_simulator::SetFabricStretchingCompliance::Response &res);
    bool setFabricBendingComplianceCallback(fabric_simulator::SetFabricBendingCompliance::Request &req,
                    fabric_simulator::SetFabricBendingCompliance::Response &res);

    // Create getStretchingCompliance service callback and getBendingCompliance service callback
    bool getFabricStretchingComplianceCallback(fabric_simulator::GetFabricStretchingCompliance::Request &req,
                fabric_simulator::GetFabricStretchingCompliance::Response &res);
    bool getFabricBendingComplianceCallback(fabric_simulator::GetFabricBendingCompliance::Request &req,
                    fabric_simulator::GetFabricBendingCompliance::Response &res);

    bool enableCollisionHandlingCallback(fabric_simulator::EnableCollisionHandling::Request &req,
                    fabric_simulator::EnableCollisionHandling::Response &res);

    // ROS variables---------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    boost::recursive_mutex &mtx_;

    ros::Publisher pub_fabric_state_;
    ros::Publisher pub_fabric_marker_array_;

    ros::Publisher pub_face_tri_ids_;

    ros::Publisher pub_wrench_stamped_01_;
    ros::Publisher pub_wrench_stamped_02_;
    ros::Publisher pub_wrench_stamped_03_;
    ros::Publisher pub_wrench_stamped_04_;

    ros::Publisher pub_rb_marker_array_;

    ros::Publisher pub_min_dist_to_rb_;
    ros::Publisher pub_min_dist_marker_array_;

    ros::ServiceServer params_srv_;
    
    ros::Subscriber sub_odom_01_;
    ros::Subscriber sub_odom_02_;
    ros::Subscriber sub_odom_03_;
    ros::Subscriber sub_odom_04_;

    ros::Subscriber sub_change_particle_dynamicity_;
    ros::ServiceServer set_particle_dynamicity_srv_;

    ros::Subscriber sub_fix_nearest_fabric_particle_request_;
    ros::ServiceServer fix_nearest_fabric_particle_srv_;

    ros::Subscriber sub_attach_external_odom_frame_request_;
    ros::ServiceServer attach_external_odom_frame_srv_;

    ros::Subscriber sub_change_stretching_compliance_;
    ros::Subscriber sub_change_bending_compliance_;
    ros::ServiceServer set_stretching_compliance_srv_;
    ros::ServiceServer set_bending_compliance_srv_;
    ros::ServiceServer get_stretching_compliance_srv_;
    ros::ServiceServer get_bending_compliance_srv_;

    ros::ServiceServer enable_collision_handling_srv_;

    // Map to hold particle ID and its corresponding subscriber
    std::map<int, ros::Subscriber> custom_static_particles_odom_subscribers_;
    std::map<int, ros::Subscriber> custom_static_particles_cmd_vel_subscribers_;

    std::map<std::string, ExternalOdomAttachment> external_odom_attachments_;

    ros::Timer timer_render_;
    ros::Timer timer_simulate_;
    ros::Timer timer_wrench_pub_;
    ros::Timer timer_render_rb_;
    ros::Timer timer_min_dist_to_rb_pub_; // renderMinDistancesToRigidBodies
    ros::Timer timer_fabric_state_pub_; // 

    // ROS Parameters
    bool p_active_;
    bool p_reset_;

    Real gravity_x_;
    Real gravity_y_;
    Real gravity_z_;
    
    Real dt_; // simulation time step size
    bool set_sim_rate_auto_; //param to set the simulation rate and dt automatically

    int num_substeps_;
    int num_steps_;

    bool is_collision_handling_enabled_;
    bool visualize_min_distances_;

    Real contact_tolerance_;
    Real contact_sdf_domain_offset_;

    int fabric_visualization_mode_;
    int rb_visualization_mode_;

    std::string fabric_mesh_path_;
    
    // Fabric mesh properties: (Assuming fabric is rectangular)
    Real fabric_x_; // expansion in x dimension (m)
    Real fabric_y_; // expansion in y dimension (m)
    Real fabric_resolution_; // particle resolution per meter

    Real fabric_density_; // fabric mass per meter square (kg/m^2)

    Real fabric_stretching_compliance_;
    Real fabric_bending_compliance_;
    
    Real initial_height_; // initial fabric height from ground (m)
    std::vector<Real> fabric_translation_;
    std::vector<Real> fabric_rotationAxis_;
    Real fabric_rotationAngle_;
    std::vector<Real> fabric_scale_;

    int num_hang_corners_; // num of corners to hang fabric from (options: 0,1,2,3,4)
    std::vector<int> custom_static_particles_; // particle ids to set as static

    std::string custom_static_particles_odom_topic_prefix_;
    std::string custom_static_particles_cmd_vel_topic_prefix_;

    Real global_damp_coeff_v_; 
    
    Real simulation_rate_;
    Real rendering_rate_;

    Real wrench_pub_rate_;
    Real rendering_rb_rate_;
    Real min_dist_to_rb_pub_rate_; 
    Real fabric_state_pub_rate_; 

    std::string rb_scene_config_path_;

    std::string fabric_state_topic_name_;
    std::string fabric_markers_topic_name_;
    std::string fabric_frame_id_;

    std::string face_tri_ids_topic_name_;

    std::string odom_01_topic_name_;
    std::string odom_02_topic_name_; 
    std::string odom_03_topic_name_; 
    std::string odom_04_topic_name_;

    std::string wrench_01_topic_name_;
    std::string wrench_02_topic_name_; 
    std::string wrench_03_topic_name_; 
    std::string wrench_04_topic_name_;

    std::string wrench_01_frame_id_;
    std::string wrench_02_frame_id_; 
    std::string wrench_03_frame_id_; 
    std::string wrench_04_frame_id_;

    std::string min_dist_to_rb_topic_name_;
    std::string min_dist_markers_topic_name_;

    std::string change_particle_dynamicity_topic_name_;
    std::string set_particle_dynamicity_service_name_;

    std::string fix_nearest_fabric_particle_topic_name_;
    std::string fix_nearest_fabric_particle_service_name_;

    std::string attach_external_odom_frame_topic_name_;
    std::string attach_external_odom_frame_service_name_;

    std::string set_fabric_stretching_compliance_service_name_;
    std::string set_fabric_bending_compliance_service_name_;
    std::string get_fabric_stretching_compliance_service_name_;
    std::string get_fabric_bending_compliance_service_name_;
    std::string change_fabric_stretching_compliance_topic_name_;
    std::string change_fabric_bending_compliance_topic_name_;

    std::string enable_collision_handling_service_name_;
    
    Real fabric_rob_z_offset_; // Additional  attachment height to robots

    Real robot_attach_radius_;

    // Fabric visualization parameters 
    Real point_marker_scale_;

    std::vector<Real> point_marker_color_rgba_;
    std::vector<Real> static_point_marker_color_rgba_;

    Real line_marker_scale_multiplier_;

    std::vector<Real> line_marker_color_rgba_;

    std::vector<Real> mesh_marker_color_rgba_;

    // Rigid Body visualization parameters
    std::string rb_markers_topic_name_;

    Real rb_line_marker_scale_multiplier_;

    std::vector<Real> rb_line_marker_color_rgba_;

    std::vector<Real> rb_mesh_marker_color_rgba_;

    Real min_dist_line_marker_scale_multiplier_;

    std::vector<Real> min_dist_line_marker_color_rgba_;
    
    // Other variables
    Eigen::Matrix<Real,1,3> gravity_;

    bool is_auto_sim_rate_set_;

    bool is_rob_01_attached_;
    bool is_rob_02_attached_;
    bool is_rob_03_attached_;
    bool is_rob_04_attached_;

    int rob_01_attached_id_;
    int rob_02_attached_id_;
    int rob_03_attached_id_;
    int rob_04_attached_id_;

    std::vector<int> rob_01_attached_ids_;
    std::vector<int> rob_02_attached_ids_;
    std::vector<int> rob_03_attached_ids_;
    std::vector<int> rob_04_attached_ids_;

    std::vector<Eigen::Matrix<Real,1,3>> rob_01_attached_rel_poses_;
    std::vector<Eigen::Matrix<Real,1,3>> rob_02_attached_rel_poses_;
    std::vector<Eigen::Matrix<Real,1,3>> rob_03_attached_rel_poses_;
    std::vector<Eigen::Matrix<Real,1,3>> rob_04_attached_rel_poses_;

    Eigen::Quaternion<Real> rob_01_attached_orient_;
    Eigen::Quaternion<Real> rob_02_attached_orient_;
    Eigen::Quaternion<Real> rob_03_attached_orient_;
    Eigen::Quaternion<Real> rob_04_attached_orient_;

    Eigen::Matrix<Real,1,3> rob_01_attached_force_;
    Eigen::Matrix<Real,1,3> rob_02_attached_force_;
    Eigen::Matrix<Real,1,3> rob_03_attached_force_;
    Eigen::Matrix<Real,1,3> rob_04_attached_force_;

    int time_frames_; //to report the performance per frame
    Real time_sum_; //to report the performance per frame

    int marker_id_; // for bookkeeping of the visualization markers of rigid bodies used in the code.

    // TODO: In the future, implement cloth and other deformable/rigid objects as PBDObjects
    // std::vector<PBDObject> sim_objects_;
    pbd_object::Cloth fabric_;
    std::vector<utilities::RigidBodySceneLoader::RigidBodyData> rigid_bodies_;

    // utilities::CollisionHandler collision_handler_;
    // std::unique_ptr<utilities::CollisionHandler> collision_handler_;
    utilities::CollisionHandler* collision_handler_;

    static void contactCallbackFunction(const unsigned int contactType,
                                        const unsigned int bodyIndex1, 
                                        const unsigned int bodyIndex2,
                                        const Eigen::Matrix<Real, 3, 1> &cp1, 
                                        const Eigen::Matrix<Real, 3, 1> &cp2,
                                        const Eigen::Matrix<Real, 3, 1> &normal, 
                                        const Real dist,
                                        const Real restitutionCoeff, 
                                        const Real frictionCoeffStatic,
                                        const Real frictionCoeffDynamic,
                                        void *userData);

};

} // namespace fabric_simulator

#endif /* !FABRIC_SIMULATOR_H */