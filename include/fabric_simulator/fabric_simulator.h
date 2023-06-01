/*
 * Author: Burak Aksoy
 */

#ifndef FABRIC_SIMULATOR_H
#define FABRIC_SIMULATOR_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>

#include <std_srvs/Empty.h>

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

#include "fabric_simulator/utilities/cloth.h"

namespace fabric_simulator
{

class FabricSimulator
{
public:
    FabricSimulator(ros::NodeHandle &nh, ros::NodeHandle &nh_local, boost::recursive_mutex &mtx);
    ~FabricSimulator();

private:
    // Functions ---------------------------------
    
    // Create a mesh structure with face triangle ids and vertice for particle initial poses
    // face triangle ids; vector with n elements with 3d integer rowvectors. each row is 3 particle id of a triangle
    // vertice: vector with m elements with 3d row vectors. each row is 3D xyz position of a particle.
    pbd_object::Mesh createMeshRectangular(const std::string &name, const Real &fabric_x, const Real &fabric_y, const Real &fabric_z, const Real &fabric_res);

    void readAttachedRobotForces();

    // Helper functions to publish created markers
    void publishRvizPoints(const std::vector<geometry_msgs::Point> &points);
    void publishRvizLines(const std::vector<geometry_msgs::Point> &points);

    // Creates the markers to publish
    // void drawRviz(const Eigen::Matrix<Real,Eigen::Dynamic,3> &poses);
    // void drawRvizWireframe(const Eigen::Matrix<Real,Eigen::Dynamic,3> &poses, const Eigen::MatrixX2i &ids);
    void drawRviz(const Eigen::Matrix<Real,Eigen::Dynamic,3> *poses);
    void drawRvizWireframe(const Eigen::Matrix<Real,Eigen::Dynamic,3> *poses, const Eigen::MatrixX2i *ids);

    // Timer callback functions
    void simulate(const ros::TimerEvent& e);
    void render(const ros::TimerEvent& e);
    void publishWrenches(const ros::TimerEvent& e);
    void publishZeroWrenches();

    // Odometry callback functions
    void odometryCb_01(const nav_msgs::Odometry::ConstPtr odom_msg);
    void odometryCb_02(const nav_msgs::Odometry::ConstPtr odom_msg);
    void odometryCb_03(const nav_msgs::Odometry::ConstPtr odom_msg);
    void odometryCb_04(const nav_msgs::Odometry::ConstPtr odom_msg);

    // Service functions
    //   void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }
    bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void reset();
    void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

    // ROS variables---------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    boost::recursive_mutex &mtx_;

    ros::Publisher pub_fabric_points_;

    ros::Publisher pub_wrench_stamped_01_;
    ros::Publisher pub_wrench_stamped_02_;
    ros::Publisher pub_wrench_stamped_03_;
    ros::Publisher pub_wrench_stamped_04_;

    ros::ServiceServer params_srv_;
    
    ros::Subscriber sub_odom_01_;
    ros::Subscriber sub_odom_02_;
    ros::Subscriber sub_odom_03_;
    ros::Subscriber sub_odom_04_;

    ros::Timer timer_render_;
    ros::Timer timer_simulate_;
    ros::Timer timer_wrench_pub_;

    // ROS Parameters
    bool p_active_;
    bool p_reset_;

    Real gravity_x_;
    Real gravity_y_;
    Real gravity_z_;
    
    Real dt_;
    bool set_sim_rate_auto_; //param to set the simulation rate and dt automatically

    int num_substeps_;
    int num_steps_;
    
    // Fabric mesh properties: (Assuming fabric is rectangular)
    Real fabric_x_; // expansion in x dimension (m)
    Real fabric_y_; // expansion in y dimension (m)
    Real fabric_density_; // fabric mass per meter square (kg/m^2)
    Real fabric_resolution_; // particle resolution per meter

    Real fabric_stretching_compliance_;
    Real fabric_bending_compliance_;
    
    Real initial_height_; // initial fabric height from ground (m)

    int num_hang_corners_; // num of corners to hang dlo from (options: 0,1,2,3,4)

    Real global_damp_coeff_v_; 
    
    Real simulation_rate_;
    Real rendering_rate_;

    Real wrench_pub_rate_;

    std::string fabric_points_topic_name_;
    std::string fabric_points_frame_id_;

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
    
    Real fabric_rob_z_offset_; // Additional  attachment height to robots

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

    Eigen::Matrix<Real,1,3> rob_01_attached_force_;
    Eigen::Matrix<Real,1,3> rob_02_attached_force_;
    Eigen::Matrix<Real,1,3> rob_03_attached_force_;
    Eigen::Matrix<Real,1,3> rob_04_attached_force_;

    int time_frames_; //to report the performance per frame
    Real time_sum_; //to report the performance per frame

    // TODO: In the future, implement cloth and other deformable/rigid objects as PBDObjects
    // std::vector<PBDObject> sim_objects_;
    pbd_object::Cloth fabric_;
};

} // namespace fabric_simulator

#endif /* !FABRIC_SIMULATOR_H */