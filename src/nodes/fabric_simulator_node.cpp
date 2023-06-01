/*
 * Author: Burak Aksoy
 */

#include "fabric_simulator/fabric_simulator.h"

using namespace fabric_simulator;

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "fabric_simulator_node", ros::init_options::AnonymousName);
    ros::init(argc, argv, "fabric_simulator_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");
    boost::recursive_mutex mtx;

    try {
        ros::AsyncSpinner spinner(0); 
        // See: https://roboticsbackend.com/ros-asyncspinner-example/
        spinner.start();

        // Inform user with the node name
        std::string node_name = ros::this_node::getName();
        ROS_INFO("[%s]: Initializing",node_name.c_str());
        
        FabricSimulator fabricSimulator(nh, nh_local, mtx);

        ROS_INFO("[%s]: Initialized.",node_name.c_str());
        
        ros::waitForShutdown();
        // ros::spin();
    }
    catch (const char* s) {
        ROS_FATAL_STREAM("[Fabric Simulator]: " << s);
    }
    catch (...) {
        ROS_FATAL_STREAM("[Fabric Simulator]: Unexpected error");
    }

    return 0;
}




