/*
 * Author: Burak Aksoy
 */

#include "fabric_simulator/fabric_simulator.h"
// #include "fabric_simulator/utilities/collision_handler.h"

// using namespace std;
using namespace fabric_simulator;

FabricSimulator::FabricSimulator(ros::NodeHandle &nh, ros::NodeHandle &nh_local, boost::recursive_mutex &mtx): 
    nh_(nh), 
    nh_local_(nh_local),
    mtx_(mtx)
{
    p_active_ = false;

    time_frames_ = 0;
    time_sum_ = 0.0;

    marker_id_ = 3; // for bookkeeping of the visualization markers used in the code.

    is_auto_sim_rate_set_ = false; 

    is_rob_01_attached_ = false;
    is_rob_02_attached_ = false;
    is_rob_03_attached_ = false;
    is_rob_04_attached_ = false;

    rob_01_attached_id_ = -1; //note: ids start from 0. -1 would be a null id.
    rob_02_attached_id_ = -1;
    rob_03_attached_id_ = -1;
    rob_04_attached_id_ = -1;

    rob_01_attached_ids_.clear();
    rob_02_attached_ids_.clear();
    rob_03_attached_ids_.clear();
    rob_04_attached_ids_.clear();

    rob_01_attached_rel_poses_.clear();
    rob_02_attached_rel_poses_.clear();
    rob_03_attached_rel_poses_.clear();
    rob_04_attached_rel_poses_.clear();

    rob_01_attached_force_.setZero();
    rob_02_attached_force_.setZero();
    rob_03_attached_force_.setZero();
    rob_04_attached_force_.setZero();

    // Initialize Timers with deafault period (note: last 2 false mean: oneshot=false, autostart=false)
    timer_render_ = nh_.createTimer(ros::Duration(1.0), &FabricSimulator::render, this,false, false); 
    timer_simulate_ = nh_.createTimer(ros::Duration(1.0), &FabricSimulator::simulate, this,false, false); 

    timer_wrench_pub_ = nh_.createTimer(ros::Duration(1.0), &FabricSimulator::publishWrenchesOnExternalOdoms, this,false, false); 
    timer_render_rb_ = nh_.createTimer(ros::Duration(1.0), &FabricSimulator::renderRigidBodies, this,false, false); 
    timer_min_dist_to_rb_pub_ = nh_.createTimer(ros::Duration(1.0), &FabricSimulator::publishMinDistancesToRigidBodies, this,false, false); 
    timer_fabric_state_pub_ = nh_.createTimer(ros::Duration(1.0), &FabricSimulator::publishFabricState, this,false, false); 

    // Initilize parameters
    params_srv_ = nh_local_.advertiseService("params", &FabricSimulator::updateParams, this);
    initialize(); // calls updateParams, implemented in the header file (fabric_simulator.h) as a ros service call
}

FabricSimulator::~FabricSimulator() {
    publishZeroWrenches();

    nh_local_.deleteParam("gravity_x");
    nh_local_.deleteParam("gravity_y");
    nh_local_.deleteParam("gravity_z");

    nh_local_.deleteParam("dt");
    nh_local_.deleteParam("set_sim_rate_auto");
    nh_local_.deleteParam("num_substeps");
    nh_local_.deleteParam("num_steps");

    nh_local_.deleteParam("is_collision_handling_enabled");
    nh_local_.deleteParam("visualize_min_distances");

    nh_local_.deleteParam("fabric_visualization_mode");
    nh_local_.deleteParam("rb_visualization_mode");

    nh_local_.deleteParam("fabric_mesh_path");

    nh_local_.deleteParam("fabric_x");
    nh_local_.deleteParam("fabric_y");
    nh_local_.deleteParam("fabric_resolution");

    nh_local_.deleteParam("fabric_density");

    nh_local_.deleteParam("fabric_stretching_compliance");
    nh_local_.deleteParam("fabric_bending_compliance");

    nh_local_.deleteParam("contact_tolerance");
    nh_local_.deleteParam("contact_sdf_domain_offset");
    
    nh_local_.deleteParam("initial_height");
    nh_local_.deleteParam("fabric_translation");
    nh_local_.deleteParam("fabric_rotationAxis");
    nh_local_.deleteParam("fabric_rotationAngle");
    nh_local_.deleteParam("fabric_scale");

    nh_local_.deleteParam("num_hang_corners");
    nh_local_.deleteParam("custom_static_particles");

    nh_local_.deleteParam("custom_static_particles_odom_topic_prefix");
    nh_local_.deleteParam("custom_static_particles_cmd_vel_topic_prefix");

    nh_local_.deleteParam("global_damp_coeff_v");

    nh_local_.deleteParam("simulation_rate");
    nh_local_.deleteParam("rendering_rate");
    nh_local_.deleteParam("wrench_pub_rate");
    nh_local_.deleteParam("rendering_rb_rate");
    nh_local_.deleteParam("min_dist_to_rb_pub_rate");
    nh_local_.deleteParam("fabric_state_pub_rate");

    nh_local_.deleteParam("rb_scene_config_path");

    nh_local_.deleteParam("fabric_state_topic_name");
    nh_local_.deleteParam("fabric_markers_topic_name");
    nh_local_.deleteParam("fabric_frame_id");

    nh_local_.deleteParam("face_tri_ids_topic_name");
    
    nh_local_.deleteParam("odom_01_topic_name");
    nh_local_.deleteParam("odom_02_topic_name");
    nh_local_.deleteParam("odom_03_topic_name");
    nh_local_.deleteParam("odom_04_topic_name");

    nh_local_.deleteParam("wrench_01_topic_name");
    nh_local_.deleteParam("wrench_02_topic_name");
    nh_local_.deleteParam("wrench_03_topic_name");
    nh_local_.deleteParam("wrench_04_topic_name");

    nh_local_.deleteParam("wrench_01_frame_id");
    nh_local_.deleteParam("wrench_02_frame_id");
    nh_local_.deleteParam("wrench_03_frame_id");
    nh_local_.deleteParam("wrench_04_frame_id");
    
    nh_local_.deleteParam("fabric_rob_z_offset");

    nh_local_.deleteParam("robot_attach_radius");

    nh_local_.deleteParam("point_marker_scale");

    nh_local_.deleteParam("point_marker_color_rgba");
    nh_local_.deleteParam("static_point_marker_color_rgba");

    nh_local_.deleteParam("line_marker_scale_multiplier");
    
    nh_local_.deleteParam("line_marker_color_rgba");

    nh_local_.deleteParam("mesh_marker_color_rgba");


    nh_local_.deleteParam("rb_markers_topic_name");

    nh_local_.deleteParam("min_dist_to_rb_topic_name");
    nh_local_.deleteParam("min_dist_markers_topic_name");

    nh_local_.deleteParam("change_particle_dynamicity_topic_name");
    nh_local_.deleteParam("set_particle_dynamicity_service_name");

    nh_local_.deleteParam("fix_nearest_fabric_particle_topic_name");
    nh_local_.deleteParam("fix_nearest_fabric_particle_service_name");
    
    nh_local_.deleteParam("attach_external_odom_frame_topic_name");
    nh_local_.deleteParam("attach_external_odom_frame_service_name");

    nh_local_.deleteParam("rb_line_marker_scale_multiplier");

    nh_local_.deleteParam("rb_line_marker_color_rgba");

    nh_local_.deleteParam("rb_mesh_marker_color_rgba");

    nh_local_.deleteParam("min_dist_line_marker_scale_multiplier");

    nh_local_.deleteParam("min_dist_line_marker_color_rgba");

    delete collision_handler_;
}

bool FabricSimulator::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    bool prev_active = p_active_;

    // Get parameters from the parameter server
    nh_local_.param<bool>("active", p_active_, true);
    nh_local_.param<bool>("reset", p_reset_, false);

    nh_local_.param<Real>("gravity_x", gravity_x_, 0.0);
    nh_local_.param<Real>("gravity_y", gravity_y_, 0.0);
    nh_local_.param<Real>("gravity_z", gravity_z_, -9.81);
    
    nh_local_.param<Real>("dt", dt_, 1.0 / 100.0); //200
    nh_local_.param<bool>("set_sim_rate_auto", set_sim_rate_auto_, false); // to set the simulation rate and dt automatically

    nh_local_.param<int>("num_substeps", num_substeps_, 3); //3
    nh_local_.param<int>("num_steps", num_steps_, 1);

    nh_local_.param<bool>("is_collision_handling_enabled", is_collision_handling_enabled_, true);
    nh_local_.param<bool>("visualize_min_distances", visualize_min_distances_, true);

    nh_local_.param<int>("fabric_visualization_mode", fabric_visualization_mode_, 4); // 0: Points Only, 1: Wireframe Only, 2: Mesh Only, 3: Points and Wireframe, 4: All
    nh_local_.param<int>("rb_visualization_mode", rb_visualization_mode_, 2); // 0: Mesh Only, 1: Wireframe Only, 2: Both

    nh_local_.param<std::string>("fabric_mesh_path", fabric_mesh_path_, std::string(""));
    
    nh_local_.param<Real>("fabric_x", fabric_x_, 2.); //2
    nh_local_.param<Real>("fabric_y", fabric_y_, 2.); //2
    nh_local_.param<Real>("fabric_resolution", fabric_resolution_, 10); //10

    nh_local_.param<Real>("fabric_density", fabric_density_, 1.0);

    nh_local_.param<Real>("fabric_stretching_compliance", fabric_stretching_compliance_, 0.01);
    nh_local_.param<Real>("fabric_bending_compliance", fabric_bending_compliance_, 0.01);

    nh_local_.param<Real>("contact_tolerance", contact_tolerance_, static_cast<Real>(0.1));
    nh_local_.param<Real>("contact_sdf_domain_offset", contact_sdf_domain_offset_, static_cast<Real>(2.0));
    
    nh_local_.param<Real>("initial_height", initial_height_, 0);
    nh_local_.param("fabric_translation", fabric_translation_, std::vector<Real>({0.0, 0.0, 0.0}));
    nh_local_.param("fabric_rotationAxis", fabric_rotationAxis_, std::vector<Real>({0.0, 0.0, 1.0}));
    nh_local_.param<Real>("fabric_rotationAngle", fabric_rotationAngle_, 0.0);
    nh_local_.param("fabric_scale", fabric_scale_, std::vector<Real>({1.0, 1.0, 1.0}));

    nh_local_.param<int>("num_hang_corners", num_hang_corners_, 0);
    nh_local_.param("custom_static_particles", custom_static_particles_, std::vector<int>());

    nh_local_.param<std::string>("custom_static_particles_odom_topic_prefix", custom_static_particles_odom_topic_prefix_, std::string("custom_static_particles_odom_"));
    nh_local_.param<std::string>("custom_static_particles_cmd_vel_topic_prefix", custom_static_particles_cmd_vel_topic_prefix_, std::string("custom_static_particles_cmd_vel_"));

    nh_local_.param<Real>("global_damp_coeff_v", global_damp_coeff_v_, 0.0);
    
    nh_local_.param<Real>("simulation_rate", simulation_rate_, 90.0); //90
    nh_local_.param<Real>("rendering_rate", rendering_rate_, 30.0); //30

    nh_local_.param<Real>("wrench_pub_rate", wrench_pub_rate_, 60.0); //60
    nh_local_.param<Real>("rendering_rb_rate", rendering_rb_rate_, 1.0); //1
    nh_local_.param<Real>("min_dist_to_rb_pub_rate", min_dist_to_rb_pub_rate_, 60.0); //60.0
    nh_local_.param<Real>("fabric_state_pub_rate", fabric_state_pub_rate_, 60.0); //60.0

    nh_local_.param<std::string>("rb_scene_config_path", rb_scene_config_path_, std::string(""));

    nh_local_.param<std::string>("fabric_state_topic_name", fabric_state_topic_name_, std::string("fabric_state"));
    nh_local_.param<std::string>("fabric_markers_topic_name", fabric_markers_topic_name_, std::string("fabric_markers"));
    nh_local_.param<std::string>("fabric_frame_id", fabric_frame_id_, std::string("map"));

    nh_local_.param<std::string>("face_tri_ids_topic_name", face_tri_ids_topic_name_, std::string("face_tri_ids"));

    nh_local_.param<std::string>("odom_01_topic_name", odom_01_topic_name_, std::string("d1/ground_truth/fabric_mount/odom"));
    nh_local_.param<std::string>("odom_02_topic_name", odom_02_topic_name_, std::string("d2/ground_truth/fabric_mount/odom"));
    nh_local_.param<std::string>("odom_03_topic_name", odom_03_topic_name_, std::string("d3/ground_truth/fabric_mount/odom"));
    nh_local_.param<std::string>("odom_04_topic_name", odom_04_topic_name_, std::string("d4/ground_truth/fabric_mount/odom"));

    nh_local_.param<std::string>("wrench_01_topic_name", wrench_01_topic_name_, std::string("d1/fabric_wrench_stamped"));
    nh_local_.param<std::string>("wrench_02_topic_name", wrench_02_topic_name_, std::string("d2/fabric_wrench_stamped"));
    nh_local_.param<std::string>("wrench_03_topic_name", wrench_03_topic_name_, std::string("d3/fabric_wrench_stamped"));
    nh_local_.param<std::string>("wrench_04_topic_name", wrench_04_topic_name_, std::string("d4/fabric_wrench_stamped"));

    nh_local_.param<std::string>("wrench_01_frame_id", wrench_01_frame_id_, std::string("d1_tf_fabric_mount_link"));
    nh_local_.param<std::string>("wrench_02_frame_id", wrench_02_frame_id_, std::string("d2_tf_fabric_mount_link"));
    nh_local_.param<std::string>("wrench_03_frame_id", wrench_03_frame_id_, std::string("d3_tf_fabric_mount_link"));
    nh_local_.param<std::string>("wrench_04_frame_id", wrench_04_frame_id_, std::string("d4_tf_fabric_mount_link"));

    nh_local_.param<Real>("fabric_rob_z_offset", fabric_rob_z_offset_, 0.0); // 0.785145); // makes it 80cm above ground

    nh_local_.param<Real>("robot_attach_radius", robot_attach_radius_, 0.0);


    nh_local_.param<Real>("point_marker_scale",   point_marker_scale_,   0.015);

    nh_local_.param("point_marker_color_rgba", point_marker_color_rgba_, std::vector<Real>({1.0, 0.5, 0.0, 1.0}));
    nh_local_.param("static_point_marker_color_rgba", static_point_marker_color_rgba_, std::vector<Real>({1.0, 0.5, 1.0, 1.0}));

    nh_local_.param<Real>("line_marker_scale_multiplier", line_marker_scale_multiplier_, 1.0);
    
    nh_local_.param("line_marker_color_rgba", line_marker_color_rgba_, std::vector<Real>({0.0, 1.0, 0.0, 1.0}));

    nh_local_.param("mesh_marker_color_rgba", mesh_marker_color_rgba_, std::vector<Real>({0.2, 0.5, 0.5, 0.7}));

    nh_local_.param<std::string>("rb_markers_topic_name", rb_markers_topic_name_, std::string("rigid_body_markers"));
    nh_local_.param<std::string>("min_dist_to_rb_topic_name", min_dist_to_rb_topic_name_, std::string("min_dist_to_rigid_bodies"));
    nh_local_.param<std::string>("min_dist_markers_topic_name", min_dist_markers_topic_name_, std::string("min_dist_markers"));

    nh_local_.param<std::string>("change_particle_dynamicity_topic_name", change_particle_dynamicity_topic_name_, std::string("change_particle_dynamicity"));
    nh_local_.param<std::string>("set_particle_dynamicity_service_name", set_particle_dynamicity_service_name_, std::string("set_particle_dynamicity"));
    
    nh_local_.param<std::string>("fix_nearest_fabric_particle_topic_name", fix_nearest_fabric_particle_topic_name_, std::string("fix_nearest_fabric_particle_request"));
    nh_local_.param<std::string>("fix_nearest_fabric_particle_service_name", fix_nearest_fabric_particle_service_name_, std::string("fix_nearest_fabric_particle"));
    
    nh_local_.param<std::string>("attach_external_odom_frame_topic_name", attach_external_odom_frame_topic_name_, std::string("attach_external_odom_frame_request"));
    nh_local_.param<std::string>("attach_external_odom_frame_service_name", attach_external_odom_frame_service_name_, std::string("attach_external_odom_frame"));

    nh_local_.param<std::string>("set_fabric_stretching_compliance_service_name", set_fabric_stretching_compliance_service_name_, std::string("set_fabric_stretching_compliance"));
    nh_local_.param<std::string>("set_fabric_bending_compliance_service_name", set_fabric_bending_compliance_service_name_, std::string("set_fabric_bending_compliance"));
    nh_local_.param<std::string>("get_fabric_stretching_compliance_service_name", get_fabric_stretching_compliance_service_name_, std::string("get_fabric_stretching_compliance"));
    nh_local_.param<std::string>("get_fabric_bending_compliance_service_name", get_fabric_bending_compliance_service_name_, std::string("get_fabric_bending_compliance"));
    nh_local_.param<std::string>("change_fabric_stretching_compliance_topic_name", change_fabric_stretching_compliance_topic_name_, std::string("change_fabric_stretching_compliance"));
    nh_local_.param<std::string>("change_fabric_bending_compliance_topic_name", change_fabric_bending_compliance_topic_name_, std::string("change_fabric_bending_compliance"));
    
    nh_local_.param<std::string>("enable_collision_handling_service_name", enable_collision_handling_service_name_, std::string("enable_collision_handling"));

    nh_local_.param<Real>("rb_line_marker_scale_multiplier", rb_line_marker_scale_multiplier_, 1.0);
    
    nh_local_.param("rb_line_marker_color_rgba", rb_line_marker_color_rgba_, std::vector<Real>({0.0, 0.0, 1.0, 1.0}));
    
    nh_local_.param("rb_mesh_marker_color_rgba", rb_mesh_marker_color_rgba_, std::vector<Real>({0.5, 0.5, 0.5, 0.7}));

    nh_local_.param<Real>("min_dist_line_marker_scale_multiplier", min_dist_line_marker_scale_multiplier_, 1.0);

    nh_local_.param("min_dist_line_marker_color_rgba", min_dist_line_marker_color_rgba_, std::vector<Real>({0.0, 0.0, 0.0, 0.5}));

    // Set timer periods based on the parameters
    timer_render_.setPeriod(ros::Duration(1.0/rendering_rate_));
    timer_simulate_.setPeriod(ros::Duration(1.0/simulation_rate_));

    timer_wrench_pub_.setPeriod(ros::Duration(1.0/wrench_pub_rate_));
    timer_render_rb_.setPeriod(ros::Duration(1.0/rendering_rb_rate_));
    timer_min_dist_to_rb_pub_.setPeriod(ros::Duration(1.0/min_dist_to_rb_pub_rate_));
    timer_fabric_state_pub_.setPeriod(ros::Duration(1.0/fabric_state_pub_rate_));

    // Initilize gravity vector
    gravity_ << gravity_x_, gravity_y_, gravity_z_;    

    std::string fabric_name = "cloth";

    pbd_object::Mesh fabric_mesh;

    if(fabric_mesh_path_.empty()) {
        //Create mesh
        fabric_mesh = FabricSimulator::createMeshRectangular(fabric_name, fabric_x_, fabric_y_, initial_height_, fabric_resolution_);
    } else {
        //Load mesh
        fabric_mesh = FabricSimulator::loadMesh(fabric_name, fabric_mesh_path_, initial_height_);
    }

    // std::cout << "fabric_mesh.name: " << fabric_mesh.name << std::endl;
    // std::cout << "fabric_mesh.vertices:\n" << fabric_mesh.vertices << std::endl;
    // std::cout << "fabric_mesh.face_tri_ids:\n" << fabric_mesh.face_tri_ids << std::endl;

    // Transform the created/loaded mesh
    fabric_mesh = FabricSimulator::transformMesh(fabric_mesh,
                                                fabric_translation_,
                                                fabric_rotationAxis_,
                                                fabric_rotationAngle_,
                                                fabric_scale_);

    // Create cloth
    fabric_ = pbd_object::Cloth(fabric_mesh, 
                                fabric_stretching_compliance_,
                                fabric_bending_compliance_,
                                fabric_density_,
                                global_damp_coeff_v_,
                                gravity_);

    if(fabric_mesh_path_.empty()) {
        // Hang fabric from corners, add the ids of the hang corners to the custom_static_particles_ vector
        fabric_.hangFromCorners(num_hang_corners_, custom_static_particles_);

        // Update the ROS parameter server with the new vector
        nh_local_.setParam("custom_static_particles", custom_static_particles_);
    }
    
    // Set static particles
    fabric_.setStaticParticles(custom_static_particles_);

    // Rigid body scene setting 
    // (sets rigid_bodies_ vector which holds the data related to all the rigid bodies in the scene)
    if(!rb_scene_config_path_.empty()){
        // read the rigid body scene json file with rigid_body_scene_loader utility class
        utilities::RigidBodySceneLoader *rb_scene_loader = new utilities::RigidBodySceneLoader();
        rb_scene_loader->readScene(rb_scene_config_path_, rigid_bodies_);

        // Print all the read data from rigid_bodies_ vector for debugging
        /*
        std::cout << "---------------------------------------" << std::endl;
        for (const utilities::RigidBodySceneLoader::RigidBodyData& rbd : rigid_bodies_) {
            std::cout << "ID: " << rbd.m_id << std::endl;
            std::cout << "Model File: " << rbd.m_modelFile << std::endl;
            std::cout << "Is Dynamic: " << std::boolalpha << rbd.m_isDynamic << std::endl;
            std::cout << "Density: " << rbd.m_density << std::endl;
            std::cout << "Translation: " << rbd.m_x << std::endl;
            std::cout << "Rotation Quaternion: " << rbd.m_q.coeffs() << std::endl; 
            // Note: Eigen quaternions store coefficients as (x, y, z, w)
            std::cout << "Scale: " << rbd.m_scale << std::endl;
            std::cout << "Linear Velocity: " << rbd.m_v << std::endl;
            std::cout << "Angular Velocity: " << rbd.m_omega << std::endl;
            std::cout << "Restitution Coefficient: " << rbd.m_restitutionCoeff << std::endl;
            std::cout << "friction Static Coefficient: " << rbd.m_frictionCoeffStatic << std::endl;
            std::cout << "friction Dynamic Coefficient: " << rbd.m_frictionCoeffDynamic << std::endl;
            std::cout << "Collision Object File Name: " << rbd.m_collisionObjectFileName << std::endl;
            std::cout << "Collision Object Scale: " << rbd.m_collisionObjectScale << std::endl;
            std::cout << "Resolution SDF: " << rbd.m_resolutionSDF << std::endl;
            std::cout << "Invert SDF: " << std::boolalpha << rbd.m_invertSDF << std::endl;
            std::cout << "Is Visible: " << std::boolalpha << rbd.m_isVisible << std::endl;
            std::cout << "---------------------------------------" << std::endl;
        }
        */

        // For each rigid body entry in the json file
        // load its mesh (read from .obj file using initial height = 0
        // transform its mesh (translate, rotate, scale)  (overload the transform Mesh function with accepting a quaternion)
        // update the the created rigid body data's pbd_object::Mesh object with the loaded mesh
        for (utilities::RigidBodySceneLoader::RigidBodyData& rbd : rigid_bodies_) {
            // Load the mesh
            pbd_object::Mesh mesh = loadMesh("RigidBodyMesh_" + std::to_string(rbd.m_id), rbd.m_modelFile, 0);

            // Transform the mesh
            mesh = transformMesh(mesh, rbd.m_x, rbd.m_q.normalized(), rbd.m_scale);

            // Update the RigidBodyData's mesh
            rbd.m_mesh = mesh;
        }

        delete rb_scene_loader; // Don't forget to delete the loader to free memory
    }

    // For each rigid body entry in the json file
    // Set the Signed Distance Field function for the rigid bodies
    // TODO: Make this for loop a function
    for (utilities::RigidBodySceneLoader::RigidBodyData& rbd : rigid_bodies_) {
        // use Discregrid Library to set discrete signed distance fields of the rigid bodies

        // This is for the parallel discretization of (preferably smooth) functions on regular grids. 
	    // This is especially well-suited for the discretization of signed distance functions. 

        // get the sdf file name entry in the json file
        std::string sdf_file_name = rbd.m_collisionObjectFileName; 

        // check if collision sdf file is speficied in the json file
        std::ifstream sdf_file(sdf_file_name); 

        // if the sdf is specifed and it actually exists
		if ((sdf_file_name != "") && (sdf_file.good()))
		{
            // load the sdf file
            // append the loaded sdf file to the rigidBodyData vector(rigid_bodies_)

            std::cout << "Load SDF file: " << sdf_file_name << std::endl;
            rbd.m_discregrid_ptr = std::make_shared<utilities::CollisionHandler::Grid>(sdf_file_name);

		}
        else // sdf file  is not specified in the json file or not readable
        {
            // Therefore,
            // With the Discregrid library generate a (cubic) polynomial discretization given: 
            // a box-shaped domain (A), 
            // a grid resolution (B), 
            // and a function that maps a 3D position in space to a real scalar value.(C)
            // It can also serialize and save the discretization to a file (D)


            // First we Create Discregrid's TriangleMesh object from 
            // the object's vertexData, mesh faces data, and number of faces.

            // std::vector<double> doubleVec;
            // doubleVec.resize(3 * rbd.m_mesh.vertices.rows());
            // for (unsigned int i = 0; i < rbd.m_mesh.vertices.rows(); i++)
            //     for (unsigned int j = 0; j < 3; j++)
            //         doubleVec[3 * i + j] = rbd.m_mesh.vertices.row(i)[j];

            // std::vector<unsigned int> facesVec;
            // facesVec.resize(3 * rbd.m_mesh.face_tri_ids.rows());
            // for (unsigned int i = 0; i < rbd.m_mesh.face_tri_ids.rows(); i++)
            //     for (unsigned int j = 0; j < 3; j++)
            //         facesVec[3 * i + j] = rbd.m_mesh.face_tri_ids.row(i)[j];
            
            // Discregrid::TriangleMesh sdfMesh(&doubleVec[0], facesVec.data(), rbd.m_mesh.vertices.rows(), rbd.m_mesh.face_tri_ids.rows());
            // Note that if you do this the SDF object pose and scale are defaulted to the transformed object, 
            // This is because the mesh is already transformed. So you may need to update your json file accordingly.

            // OR another option is to Create Discregrid's TriangleMesh object directly 
            // reading the obj file using the discregrid library's file reader function:
            Discregrid::TriangleMesh sdfMesh(rbd.m_modelFile);

            // --------------- (A) -------------------
            // Now create the box shaped domain of the discretization from the size of 
            // the AABB(Axis Aligned Bounding Box) of the Discregrid's TriangleMesh object.
            Eigen::AlignedBox3d domain;
            for (auto const& x : sdfMesh.vertices())
            {
                domain.extend(x);
            }

            // This offset defines the minimum distance to a rigid body that will start reporting minimum distance readings
            // Eigen::Vector3d offsetVector = contact_sdf_domain_offset_ * Eigen::Vector3d::Ones();
            Eigen::Vector3d offsetVector = (contact_sdf_domain_offset_ * Eigen::Vector3d::Ones()).array() / rbd.m_scale.transpose().array();
            std::cout << "SDF domain offset :" << offsetVector << std::endl;
            domain.max() += offsetVector;
            domain.min() -= offsetVector;

            // --------------- (B) -------------------
            // Specify the grid resolution from the scene (.json) file
            std::cout << "Set SDF resolution: " 
                        << rbd.m_resolutionSDF[0] << ", " 
                        << rbd.m_resolutionSDF[1] << ", " 
                        << rbd.m_resolutionSDF[2] << std::endl;
            
            std::array<unsigned int, 3> resolution({rbd.m_resolutionSDF[0], 
                                                    rbd.m_resolutionSDF[1], 
                                                    rbd.m_resolutionSDF[2] });

            // Now generate the SDF Grid object by creating the 
            // Discregrid::CubicLagrangeDiscreteGrid object,
            // with the domain and the specified resolution:
            rbd.m_discregrid_ptr = std::make_shared<utilities::CollisionHandler::Grid>(domain, resolution);

            // --------------- (C) -------------------
            // Now specify the function that maps a 3D position in space to a real scalar value.
            
            // We need the signed distances hence we
            // create object of a TriangleMeshDistance class data structure that directly provides 
            // the capability to compute and discretize signed distance fields to triangle meshes.
            Discregrid::TriangleMeshDistance md(sdfMesh); 

            // create a function object "func" using the default constructor of Discregrid::DiscreteGrid::ContinuousFunction
            auto func = Discregrid::DiscreteGrid::ContinuousFunction{}; 
            // Reassign func to a lambda function that takes an Eigen::Vector3d input and returns the distance from the input point to the mesh.
            func = [&md](Eigen::Vector3d const& xi) {return md.signed_distance(xi).distance; };
            
            // add the function "func" to the distance field grid corresponding to sdfFileName.
            // This generated the discretization on the initiated grid.
            rbd.m_discregrid_ptr->addFunction(func, true); 

            // --------------- (D) -------------------
            // Create the sdf file for later use.

            // We can serialize this discretization to an output file:
            
            // Log the message that an SDF is being generated for the model file.
            std::cout << "Generate SDF for model file: " << rbd.m_modelFile << std::endl;

            // create a file path and name for the collision object sdf file if it is not specified in the json file
            const std::string resStr = std::to_string(rbd.m_resolutionSDF[0]) + "_" + 
                                        std::to_string(rbd.m_resolutionSDF[1]) + "_" + 
                                        std::to_string(rbd.m_resolutionSDF[2]); // sdf resolution string to append to end of obj file name
            sdf_file_name =  rbd.m_modelFile + "_" + resStr + ".csdf"; // sdf file name with full path next to the obj file.

            // Log the message that the SDF is being saved.
            std::cout << "Save SDF: " << sdf_file_name << std::endl;

            // save the sdf file to the specified path (sdf_file_name)
            rbd.m_discregrid_ptr->save(sdf_file_name);
        }
    }


    // Create CollisionHandler  ()'this' pointer to pass the data of this fabric simulator class)
    // collision_handler_ = std::make_unique<utilities::CollisionHandler>(fabric_, rigid_bodies_);
    collision_handler_ = new utilities::CollisionHandler(fabric_, rigid_bodies_);

    collision_handler_->setContactTolerance(contact_tolerance_);

    collision_handler_->setContactCallback(contactCallbackFunction, this);

    // Add cloth object to the collision handler
    collision_handler_->addCollisionObjectWithoutGeometry(0, // unsigned int bodyIndex
                                                        utilities::CollisionHandler::CollisionObject::TriangleModelCollisionObjectType,
                                                        fabric_.getPosPtr(), // Eigen::Matrix<Real,Eigen::Dynamic,3> *,
                                                        fabric_.getPosPtr()->rows(), // unsigned int numVertices
                                                        true);// bool testMesh)


    // Add rigid bodies to the collision handler
    // TODO: Make this for loop a function
    int i = 0;
    for (utilities::RigidBodySceneLoader::RigidBodyData& rbd : rigid_bodies_) {

        collision_handler_->addCubicSDFCollisionObject(i, // unsigned int bodyIndex
                                                    utilities::CollisionHandler::CollisionObject::RigidBodyCollisionObjectType, 
                                                    &rbd.m_mesh.vertices, // Eigen::Matrix<Real,Eigen::Dynamic,3> *
                                                    rbd.m_mesh.vertices.rows(), // int
                                                    rbd.m_discregrid_ptr, //std::shared_ptr<Discregrid::CubicLagrangeDiscreteGrid>
                                                    rbd.m_collisionObjectScale, //const Eigen::Matrix<Real, 3, 1> 
                                                    true, //bool test mesh
                                                    rbd.m_invertSDF); // bool invert sdf

        i++;
    }


    if (p_active_ != prev_active) {
        if (p_active_) {
            // Create visualization marker publisher
            pub_fabric_state_ = nh_.advertise<fabric_simulator::SegmentStateArray>(fabric_state_topic_name_, 1);
            pub_fabric_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>(fabric_markers_topic_name_, 1);

            // Create subscribers
            sub_odom_01_ = nh_.subscribe(odom_01_topic_name_, 1, &FabricSimulator::odometryCb_01, this);
            sub_odom_02_ = nh_.subscribe(odom_02_topic_name_, 1, &FabricSimulator::odometryCb_02, this);
            sub_odom_03_ = nh_.subscribe(odom_03_topic_name_, 1, &FabricSimulator::odometryCb_03, this);
            sub_odom_04_ = nh_.subscribe(odom_04_topic_name_, 1, &FabricSimulator::odometryCb_04, this);

            // Create a subscriber for each custom static particle
            for (const int& particle_id : custom_static_particles_) {
                // Check if an odom subscriber for this particle ID already exists
                if (custom_static_particles_odom_subscribers_.find(particle_id) == custom_static_particles_odom_subscribers_.end()) {
                    std::string topic = custom_static_particles_odom_topic_prefix_ + std::to_string(particle_id);
                    ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(topic, 10,
                                                                            [this, particle_id](const nav_msgs::Odometry::ConstPtr& odom_msg) { 
                                                                                this->odometryCb_custom_static_particles(odom_msg, particle_id); }
                                                                            );
                    // Add the new subscriber to the map
                    custom_static_particles_odom_subscribers_[particle_id] = sub;
                }
                // Else, a subscriber for this particle ID already exists

                // Check if a cmd_vel subscriber for this particle ID already exists
                if (custom_static_particles_cmd_vel_subscribers_.find(particle_id) == custom_static_particles_cmd_vel_subscribers_.end()) {
                    std::string topic = custom_static_particles_cmd_vel_topic_prefix_ + std::to_string(particle_id);
                    ros::Subscriber sub = nh_.subscribe<geometry_msgs::Twist>(topic, 10,
                                                                            [this, particle_id](const geometry_msgs::Twist::ConstPtr& twist_msg) { 
                                                                                this->cmdVelCb_custom_static_particles(twist_msg, particle_id); }
                                                                            );    
                    // Add the new subscriber to the map
                    custom_static_particles_cmd_vel_subscribers_[particle_id] = sub;
                }
                // Else, a subscriber for this particle ID already exists
            }

            // Create a subscriber for the change particle dynamicity topic
            sub_change_particle_dynamicity_ = nh_.subscribe(change_particle_dynamicity_topic_name_, 
                                                            1, 
                                                            &FabricSimulator::changeParticleDynamicityCb, 
                                                            this);

            // Create a subscriber for handling the requests to fix (stick) particles to the environment
            sub_fix_nearest_fabric_particle_request_ = nh_.subscribe(fix_nearest_fabric_particle_topic_name_, 
                                                                1, 
                                                                &FabricSimulator::fixNearestFabricParticleRequestCb, 
                                                                this);

            sub_attach_external_odom_frame_request_ = nh_.subscribe(attach_external_odom_frame_topic_name_, 
                                                                    1, 
                                                                    &FabricSimulator::attachExternalOdomFrameRequestCb, 
                                                                    this);

            // Create a subscriber for the change fabric compliance topics
            sub_change_stretching_compliance_ = nh_.subscribe(change_fabric_stretching_compliance_topic_name_, 
                                                                1, 
                                                                &FabricSimulator::changeStretchingComplianceCb, 
                                                                this);
        
            sub_change_bending_compliance_ = nh_.subscribe(change_fabric_bending_compliance_topic_name_,
                                                            1,
                                                            &FabricSimulator::changeBendingComplianceCb,
                                                            this);  

            // Create publishers
            pub_face_tri_ids_ = nh_.advertise<std_msgs::Int32MultiArray>(face_tri_ids_topic_name_, 1);

            pub_wrench_stamped_01_ = nh_.advertise<geometry_msgs::WrenchStamped>(wrench_01_topic_name_, 1);
            pub_wrench_stamped_02_ = nh_.advertise<geometry_msgs::WrenchStamped>(wrench_02_topic_name_, 1);
            pub_wrench_stamped_03_ = nh_.advertise<geometry_msgs::WrenchStamped>(wrench_03_topic_name_, 1);
            pub_wrench_stamped_04_ = nh_.advertise<geometry_msgs::WrenchStamped>(wrench_04_topic_name_, 1);

            pub_rb_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>(rb_markers_topic_name_, 1);

            pub_min_dist_to_rb_ = nh_.advertise<fabric_simulator::MinDistanceDataArray>(min_dist_to_rb_topic_name_, 1);

            pub_min_dist_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>(min_dist_markers_topic_name_, 1);

            // Create Services
            set_particle_dynamicity_srv_ = nh_local_.advertiseService(set_particle_dynamicity_service_name_, 
                &FabricSimulator::setParticleDynamicityCallback, this);

            fix_nearest_fabric_particle_srv_ = nh_local_.advertiseService(fix_nearest_fabric_particle_service_name_, 
                &FabricSimulator::fixNearestFabricParticleCallback, this);

            attach_external_odom_frame_srv_ = nh_local_.advertiseService(attach_external_odom_frame_service_name_, 
                &FabricSimulator::attachExternalOdomFrameCallback, this);

            set_stretching_compliance_srv_ = nh_local_.advertiseService(set_fabric_stretching_compliance_service_name_,
                            &FabricSimulator::setFabricStretchingComplianceCallback, this);

            set_bending_compliance_srv_ = nh_local_.advertiseService(set_fabric_bending_compliance_service_name_,
                            &FabricSimulator::setFabricBendingComplianceCallback, this);

            get_stretching_compliance_srv_ = nh_local_.advertiseService(get_fabric_stretching_compliance_service_name_,
                            &FabricSimulator::getFabricStretchingComplianceCallback, this);

            get_bending_compliance_srv_ = nh_local_.advertiseService(get_fabric_bending_compliance_service_name_,
                            &FabricSimulator::getFabricBendingComplianceCallback, this);

            enable_collision_handling_srv_ = nh_local_.advertiseService(enable_collision_handling_service_name_,
                            &FabricSimulator::enableCollisionHandlingCallback, this);

            // Start timers
            timer_simulate_.start();
            timer_render_.start();
            timer_wrench_pub_.start();
            timer_render_rb_.start();
            timer_min_dist_to_rb_pub_.start();
            timer_fabric_state_pub_.start();
        }
        else {
            // Send empty message?
            // TODO: Send zero (or null corresponding) message from each publisher to end cleanly.
            publishZeroWrenches();

            // Stop publishers
            pub_fabric_marker_array_.shutdown();
            pub_fabric_state_.shutdown();

            pub_face_tri_ids_.shutdown();

            // Stop subscribers
            sub_odom_01_.shutdown();
            sub_odom_02_.shutdown();
            sub_odom_03_.shutdown();
            sub_odom_04_.shutdown();

            sub_change_particle_dynamicity_.shutdown();

            sub_change_stretching_compliance_.shutdown();
            sub_change_bending_compliance_.shutdown();

            // Iterate through the map and shut down each subscriber
            for (auto& kv : custom_static_particles_odom_subscribers_) {
                ros::Subscriber& subscriber = kv.second; // Get the subscriber (which is the value in the key-value pair)
                subscriber.shutdown();
            }
            
            for (auto& kv : custom_static_particles_cmd_vel_subscribers_) {
                ros::Subscriber& subscriber = kv.second; // Get the subscriber (which is the value in the key-value pair)
                subscriber.shutdown();
            }

            for (auto& kv : external_odom_attachments_) {
                ros::Subscriber& subscriber = kv.second.odom_sub; // Get the subscriber
                subscriber.shutdown();
            }
            
            // Stop publishers
            pub_wrench_stamped_01_.shutdown();
            pub_wrench_stamped_02_.shutdown();
            pub_wrench_stamped_03_.shutdown();
            pub_wrench_stamped_04_.shutdown();

            pub_rb_marker_array_.shutdown();

            pub_min_dist_to_rb_.shutdown();

            // Stop timers
            timer_render_.stop();
            timer_simulate_.stop();
            timer_wrench_pub_.stop();
            timer_render_rb_.stop();
            timer_min_dist_to_rb_pub_.stop();
            timer_fabric_state_pub_.stop();
        }
    }

    if (p_reset_)
        reset();

    return true;
}

void FabricSimulator::reset(){
    time_frames_ = 0;
    time_sum_ = 0.0;

    marker_id_ = 3;

    is_auto_sim_rate_set_ = false; 

    is_rob_01_attached_ = false;
    is_rob_02_attached_ = false;
    is_rob_03_attached_ = false;
    is_rob_04_attached_ = false;

    rob_01_attached_id_ = -1; //note: ids start from 0. -1 would be a null id.
    rob_02_attached_id_ = -1;
    rob_03_attached_id_ = -1;
    rob_04_attached_id_ = -1;

    rob_01_attached_ids_.clear();
    rob_02_attached_ids_.clear();
    rob_03_attached_ids_.clear();
    rob_04_attached_ids_.clear();

    rob_01_attached_rel_poses_.clear();
    rob_02_attached_rel_poses_.clear();
    rob_03_attached_rel_poses_.clear();
    rob_04_attached_rel_poses_.clear();

    rob_01_attached_force_.setZero();
    rob_02_attached_force_.setZero();
    rob_03_attached_force_.setZero();
    rob_04_attached_force_.setZero();

    p_reset_ = false;
    nh_local_.setParam("reset",false);
}


// Service callback calls the helper and sets the response accordingly.
bool FabricSimulator::setParticleDynamicityCallback(
    fabric_simulator::SetParticleDynamicity::Request &req,
    fabric_simulator::SetParticleDynamicity::Response &res)
{
    bool success = updateParticleDynamicityCommon(req.particle_id, req.is_dynamic);
    res.success = success;
    return true;  // Always return true to indicate the service was processed.
}

// Message callback calls the same helper function.
void FabricSimulator::changeParticleDynamicityCb(
    const fabric_simulator::ChangeParticleDynamicity::ConstPtr& msg)
{
    updateParticleDynamicityCommon(msg->particle_id, msg->is_dynamic);
}

// Helper function that contains the common logic.
bool FabricSimulator::updateParticleDynamicityCommon(int particle_id, bool is_dynamic) {
    // Attempt to change dynamicity
    try {
        fabric_.changeParticleDynamicity(particle_id, is_dynamic);
    } catch (const std::out_of_range& e) {
        ROS_ERROR("Error in updateParticleDynamicityCommon: %s", e.what());
        return false;
    }

    // Update custom_static_particles_ accordingly
    if (is_dynamic) {
        auto it = std::find(custom_static_particles_.begin(), custom_static_particles_.end(), particle_id);
        if (it != custom_static_particles_.end()) {
            custom_static_particles_.erase(it);
            ROS_INFO("Particle %d set to dynamic and removed from custom_static_particles_", particle_id);
        } else {
            ROS_INFO("Particle %d not found in custom_static_particles_", particle_id);
        }
    } else {
        if (std::find(custom_static_particles_.begin(), custom_static_particles_.end(), particle_id) == custom_static_particles_.end()) {
            custom_static_particles_.push_back(particle_id);
            ROS_INFO("Particle %d set to static and added to custom_static_particles_", particle_id);
        } else {
            ROS_INFO("Particle %d already in custom_static_particles_", particle_id);
        }
    }

    // Manage odom and cmd_vel subscribers
    auto sub_iter = custom_static_particles_odom_subscribers_.find(particle_id);
    auto sub_iter_cmd_vel = custom_static_particles_cmd_vel_subscribers_.find(particle_id);

    if (is_dynamic) {
        if (sub_iter != custom_static_particles_odom_subscribers_.end()) {
            sub_iter->second.shutdown();
            custom_static_particles_odom_subscribers_.erase(sub_iter);
            ROS_INFO("Odom Subscriber for particle %d shut down", particle_id);
        } else {
            ROS_INFO("No odom subscriber found for particle %d", particle_id);
        }
        if (sub_iter_cmd_vel != custom_static_particles_cmd_vel_subscribers_.end()) {
            sub_iter_cmd_vel->second.shutdown();
            custom_static_particles_cmd_vel_subscribers_.erase(sub_iter_cmd_vel);
            ROS_INFO("Vel Subscriber for particle %d shut down", particle_id);
        } else {
            ROS_INFO("No cmd_vel subscriber found for particle %d", particle_id);
        }
    } else {
        if (sub_iter == custom_static_particles_odom_subscribers_.end()) {
            std::string topic = custom_static_particles_odom_topic_prefix_ + std::to_string(particle_id);
            ros::Subscriber sub = nh_.subscribe<nav_msgs::Odometry>(topic, 1,
                                                                    [this, particle_id](const nav_msgs::Odometry::ConstPtr& odom_msg) {
                                                                        this->odometryCb_custom_static_particles(odom_msg, particle_id);
                                                                    });
            custom_static_particles_odom_subscribers_[particle_id] = sub;
            ROS_INFO("Odom Subscriber for particle %d created", particle_id);
        } else {
            ROS_INFO("Odom Subscriber for particle %d already exists", particle_id);
        }
        if (sub_iter_cmd_vel == custom_static_particles_cmd_vel_subscribers_.end()) {
            std::string topic = custom_static_particles_cmd_vel_topic_prefix_ + std::to_string(particle_id);
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::Twist>(topic, 10,
                [this, particle_id](const geometry_msgs::Twist::ConstPtr& twist_msg) {
                    this->cmdVelCb_custom_static_particles(twist_msg, particle_id);
                });
            custom_static_particles_cmd_vel_subscribers_[particle_id] = sub;
            ROS_INFO("Vel Subscriber for particle %d created", particle_id);
        } else {
            ROS_INFO("Vel Subscriber for particle %d already exists", particle_id);
        }
    }

    // If everything went smoothly, return success
    return true;
}


// Request callback simply forwards the values to the common function.
void FabricSimulator::fixNearestFabricParticleRequestCb(const fabric_simulator::FixNearestFabricParticleRequest::ConstPtr& 
                                                                                fix_nearest_fabric_particle_request_msg)
{
    fixNearestFabricParticleCommon(fix_nearest_fabric_particle_request_msg->is_fix,
                                    fix_nearest_fabric_particle_request_msg->pose);
}

// Service callback uses the common helper and sets the response accordingly.
bool FabricSimulator::fixNearestFabricParticleCallback(fabric_simulator::FixNearestFabricParticle::Request &req, 
                                                        fabric_simulator::FixNearestFabricParticle::Response &res)
{
    bool success = fixNearestFabricParticleCommon(req.is_fix, req.pose);
    res.success = success;
    return true;
}

// Common helper that does the actual work.
bool FabricSimulator::fixNearestFabricParticleCommon(bool is_fix, const geometry_msgs::PoseStamped &pose)
{
    // Lock to prevent collisions with the simulation.
    boost::recursive_mutex::scoped_lock lock(mtx_);
    
    // Convert Pose to Eigen position.
    Eigen::Matrix<Real,1,3> pos(pose.pose.position.x, 
                                pose.pose.position.y, 
                                pose.pose.position.z);

    // Convert orientation (if needed).
    Eigen::Quaternion<Real> ori(pose.pose.orientation.w, 
                                pose.pose.orientation.x, 
                                pose.pose.orientation.y, 
                                pose.pose.orientation.z);
    ori.normalize();

    // Tell simulation (fabric) to fix/unfix the nearest particles within a radius.
    std::vector<int> attached_ids;
    std::vector<Eigen::Matrix<Real,1,3>> attached_rel_poses;
    fabric_.attachWithinRadius(pos, robot_attach_radius_, 
                                    attached_ids, attached_rel_poses,
                                    is_fix);

    // Log the results.
    ROS_INFO("FabricSimulator::fixNearestFabricParticleCommon: radius: %f, "
                "is_fix: %d, "
                "requested point: %f %f %f, "
                "num affected particles: %d", 
                robot_attach_radius_,
                is_fix,
                pos(0), pos(1), pos(2),
                (int)attached_ids.size());
    // for (int i = 0; i < attached_ids.size(); ++i) {
    //     ROS_INFO("FabricSimulator::fixNearestFabricParticleCommon: "
    //                 "Attached particle id %d with rel pose %f %f %f", 
    //                 attached_ids[i],
    //                 attached_rel_poses[i](0),
    //                 attached_rel_poses[i](1),
    //                 attached_rel_poses[i](2));
    // }
                                    
    return true;
}

void FabricSimulator::attachExternalOdomFrameRequestCb(
    const fabric_simulator::AttachExternalOdomFrameRequest::ConstPtr & msg)
{
    attachExternalOdomFrameCommon(msg->odom_topic, msg->is_attach);
}

bool FabricSimulator::attachExternalOdomFrameCallback(fabric_simulator::AttachExternalOdomFrame::Request &req, 
                                                        fabric_simulator::AttachExternalOdomFrame::Response &res)
{
    bool success = attachExternalOdomFrameCommon(req.odom_topic, req.is_attach);
    res.success = success;
    return true;
}

bool FabricSimulator::attachExternalOdomFrameCommon(const std::string & odom_topic, bool is_attach)
{

    if (is_attach) 
    {
        // Check if we already have an entry for this odom_topic
        auto it = external_odom_attachments_.find(odom_topic);
        if (it == external_odom_attachments_.end())
        {
            // If not in the map, create a new ExternalOdomAttachment
            ExternalOdomAttachment external_odom_attachment;
            external_odom_attachment.odom_topic = odom_topic;
            
            // (Optionally) decide or read a param for the wrench frame ID or topic
            // generate a default or parse from the request, if odom msg has a child frame id, this default is updated in the odom callback
            // otherwise (i.e. if the odom msg has no child frame id/empty str), this default is used.
            external_odom_attachment.wrench_frame_id = "default_frame_id";
            std::string wrench_topic_name = odom_topic + "_wrench";
            
            // Create the subscriber. We use a bind or lambda to pass the key
            // back into the callback so that we know which external_odom_attachment is calling back
            external_odom_attachment.odom_sub = nh_.subscribe<nav_msgs::Odometry>(
                odom_topic, 10,
                [this, odom_topic](const nav_msgs::Odometry::ConstPtr& odom_msg) { 
                    this->odometryCb_external(odom_msg, odom_topic); }
                );
            
            // Create a publisher for the wrench
            external_odom_attachment.wrench_pub = nh_.advertise<geometry_msgs::WrenchStamped>(wrench_topic_name, 1);
            
            // Insert into the map
            external_odom_attachments_[odom_topic] = external_odom_attachment;

            ROS_INFO_STREAM("Created new external_odom_attachment attachment for topic: " << odom_topic);
        }
        else
        {
            ROS_INFO_STREAM("External_odom_attachment with topic '" << odom_topic << "' already known.");
        }

        // Assume not attached until we get the first callback
        external_odom_attachments_[odom_topic].is_attached = false; // We'll try to attach on first next odom callback
    } 
    else 
    { // Detach
        auto it = external_odom_attachments_.find(odom_topic);
        if (it != external_odom_attachments_.end())
        {
            // Cleanup if needed
            ROS_INFO_STREAM("Detaching external_odom_attachment with topic: " << odom_topic);
            
            // Make those particles dynamic from the fabric:
            fabric_.setDynamicParticles(it->second.attached_ids);

            // Do the internal cleanup:

            // We can either just mark it not attached:
            it->second.is_attached = false;
            // it->second.attached_ids.clear();
            // or remove from map entirely:
            it->second.odom_sub.shutdown();
            it->second.wrench_pub.shutdown();
            
            // Actually remove from map
            external_odom_attachments_.erase(it);
        }
        else
        {
            ROS_INFO_STREAM("No external_odom_attachment found for topic '" << odom_topic << "' to detach.");
        }
    }
    
    return true;
}


void FabricSimulator::changeStretchingComplianceCb(const std_msgs::Float32::ConstPtr& msg){
    Real stretching_compliance = msg->data;
    fabric_.setStretchingCompliance(stretching_compliance);
}

void FabricSimulator::changeBendingComplianceCb(const std_msgs::Float32::ConstPtr& msg){
    Real bending_compliance = msg->data;
    fabric_.setBendingCompliance(bending_compliance);
}

// Create setStretchingCompliance service callback
bool FabricSimulator::setFabricStretchingComplianceCallback(fabric_simulator::SetFabricStretchingCompliance::Request &req,
                                              fabric_simulator::SetFabricStretchingCompliance::Response &res){
    fabric_.setStretchingCompliance(req.stretching_compliance);
    res.success = true;
    return true;
}

// Create setBendingCompliance service callback
bool FabricSimulator::setFabricBendingComplianceCallback(fabric_simulator::SetFabricBendingCompliance::Request &req,
                                                fabric_simulator::SetFabricBendingCompliance::Response &res){
    fabric_.setBendingCompliance(req.bending_compliance);
    res.success = true;
    return true;
}

// Create getStretchingCompliance service callback
bool FabricSimulator::getFabricStretchingComplianceCallback(fabric_simulator::GetFabricStretchingCompliance::Request &req,
                                              fabric_simulator::GetFabricStretchingCompliance::Response &res){
    res.stretching_compliance = fabric_.getStretchingCompliance();
    return true;
}

// Create getBendingCompliance service callback
bool FabricSimulator::getFabricBendingComplianceCallback(fabric_simulator::GetFabricBendingCompliance::Request &req,
                                                fabric_simulator::GetFabricBendingCompliance::Response &res){
    res.bending_compliance = fabric_.getBendingCompliance();
    return true;
}

// Create enableCollisionHandling service callback
bool FabricSimulator::enableCollisionHandlingCallback(fabric_simulator::EnableCollisionHandling::Request &req,
                                                   fabric_simulator::EnableCollisionHandling::Response &res){
    boost::recursive_mutex::scoped_lock lock(mtx_);

    is_collision_handling_enabled_ = req.is_enable;

    // log the message
    if (is_collision_handling_enabled_){
        ROS_INFO("Collision handling is enabled.");
    }
    else{
        collision_handler_->resetContacts();
        ROS_INFO("Collision handling is disabled.");
    }

    res.success = true;
    return true;
}

pbd_object::Mesh FabricSimulator::createMeshRectangular(const std::string &name, 
                                                        const Real &fabric_x, 
                                                        const Real &fabric_y, 
                                                        const Real &fabric_z, 
                                                        const Real &fabric_res){
    // Create vector of 3D Eigen vectors to hold the "list of vertices" and "list of face triangle ids"
    std::vector<Eigen::Matrix<Real,1,3>> vertices;
    std::vector<Eigen::RowVector3i> face_tri_ids;

    int num_particle_x = fabric_x * fabric_res;
    int num_particle_y = fabric_y * fabric_res;
    // Assuming a fabric centered at the origin, create a linear spaced coordinate vectors for the coordinates
    
    // Generate the x_coords
    Eigen::Matrix<Real,1,Eigen::Dynamic> x_coords(num_particle_x + 1);
    Real start = fabric_x/2.0;
    Real end = -fabric_x/2.0;
    Real step = (end - start) / num_particle_x;

    for (int i = 0; i <= num_particle_x; i++) {
        x_coords(i) = start + i * step;
    }
    
    // Generate the y_coords
    Eigen::Matrix<Real,1,Eigen::Dynamic> y_coords(num_particle_y + 1);
    start = fabric_y/2.0;
    end = -fabric_y/2.0;
    step = (end - start) / num_particle_y;

    for (int i = 0; i <= num_particle_y; i++) {
        y_coords(i) = start + i * step;
    }

    // Create vertices with x,y,z coordinates
    for (int i = 0; i < x_coords.size(); i++) {
        for (int j = 0; j < y_coords.size(); j++) {
            Eigen::Matrix<Real,1,3> v(x_coords(i), y_coords(j), fabric_z);
            vertices.push_back(v);
        }
    }

    // Eigen::Map<Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic>> vertices_mat((Real *)vertices.data(), vertices.size(), 3);
    // (Real *)vertices.data() returns a pointer to the first element of the vertices vector, 
    // which can be cast to a Real pointer. The Eigen::Map object takes the pointer, the 
    // number of rows vertices.size(), and the number of columns 3, as arguments, and maps this 
    // memory block to an Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic> object.
    Eigen::Matrix<Real,Eigen::Dynamic,Eigen::Dynamic> vertices_mat(vertices.size(), 3);
    for (int i = 0; i < vertices.size(); i++) {
        vertices_mat.row(i) = vertices[i];
    }

    // Create face triangle ids
    // int id = 0;
    // for (int i = 0; i < x_coords.size() - 1; i++) {
    //     for (int j = 0; j < y_coords.size() - 1; j++) {

    //         Eigen::RowVector3i ids(id, id + 1, id + y_coords.size());
    //         face_tri_ids.push_back(ids);

    //         if (j > 0) {
    //             Eigen::RowVector3i ids(id, id + y_coords.size(), id + y_coords.size() - 1);
    //             face_tri_ids.push_back(ids);
    //         }

    //         if (j + 1 == y_coords.size() - 1) {
    //             Eigen::RowVector3i ids(id + 1, id + 1 + y_coords.size(), id + 1 + y_coords.size() - 1);
    //             face_tri_ids.push_back(ids);
    //             id++;
    //         }
    //         id++;
    //     }
    // }

    // Create face triangle ids (more regular mesh than above)
    int id = 0;
    for (int i = 0; i < x_coords.size() - 1; i++) {
        for (int j = 0; j < y_coords.size() - 1; j++) {
            if (i % 2 == 0) {
                if (j % 2 == 0){
                    Eigen::RowVector3i ids(id, id+1, id+1+y_coords.size());
                    face_tri_ids.push_back(ids);

                    Eigen::RowVector3i idss(id, id+1+y_coords.size(), id+y_coords.size());
                    face_tri_ids.push_back(idss);
                } else { // ie (j % 2 == 1)
                    Eigen::RowVector3i ids(id, id+1, id+y_coords.size());
                    face_tri_ids.push_back(ids);

                    Eigen::RowVector3i idss(id+1, id+1+y_coords.size(), id+y_coords.size());
                    face_tri_ids.push_back(idss);
                }
            } 
            else { // ie (i % 2 == 1)
                if (j % 2 == 0){
                    Eigen::RowVector3i ids(id, id+1, id+y_coords.size());
                    face_tri_ids.push_back(ids);

                    Eigen::RowVector3i idss(id+1, id+1+y_coords.size(), id+y_coords.size());
                    face_tri_ids.push_back(idss);
                } else { // ie (j % 2 == 1)
                    Eigen::RowVector3i ids(id, id+1, id+1+y_coords.size());
                    face_tri_ids.push_back(ids);

                    Eigen::RowVector3i idss(id, id+1+y_coords.size(), id+y_coords.size());
                    face_tri_ids.push_back(idss);
                }
            }


            if (j + 1 == y_coords.size() - 1) {
                id++;
            }
            id++;
        }
    }

    // Eigen::Map<Eigen::MatrixXi> face_tri_ids_mat((int *)face_tri_ids.data(), face_tri_ids.size(), 3);
    Eigen::MatrixXi face_tri_ids_mat(face_tri_ids.size(), 3);
    for (int i = 0; i < face_tri_ids.size(); i++) {
        face_tri_ids_mat.row(i) = face_tri_ids[i];
    }

    pbd_object::Mesh mesh;
    mesh.name = name;
    mesh.vertices = vertices_mat;
    mesh.face_tri_ids = face_tri_ids_mat;

    return mesh;
}

pbd_object::Mesh FabricSimulator::loadMesh(const std::string &name, 
                                        const std::string &path, 
                                        const Real &fabric_z) {
    // Function to load an .obj file 
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file " + path);
    }

    std::vector<Eigen::Matrix<Real, 1, 3>> vertices;
    std::vector<Eigen::Matrix<int, 1, 3>> face_tri_ids;
    std::vector<Eigen::Matrix<Real, 1, 2>> tex_coords;
    std::vector<Eigen::Matrix<Real, 1, 3>> normals;

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string prefix;
        ss >> prefix;

        if (prefix == "v") {
            Eigen::Matrix<Real, 1, 3> vertex;
            ss >> vertex(0, 0) >> vertex(0, 1) >> vertex(0, 2);
            vertex(0, 2) += fabric_z; // Adjust z-coordinate
            vertices.push_back(vertex);
        } else if (prefix == "vt") {
            Eigen::Matrix<Real, 1, 2> tex_coord;
            ss >> tex_coord(0, 0) >> tex_coord(0, 1);
            tex_coords.push_back(tex_coord);
        } else if (prefix == "vn") {
            Eigen::Matrix<Real, 1, 3> normal;
            ss >> normal(0, 0) >> normal(0, 1) >> normal(0, 2);
            normals.push_back(normal);
        } else if (prefix == "f") {
            Eigen::Matrix<int, 1, 3> face;
            for (int i = 0; i < 3; ++i) {
                std::string vertex_spec;
                ss >> vertex_spec;
                std::stringstream vertex_ss(vertex_spec);
                std::string vertex_index;
                std::getline(vertex_ss, vertex_index, '/');
                face(0, i) = std::stoi(vertex_index) - 1; // Convert to 0-based index
            }
            face_tri_ids.push_back(face);
        }
    }
    file.close();

    // Convert vectors to Eigen matrices
    Eigen::Matrix<Real, Eigen::Dynamic, 3> vertices_mat(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        vertices_mat.row(i) = vertices[i];
    }

    Eigen::Matrix<int, Eigen::Dynamic, 3> face_tri_ids_mat(face_tri_ids.size(), 3);
    for (size_t i = 0; i < face_tri_ids.size(); ++i) {
        face_tri_ids_mat.row(i) = face_tri_ids[i];
    }

    Eigen::Matrix<Real, Eigen::Dynamic, 2> tex_coords_mat(tex_coords.size(), 2);
    for (size_t i = 0; i < tex_coords.size(); ++i) {
        tex_coords_mat.row(i) = tex_coords[i];
    }

    Eigen::Matrix<Real, Eigen::Dynamic, 3> normals_mat(normals.size(), 3);
    for (size_t i = 0; i < normals.size(); ++i) {
        normals_mat.row(i) = normals[i];
    }

    pbd_object::Mesh mesh;
    mesh.name = name;
    mesh.vertices = vertices_mat;
    mesh.face_tri_ids = face_tri_ids_mat;
    mesh.tex_coords = tex_coords_mat;
    mesh.normals = normals_mat;

    return mesh;
}

pbd_object::Mesh FabricSimulator::transformMesh(const pbd_object::Mesh &mesh, 
                                   const std::vector<Real> &translation,
                                   const std::vector<Real> &rotationAxis,
                                   const Real &rotationAngle,
                                   const std::vector<Real> &scale){
    // Translates, rotates, and scales a given mesh with Rotation given with Axis-Angle (overloaded)

    // Check size of vectors
    assert(translation.size() == 3);
    assert(rotationAxis.size() == 3);
    assert(scale.size() == 3);

    // Prepare translation, rotation and scale matrices
    Eigen::Matrix<Real,3,1> translationVector(translation[0], translation[1], translation[2]);
    Eigen::AngleAxis<Real> rotationMatrix(rotationAngle, Eigen::Matrix<Real,3,1>(rotationAxis[0], rotationAxis[1], rotationAxis[2]));
    Eigen::Matrix<Real,3,1> scaleVector(scale[0], scale[1], scale[2]);

    // Create a copy of the input mesh
    pbd_object::Mesh transformed_mesh = mesh;

    // Apply the transformation matrix to each vertex in the mesh
    for(int i=0; i<transformed_mesh.vertices.rows(); i++)
    {
        Eigen::Matrix<Real,3,1> vertex = transformed_mesh.vertices.row(i).transpose();
        vertex = rotationMatrix * (vertex.cwiseProduct(scaleVector)) + translationVector;
        transformed_mesh.vertices.row(i) = vertex.transpose();
    }

    return transformed_mesh;
}

pbd_object::Mesh FabricSimulator::transformMesh(const pbd_object::Mesh &mesh, 
                                   const Eigen::Matrix<Real, 1, 3> &translation,
                                   const Eigen::Quaternion<Real> &rotation,
                                   const Eigen::Matrix<Real, 1, 3> &scale){
    // Translates, rotates, and scales a given mesh with Rotation given with Quaternion (overloaded)

    // Prepare translation and scale matrices
    Eigen::Matrix<Real,3,1> translationVector(translation[0], translation[1], translation[2]);
    Eigen::Matrix<Real,3,1> scaleVector(scale[0], scale[1], scale[2]);

    // Create a copy of the input mesh
    pbd_object::Mesh transformed_mesh = mesh;

    // Apply the transformation matrix to each vertex in the mesh
    for(int i=0; i<transformed_mesh.vertices.rows(); i++)
    {
        Eigen::Matrix<Real,3,1> vertex = transformed_mesh.vertices.row(i).transpose();
        vertex = rotation.normalized() * (vertex.cwiseProduct(scaleVector)) + translationVector;
        transformed_mesh.vertices.row(i) = vertex.transpose();
    }

    return transformed_mesh;
}


void FabricSimulator::simulate(const ros::TimerEvent& e){
    // With some kind of self lock to prevent collision with rendering
    boost::recursive_mutex::scoped_lock lock(mtx_);

    Real sdt = dt_ / num_substeps_; // substep time step size

    // std::chrono::high_resolution_clock::time_point start_time = high_resolution_clock::now();
    ros::Time start_time = ros::Time::now();

    // // -------------------------------
    // // Small steps implementation
    // for (int i = 0; i< num_steps_; i++){
    //     for (int j = 0; j < num_substeps_; j++){     
    //         fabric_.preSolve(sdt,gravity_);
 
    //         // Collision Handling, detect collisions
    //         if (is_collision_handling_enabled_){
    //             collision_handler_->collisionDetection();  
    //         }
    //         collision_handler_->solveContactPositionConstraints(sdt);
    //         fabric_.solve(sdt);
            
    //         fabric_.postSolve(sdt);

    //         collision_handler_->solveContactVelocityConstraints(sdt);
    //     }
    // }
    // // -------------------------------

    // --------------------------------------------------------------
    // Small steps (Substep XPBD) 2019 implementation 2 (CORRECT)
    for (int i = 0; i< num_substeps_; i++){
        fabric_.resetLambdas();
        
        fabric_.preSolve(sdt,gravity_);
        
        for (int j = 0; j < num_steps_; j++){     
            fabric_.resetForces();
            
            // Collision Handling, detect collisions
            if (is_collision_handling_enabled_){
                collision_handler_->collisionDetection();  
            }
            collision_handler_->solveContactPositionConstraints(sdt);
            fabric_.solve(sdt);
        }
        
        fabric_.postSolve(sdt);
        collision_handler_->solveContactVelocityConstraints(sdt);
    }
    // --------------------------------------------------------------
    
    // +++++++++++++++++++++++++++++++++++++++++++++++++++==
    readAttachedRobotForces();
    // +++++++++++++++++++++++++++++++++++++++++++++++++++==

    // std::chrono::high_resolution_clock::time_point finish_time = high_resolution_clock::now();
    ros::Time finish_time = ros::Time::now();
    // Real elapsed_time = duration_cast<microseconds>(finish_time - start_time).count() * 0.000001;
    ros::Duration elapsed_time = finish_time - start_time;
    time_sum_ += elapsed_time.toSec();
    time_frames_ += 1;
    if (time_frames_ > 2) {
        time_sum_ /= time_frames_;
        ROS_INFO_THROTTLE(5, "[Fabric Simulator]: %-4.2lf ms per simulation iteration", time_sum_*1000);
        // Smart dt and simulation rate selection (debug)
        if (!is_auto_sim_rate_set_ && set_sim_rate_auto_) {
            dt_ = time_sum_;
            timer_simulate_.setPeriod(ros::Duration(time_sum_));
            is_auto_sim_rate_set_ = true; 
            std::cout << "Automatic setup. dt = " << dt_ << " seconds." << std::endl;
        }
        time_frames_ = 0;
        time_sum_ = 0;
    }
}


void FabricSimulator::render(const ros::TimerEvent& e){
    if (fabric_visualization_mode_ == -1){
        return;
    }
    
    // Lock to prevent collision with simulation
    boost::recursive_mutex::scoped_lock lock(mtx_);

    int marker_id = 0;
    
    // Get the pointers to the data from the fabric object
    const Eigen::Matrix<Real,Eigen::Dynamic,3> *pos_ptr = fabric_.getPosPtr();
    const Eigen::MatrixX2i *stretching_ids_ptr = fabric_.getStretchingIdsPtr();
    const Eigen::MatrixX3i *face_tri_ids_ptr = fabric_.getFaceTriIdsPtr();

    // Call the new renderMarkers function
    renderMarkers(pos_ptr, stretching_ids_ptr, face_tri_ids_ptr, pub_fabric_marker_array_, marker_id);
}

void FabricSimulator::renderMarkers(const Eigen::Matrix<Real,Eigen::Dynamic,3>* pos_ptr,
                                    const Eigen::MatrixX2i* stretching_ids_ptr,
                                    const Eigen::MatrixX3i* face_tri_ids_ptr,
                                    ros::Publisher& publisher,
                                    int marker_id = 0)
{
    if (fabric_visualization_mode_ == -1){
        return;
    }

    visualization_msgs::MarkerArray markerArray;
    // int fabric_visualization_mode_; 
    // 0: Points Only, 
    // 1: Wireframe Only,
    // 2: Mesh Only,
    // 3: Points and Wireframe,
    // 4: All

    visualization_msgs::Marker pointsMarker, wireframeMarker, trianglesMarker;
    createRvizPointsMarker(pos_ptr, pointsMarker);
    createRvizWireframeMarker(pos_ptr, stretching_ids_ptr, wireframeMarker);
    createRvizTrianglesMarker(pos_ptr, face_tri_ids_ptr, trianglesMarker);

    if (fabric_visualization_mode_ == 0 || 
        fabric_visualization_mode_ == 3 || 
        fabric_visualization_mode_ == 4) {
        pointsMarker.id = marker_id++;
        markerArray.markers.push_back(pointsMarker);
    }
    if (fabric_visualization_mode_ == 1 || 
        fabric_visualization_mode_ == 3 || 
        fabric_visualization_mode_ == 4) {
        wireframeMarker.id = marker_id++;
        markerArray.markers.push_back(wireframeMarker);
    }
    if (fabric_visualization_mode_ == 2 || 
        fabric_visualization_mode_ == 4) {
        trianglesMarker.id = marker_id++;
        markerArray.markers.push_back(trianglesMarker);
    }

    publisher.publish(markerArray);
}

void FabricSimulator::createRvizPointsMarker(const Eigen::Matrix<Real,Eigen::Dynamic,3> *poses, visualization_msgs::Marker &marker){
    marker.header.frame_id = fabric_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = point_marker_scale_;
    marker.scale.y = point_marker_scale_;
    marker.scale.z = point_marker_scale_;

    // marker.color.r = point_marker_color_rgba_[0];
    // marker.color.g = point_marker_color_rgba_[1];
    // marker.color.b = point_marker_color_rgba_[2];
    // marker.color.a = point_marker_color_rgba_[3];

    for (int i = 0; i < poses->rows(); i++) {
        geometry_msgs::Point p;
        p.x = (*poses)(i, 0);
        p.y = (*poses)(i, 1);
        p.z = (*poses)(i, 2);
        marker.points.push_back(p);

        std_msgs::ColorRGBA c;
        if(fabric_.isStaticParticle(i)) {
            // Color for static points
            c.r = static_point_marker_color_rgba_[0];
            c.g = static_point_marker_color_rgba_[1];
            c.b = static_point_marker_color_rgba_[2];
            c.a = static_point_marker_color_rgba_[3];
        } else {
            // Color for dynamic points
            c.r = point_marker_color_rgba_[0];
            c.g = point_marker_color_rgba_[1];
            c.b = point_marker_color_rgba_[2];
            c.a = point_marker_color_rgba_[3];
        }
        marker.colors.push_back(c);
    }
}

void FabricSimulator::createRvizWireframeMarker(const Eigen::Matrix<Real,Eigen::Dynamic,3> *poses, const Eigen::MatrixX2i *ids, visualization_msgs::Marker &marker){
    marker.header.frame_id = fabric_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.005*line_marker_scale_multiplier_;

    marker.color.r = line_marker_color_rgba_[0];
    marker.color.g = line_marker_color_rgba_[1];
    marker.color.b = line_marker_color_rgba_[2];
    marker.color.a = line_marker_color_rgba_[3];

    for (int i = 0; i < ids->rows(); i++) {
        const int &id0 = (*ids)(i, 0);
        const int &id1 = (*ids)(i, 1);

        geometry_msgs::Point p1, p2;
        p1.x = (*poses)(id0, 0);
        p1.y = (*poses)(id0, 1);
        p1.z = (*poses)(id0, 2);
        p2.x = (*poses)(id1, 0);
        p2.y = (*poses)(id1, 1);
        p2.z = (*poses)(id1, 2);

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }
}

void FabricSimulator::createRvizTrianglesMarker(const Eigen::Matrix<Real,Eigen::Dynamic,3> *poses, const Eigen::MatrixX3i *face_tri_ids, visualization_msgs::Marker &marker){
    marker.header.frame_id = fabric_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = mesh_marker_color_rgba_[0]; 
    marker.color.g = mesh_marker_color_rgba_[1];
    marker.color.b = mesh_marker_color_rgba_[2];
    marker.color.a = mesh_marker_color_rgba_[3];

    for (int i = 0; i < face_tri_ids->rows(); i++) {
        int id0 = (*face_tri_ids)(i, 0);
        int id1 = (*face_tri_ids)(i, 1);
        int id2 = (*face_tri_ids)(i, 2);

        // Swap id1 and id2 to fix the direction of mesh color
        std::swap(id1, id2);

        geometry_msgs::Point p0, p1, p2;
        p0.x = (*poses)(id0, 0);
        p0.y = (*poses)(id0, 1);
        p0.z = (*poses)(id0, 2);

        p1.x = (*poses)(id1, 0);
        p1.y = (*poses)(id1, 1);
        p1.z = (*poses)(id1, 2);

        p2.x = (*poses)(id2, 0);
        p2.y = (*poses)(id2, 1);
        p2.z = (*poses)(id2, 2);

        marker.points.push_back(p0);
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }
}

// This function is not for rendering, it's practically for publishing the state of the fabric
void FabricSimulator::publishFaceTriIds(const Eigen::MatrixX3i *ids){
    std_msgs::Int32MultiArray array;

    for (int i = 0; i < ids->rows(); i++) {
        const int &id0 = (*ids)(i, 0);
        const int &id1 = (*ids)(i, 1);
        const int &id2 = (*ids)(i, 2);

        // Append the ids to the array
        array.data.push_back(id0);
        array.data.push_back(id1);
        array.data.push_back(id2);
    }

    // Publish the array
    pub_face_tri_ids_.publish(array);
}

void FabricSimulator::odometryCb_custom_static_particles(const nav_msgs::Odometry::ConstPtr& odom_msg, const int& id) {
    // With some kind of self lock to prevent collision with simulation
    boost::recursive_mutex::scoped_lock lock(mtx_);

    // Update Velocity
    const Eigen::Matrix<Real,1,3> vel(odom_msg->twist.twist.linear.x, 
                                        odom_msg->twist.twist.linear.y, 
                                        odom_msg->twist.twist.linear.z);

    // Update Pose
    const Real x = odom_msg->pose.pose.position.x;
    const Real y = odom_msg->pose.pose.position.y;
    const Real z = odom_msg->pose.pose.position.z + fabric_rob_z_offset_;

    const Real qw = odom_msg->pose.pose.orientation.w;
    const Real qx = odom_msg->pose.pose.orientation.x;
    const Real qy = odom_msg->pose.pose.orientation.y;
    const Real qz = odom_msg->pose.pose.orientation.z;
    
    Eigen::Matrix<Real,1,3> pos(x, y, z);
    Eigen::Quaternion<Real> ori(qw,qx,qy,qz);

    fabric_.updateAttachedVelocity(id, vel);
    fabric_.updateAttachedPose(id, pos);
}

void FabricSimulator::odometryCb_external(const nav_msgs::Odometry::ConstPtr& odom_msg, const std::string& topic) {
    // With some kind of self lock to prevent collision with simulation
    boost::recursive_mutex::scoped_lock lock(mtx_);

    auto it = external_odom_attachments_.find(topic);
    if (it == external_odom_attachments_.end())
    {
        // This shouldn't happen if we remove from map cleanly,
        // but just in case.
        ROS_WARN_STREAM("No external_odom_attachment attachment found for key: " << topic);
        return;
    }

    ExternalOdomAttachment &external_odom_attachment = it->second;

    // Convert Pose to Eigen position.
    Eigen::Matrix<Real,1,3> pos(odom_msg->pose.pose.position.x, 
                                odom_msg->pose.pose.position.y, 
                                odom_msg->pose.pose.position.z);

    // Convert orientation (if needed).
    Eigen::Quaternion<Real> ori(odom_msg->pose.pose.orientation.w, 
                                odom_msg->pose.pose.orientation.x, 
                                odom_msg->pose.pose.orientation.y, 
                                odom_msg->pose.pose.orientation.z);
    ori.normalize();

    
    if (!external_odom_attachment.is_attached) // Perform the attach logic
    {
        // // tell sim objects (fabric) to attach external_odom_attachment to the nearest particle
        // external_odom_attachment.attached_id = fabric_.attachNearest(pos);
        // std::cout << "external_odom_attachment.attached_id: " << external_odom_attachment.attached_id << std::endl;

        // if (external_odom_attachment.attached_id != -1)
        // {
        //     external_odom_attachment.is_attached = true;
        // }

        fabric_.attachWithinRadius(pos, robot_attach_radius_,
                                            external_odom_attachment.attached_ids,
                                            external_odom_attachment.attached_rel_poses);
        
        if (!external_odom_attachment.attached_ids.empty()) { // if success
            external_odom_attachment.attached_id = external_odom_attachment.attached_ids[0]; 
            external_odom_attachment.attached_orient = ori;

            // Also update the default wrench frame id with the frame of the odom_msg if child frame id is not empty,
            // otherwise, keep the default wrench frame id
            if (!odom_msg->child_frame_id.empty()) {
                external_odom_attachment.wrench_frame_id = odom_msg->child_frame_id;
            }
        }

        external_odom_attachment.is_attached = true;
    }
    else
    {
        // // tell sim object to update its position
        // fabric_.updateAttachedPose(external_odom_attachment.attached_id, pos);

        // The update logic if already attached
        fabric_.updateAttachedPoses(external_odom_attachment.attached_ids, pos,
                                    external_odom_attachment.attached_rel_poses,
                                    ori, external_odom_attachment.attached_orient);
    }
}

void FabricSimulator::odometryCb_01(const nav_msgs::Odometry::ConstPtr& odom_msg){
    // // With some kind of self lock to prevent collision with simulation
    boost::recursive_mutex::scoped_lock lock(mtx_);

    Real x = odom_msg->pose.pose.position.x;
    Real y = odom_msg->pose.pose.position.y;
    Real z = odom_msg->pose.pose.position.z + fabric_rob_z_offset_;
    
    Eigen::Matrix<Real,1,3> pos(x, y, z);

    Real qw = odom_msg->pose.pose.orientation.w;
    Real qx = odom_msg->pose.pose.orientation.x;
    Real qy = odom_msg->pose.pose.orientation.y;
    Real qz = odom_msg->pose.pose.orientation.z;
    Eigen::Quaternion<Real> cur_orient(qw, qx,qy, qz);

    if (!is_rob_01_attached_)
    {
        // // tell sim objects (fabric) to attach robot to the nearest particle
        // rob_01_attached_id_ = fabric_.attachNearest(pos);
        // std::cout << "self.rob_01_attached_id, " << rob_01_attached_id_ << std::endl;

        // if (rob_01_attached_id_ != -1)
        // {
        //     is_rob_01_attached_ = true;
        // }

        rob_01_attached_orient_ = cur_orient.normalized();

        // tell sim objects (fabric) to attach robot to the nearest particles within radius
        fabric_.attachNearestWithRadius(pos, robot_attach_radius_, rob_01_attached_ids_, rob_01_attached_rel_poses_);

        if (!rob_01_attached_ids_.empty()){
            is_rob_01_attached_ = true;
            rob_01_attached_id_ = rob_01_attached_ids_[0];
        }
    }
    else
    {
        // // tell sim object to update its position
        // fabric_.updateAttachedPose(rob_01_attached_id_, pos);

        fabric_.updateAttachedPoses(rob_01_attached_ids_, pos, rob_01_attached_rel_poses_, cur_orient, rob_01_attached_orient_);
    }
}

void FabricSimulator::odometryCb_02(const nav_msgs::Odometry::ConstPtr& odom_msg){
    // // With some kind of self lock to prevent collision with simulation
    boost::recursive_mutex::scoped_lock lock(mtx_);

    Real x = odom_msg->pose.pose.position.x;
    Real y = odom_msg->pose.pose.position.y;
    Real z = odom_msg->pose.pose.position.z + fabric_rob_z_offset_;
    
    Eigen::Matrix<Real,1,3> pos(x, y, z);

    Real qw = odom_msg->pose.pose.orientation.w;
    Real qx = odom_msg->pose.pose.orientation.x;
    Real qy = odom_msg->pose.pose.orientation.y;
    Real qz = odom_msg->pose.pose.orientation.z;
    Eigen::Quaternion<Real> cur_orient(qw, qx,qy, qz);

    if (!is_rob_02_attached_)
    {
        // // tell sim objects (fabric) to attach robot to the nearest particle
        // rob_02_attached_id_ = fabric_.attachNearest(pos);
        // // std::cout << "self.rob_02_attached_id, " << rob_02_attached_id_ << std::endl;

        // if (rob_02_attached_id_ != -1)
        // {
        //     is_rob_02_attached_ = true;
        // }

        rob_02_attached_orient_ = cur_orient.normalized();

        // tell sim objects (fabric) to attach robot to the nearest particles within radius
        fabric_.attachNearestWithRadius(pos, robot_attach_radius_, rob_02_attached_ids_, rob_02_attached_rel_poses_);

        if (!rob_02_attached_ids_.empty()){
            is_rob_02_attached_ = true;
            rob_02_attached_id_ = rob_02_attached_ids_[0];
        }
    }
    else
    {
        // tell sim object to update its position
        // fabric_.updateAttachedPose(rob_02_attached_id_, pos);

        fabric_.updateAttachedPoses(rob_02_attached_ids_, pos, rob_02_attached_rel_poses_, cur_orient, rob_02_attached_orient_);
    }
}

void FabricSimulator::odometryCb_03(const nav_msgs::Odometry::ConstPtr& odom_msg){
    // // With some kind of self lock to prevent collision with simulation
    boost::recursive_mutex::scoped_lock lock(mtx_);

    Real x = odom_msg->pose.pose.position.x;
    Real y = odom_msg->pose.pose.position.y;
    Real z = odom_msg->pose.pose.position.z + fabric_rob_z_offset_;
    
    Eigen::Matrix<Real,1,3> pos(x, y, z);

    Real qw = odom_msg->pose.pose.orientation.w;
    Real qx = odom_msg->pose.pose.orientation.x;
    Real qy = odom_msg->pose.pose.orientation.y;
    Real qz = odom_msg->pose.pose.orientation.z;
    Eigen::Quaternion<Real> cur_orient(qw, qx,qy, qz);

    if (!is_rob_03_attached_)
    {
        // // tell sim objects (fabric) to attach robot to the nearest particle
        // rob_03_attached_id_ = fabric_.attachNearest(pos);
        // // std::cout << "self.rob_03_attached_id, " << rob_03_attached_id_ << std::endl;

        // if (rob_03_attached_id_ != -1)
        // {
        //     is_rob_03_attached_ = true;
        // }

        rob_03_attached_orient_ = cur_orient.normalized();

        // tell sim objects (fabric) to attach robot to the nearest particles within radius
        fabric_.attachNearestWithRadius(pos, robot_attach_radius_, rob_03_attached_ids_, rob_03_attached_rel_poses_);

        if (!rob_03_attached_ids_.empty()){
            is_rob_03_attached_ = true;
            rob_03_attached_id_ = rob_03_attached_ids_[0];
        }
    }
    else
    {
        // tell sim object to update its position
        // fabric_.updateAttachedPose(rob_03_attached_id_, pos);

        fabric_.updateAttachedPoses(rob_03_attached_ids_, pos, rob_03_attached_rel_poses_, cur_orient, rob_03_attached_orient_);
    }
}

void FabricSimulator::odometryCb_04(const nav_msgs::Odometry::ConstPtr& odom_msg){
    // // With some kind of self lock to prevent collision with simulation
    boost::recursive_mutex::scoped_lock lock(mtx_);

    Real x = odom_msg->pose.pose.position.x;
    Real y = odom_msg->pose.pose.position.y;
    Real z = odom_msg->pose.pose.position.z + fabric_rob_z_offset_;
    
    Eigen::Matrix<Real,1,3> pos(x, y, z);

    Real qw = odom_msg->pose.pose.orientation.w;
    Real qx = odom_msg->pose.pose.orientation.x;
    Real qy = odom_msg->pose.pose.orientation.y;
    Real qz = odom_msg->pose.pose.orientation.z;
    Eigen::Quaternion<Real> cur_orient(qw, qx,qy, qz);

    if (!is_rob_04_attached_)
    {
        // // tell sim objects (fabric) to attach robot to the nearest particle
        // rob_04_attached_id_ = fabric_.attachNearest(pos);
        // // std::cout << "self.rob_04_attached_id, " << rob_04_attached_id_ << std::endl;

        // if (rob_04_attached_id_ != -1)
        // {
        //     is_rob_04_attached_ = true;
        // }

        rob_04_attached_orient_ = cur_orient.normalized();

        // tell sim objects (fabric) to attach robot to the nearest particles within radius
        fabric_.attachNearestWithRadius(pos, robot_attach_radius_, rob_04_attached_ids_, rob_04_attached_rel_poses_);

        if (!rob_04_attached_ids_.empty()){
            is_rob_04_attached_ = true;
            rob_04_attached_id_ = rob_04_attached_ids_[0];
        }
    }
    else
    {
        // tell sim object to update its position
        // fabric_.updateAttachedPose(rob_04_attached_id_, pos);

        fabric_.updateAttachedPoses(rob_04_attached_ids_, pos, rob_04_attached_rel_poses_, cur_orient, rob_04_attached_orient_);
    }
}

void FabricSimulator::cmdVelCb_custom_static_particles(const geometry_msgs::Twist::ConstPtr& twist_msg, const int& id) {
    // With some kind of self lock to prevent collision with simulation
    boost::recursive_mutex::scoped_lock lock(mtx_);

    // Update Velocity
    const Eigen::Matrix<Real,1,3> vel(twist_msg->linear.x, 
                                        twist_msg->linear.y, 
                                        twist_msg->linear.z);

    fabric_.updateAttachedVelocity(id, vel);
}

void FabricSimulator::readAttachedRobotForces(){
    const Eigen::Matrix<Real,Eigen::Dynamic,3> &for_ptr = *fabric_.getForPtr();

    for (auto & kv : external_odom_attachments_)
    {
        ExternalOdomAttachment &external_odom_attachment = kv.second;
        if (external_odom_attachment.is_attached && !external_odom_attachment.attached_ids.empty())
        {
            // For simplicity, if you attach only one ID, use it directly:
            // external_odom_attachment.attached_force = for_ptr.row(external_odom_attachment.attached_id);
            
            // OR if you have multiple attached IDs, you might average them or sum them
            // as needed:
            Eigen::Matrix<Real,1,3> accum = Eigen::Matrix<Real,1,3>::Zero();
            for (int pid : external_odom_attachment.attached_ids)
            {
                // Log the forces on the attached particles with ros_info
                // ROS_INFO_STREAM("External Odom Attachment: " << kv.first << ". ID: " << pid << ". Force = " << for_ptr.row(pid) << " N.");

                accum += for_ptr.row(pid);
            }

            external_odom_attachment.attached_force = accum;
        }
        else
        {
            // ROS_WARN_STREAM("External Odom Attachment: " << kv.first << " is not attached to any particle.");
            external_odom_attachment.attached_force.setZero();
        }
    }

    if (is_rob_01_attached_){
        rob_01_attached_force_ = for_ptr.row(rob_01_attached_id_);
    }
    if (is_rob_02_attached_){
        rob_02_attached_force_ = for_ptr.row(rob_02_attached_id_);
    }
    if (is_rob_03_attached_){
        rob_03_attached_force_ = for_ptr.row(rob_03_attached_id_);
    }
    if (is_rob_04_attached_){
        rob_04_attached_force_ = for_ptr.row(rob_04_attached_id_);
    }
}

// Publish forces on each robot/external_odom_attachment by the fabric
void FabricSimulator::publishWrenchesOnExternalOdoms(const ros::TimerEvent& e){
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();

    for (auto & kv : external_odom_attachments_)
    {
        ExternalOdomAttachment &external_odom_attachment = kv.second;
        // only publish for attached robots
        if (!external_odom_attachment.is_attached){
            continue;
        }  

        msg.header.frame_id = external_odom_attachment.wrench_frame_id;   // user-defined or param
        msg.wrench.force.x  = external_odom_attachment.attached_force(0);
        msg.wrench.force.y  = external_odom_attachment.attached_force(1);
        msg.wrench.force.z  = external_odom_attachment.attached_force(2);

        // Torque // TODO: calculate the torque from the forces and the relative poses, for now, just zero
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;

        external_odom_attachment.wrench_pub.publish(msg);
    }

    if (is_rob_01_attached_){
        // std::cout << "id: " << rob_01_attached_id_ << ". Force = " << rob_01_attached_force_ << " N." << std::endl;

        msg.header.frame_id = wrench_01_frame_id_;
        msg.wrench.force.x = rob_01_attached_force_(0);
        msg.wrench.force.y = rob_01_attached_force_(1);
        msg.wrench.force.z = rob_01_attached_force_(2);
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
        pub_wrench_stamped_01_.publish(msg);
    }
    if (is_rob_02_attached_){
        // std::cout << "id: " << rob_02_attached_id_ << ". Force = " << rob_02_attached_force_ << " N." << std::endl;

        msg.header.frame_id = wrench_02_frame_id_;
        msg.wrench.force.x = rob_02_attached_force_(0);
        msg.wrench.force.y = rob_02_attached_force_(1);
        msg.wrench.force.z = rob_02_attached_force_(2);
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
        pub_wrench_stamped_02_.publish(msg);
    }
    if (is_rob_03_attached_){
        // std::cout << "id: " << rob_03_attached_id_ << ". Force = " << rob_03_attached_force_ << " N." << std::endl;

        msg.header.frame_id = wrench_03_frame_id_;
        msg.wrench.force.x = rob_03_attached_force_(0);
        msg.wrench.force.y = rob_03_attached_force_(1);
        msg.wrench.force.z = rob_03_attached_force_(2);
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
        pub_wrench_stamped_03_.publish(msg);
    }
    if (is_rob_04_attached_){
        // std::cout << "id: " << rob_04_attached_id_ << ". Force = " << rob_04_attached_force_ << " N." << std::endl;

        msg.header.frame_id = wrench_04_frame_id_;
        msg.wrench.force.x = rob_04_attached_force_(0);
        msg.wrench.force.y = rob_04_attached_force_(1);
        msg.wrench.force.z = rob_04_attached_force_(2);
        msg.wrench.torque.x = 0.0;
        msg.wrench.torque.y = 0.0;
        msg.wrench.torque.z = 0.0;
        pub_wrench_stamped_04_.publish(msg);
    }    
}

// Publish forces on each robot/external_odom_attachment by the fabric
void FabricSimulator::publishZeroWrenches(){
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();

    msg.wrench.force.x = 0.0;
    msg.wrench.force.y = 0.0;
    msg.wrench.force.z = 0.0;
    msg.wrench.torque.x = 0.0;
    msg.wrench.torque.y = 0.0;
    msg.wrench.torque.z = 0.0;

    for (auto & kv : external_odom_attachments_)
    {
        ExternalOdomAttachment &external_odom_attachment = kv.second;

        if (!external_odom_attachment.is_attached){
            continue;
        }

        msg.header.frame_id = external_odom_attachment.wrench_frame_id;
        external_odom_attachment.wrench_pub.publish(msg);
    }

    if (is_rob_01_attached_){
        // std::cout << "id: " << rob_01_attached_id_ << ". Force = " << rob_01_attached_force_ << " N." << std::endl;
        msg.header.frame_id = wrench_01_frame_id_;
        pub_wrench_stamped_01_.publish(msg);
    }
    if (is_rob_02_attached_){
        // std::cout << "id: " << rob_02_attached_id_ << ". Force = " << rob_02_attached_force_ << " N." << std::endl;
        msg.header.frame_id = wrench_02_frame_id_;
        pub_wrench_stamped_02_.publish(msg);
    }
    if (is_rob_03_attached_){
        // std::cout << "id: " << rob_03_attached_id_ << ". Force = " << rob_03_attached_force_ << " N." << std::endl;
        msg.header.frame_id = wrench_03_frame_id_;
        pub_wrench_stamped_03_.publish(msg);
    }
    if (is_rob_04_attached_){
        // std::cout << "id: " << rob_04_attached_id_ << ". Force = " << rob_04_attached_force_ << " N." << std::endl;
        msg.header.frame_id = wrench_04_frame_id_;
        pub_wrench_stamped_04_.publish(msg);
    }    
}



// Publish message to RVIZ to visualize the rigid bodies considered in the simulation
void FabricSimulator::renderRigidBodies(const ros::TimerEvent& e){
    if (rb_visualization_mode_ == -1){
        return;
    }

    visualization_msgs::MarkerArray markerArray;
    marker_id_ = 0; 

    // int rb_visualization_mode_ = 0; // 0: Mesh Only, 1: Wireframe Only, 2: Both

    for (const utilities::RigidBodySceneLoader::RigidBodyData& rbd : rigid_bodies_) {
        if (!rbd.m_isVisible) {
            continue;
        }
        visualization_msgs::Marker meshMarker, wireframeMarker;
        createMeshAndWireframeMarkers(&rbd.m_mesh.vertices, &rbd.m_mesh.face_tri_ids, meshMarker, wireframeMarker);

        if (rb_visualization_mode_ == 0 || rb_visualization_mode_ == 2) {
            meshMarker.id = marker_id_++;
            markerArray.markers.push_back(meshMarker);
        }
        if (rb_visualization_mode_ == 1 || rb_visualization_mode_ == 2) {
            wireframeMarker.id = marker_id_++;
            markerArray.markers.push_back(wireframeMarker);
        }
    }

    // Publish the entire marker array
    pub_rb_marker_array_.publish(markerArray);

}

void FabricSimulator::createMeshAndWireframeMarkers(const Eigen::Matrix<Real,Eigen::Dynamic,3> *vertices, 
                                                    const Eigen::MatrixX3i *face_tri_ids, 
                                                    visualization_msgs::Marker &meshMarker, 
                                                    visualization_msgs::Marker &wireframeMarker){
    // Set up mesh marker (triangles)
    setupMeshMarker(meshMarker);
    // Set up wireframe marker (lines)
    setupWireframeMarker(wireframeMarker);

    for (int i = 0; i < face_tri_ids->rows(); i++) {
        for (int j = 0; j < 3; j++) {
            int idx = (*face_tri_ids)(i, j);
            geometry_msgs::Point p;
            p.x = (*vertices)(idx, 0);
            p.y = (*vertices)(idx, 1);
            p.z = (*vertices)(idx, 2);
            meshMarker.points.push_back(p);

            // Wireframe points
            int next_idx = (*face_tri_ids)(i, (j + 1) % 3);
            geometry_msgs::Point p2;
            p2.x = (*vertices)(next_idx, 0);
            p2.y = (*vertices)(next_idx, 1);
            p2.z = (*vertices)(next_idx, 2);
            wireframeMarker.points.push_back(p);
            wireframeMarker.points.push_back(p2);
        }
    }
}

void FabricSimulator::setupMeshMarker(visualization_msgs::Marker &marker){
    marker.header.frame_id = fabric_frame_id_;
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    // marker.id = 1; ID IS SET IN renderRigidBodies FUNCTION
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.orientation.w = 1.0;
    
    // marker.points = points; POINTS ARE PUSHED IN createMeshAndWireframeMarkers FUNCTION
    
    // Set other properties like color, scale, etc.
    marker.scale.x = 1.0; // As it's a list of triangles, scale shouldn't matter
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = rb_mesh_marker_color_rgba_[0]; 
    marker.color.g = rb_mesh_marker_color_rgba_[1];
    marker.color.b = rb_mesh_marker_color_rgba_[2];
    marker.color.a = rb_mesh_marker_color_rgba_[3];
}

void FabricSimulator::setupWireframeMarker(visualization_msgs::Marker &marker){
    marker.header.frame_id = fabric_frame_id_;
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::LINE_LIST;
    // marker.id = 1; ID IS SET IN renderRigidBodies FUNCTION
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.orientation.w = 1.0;
    
    // marker.points = points; POINTS ARE PUSHED IN createMeshAndWireframeMarkers FUNCTION

    // Set other properties like color, scale, etc.
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    marker.scale.x = 0.005*rb_line_marker_scale_multiplier_;

    marker.color.r = rb_line_marker_color_rgba_[0];
    marker.color.g = rb_line_marker_color_rgba_[1];
    marker.color.b = rb_line_marker_color_rgba_[2];
    marker.color.a = rb_line_marker_color_rgba_[3];
}

// DEBUG TEST
// drawRvizMesh_from_resource(rbd.m_modelFile);

// THIS FUNCTION IS NOT USED HOWEVER PUT HERE AS A FUTURE REFERENCE
void FabricSimulator::drawRvizMesh_from_resource(const std::string &mesh_resource){
    // THIS FUNCTION IS NOT USED HOWEVER PUT HERE AS A FUTURE REFERENCE
    visualization_msgs::Marker m;

    m.header.frame_id = fabric_frame_id_;
    m.header.stamp = ros::Time::now();

    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.id = 2;  // Change to an unused id
    m.action = visualization_msgs::Marker::ADD;

    m.pose.orientation.w = 1.0;

    // NOTE: The format is the URI-form used by resource_retriever, including the package:// syntax.
    // e.g., m.mesh_resource = "package://package_name/data/mesh/cube.obj"; 
    // hence providing a full path (e.g., "/home/usename/catkin_ws/src/package_name/data/mesh/cube.obj") does NOT work!
    m.mesh_resource = mesh_resource; 
    m.mesh_use_embedded_materials = true; // Need this to use textures for mesh

    m.scale.x = 1.0; // You can edit the visualized scale here
    m.scale.y = 1.0;
    m.scale.z = 1.0;

    // even when mesh_use_embedded_materials is true, 
    // if the marker color is set to anything other than r=0,g=0,b=0,a=0 
    // the marker color and alpha will be used to tint the mesh with the embedded material.
    m.color.r = rb_mesh_marker_color_rgba_[0]; 
    m.color.g = rb_mesh_marker_color_rgba_[1];
    m.color.b = rb_mesh_marker_color_rgba_[2];
    m.color.a = rb_mesh_marker_color_rgba_[3];

    // pub_rb_marker_array_.publish(m);  
}


void FabricSimulator::contactCallbackFunction(const unsigned int contactType,
                                              const unsigned int bodyIndex1, 
                                              const unsigned int bodyIndex2,
                                              const Eigen::Matrix<Real, 3, 1> &cp1, 
                                              const Eigen::Matrix<Real, 3, 1> &cp2,
                                              const Eigen::Matrix<Real, 3, 1> &normal, 
                                              const Real dist,
                                              const Real restitutionCoeff, 
                                              const Real frictionCoeffStatic,
                                              const Real frictionCoeffDynamic,
                                              void *userData)
{
    FabricSimulator *fabricSimulator = (FabricSimulator*)userData; 

	if (contactType == utilities::CollisionHandler::RigidBodyContactType)
    {
        fabricSimulator->collision_handler_->addRigidBodyContactConstraint(bodyIndex1, 
                                                          bodyIndex2, 
                                                          cp1,
                                                          cp2,
                                                          normal, 
                                                          dist,
                                                          restitutionCoeff,
                                                          frictionCoeffStatic,
                                                          frictionCoeffDynamic);
    }
	else if (contactType == utilities::CollisionHandler::ParticleRigidBodyContactType)
    {
        fabricSimulator->collision_handler_->addParticleRigidBodyContactConstraint(bodyIndex1, 
                                                                  bodyIndex2, 
                                                                  cp1, 
                                                                  cp2, 
                                                                  normal, 
                                                                  dist, 
                                                                  restitutionCoeff, 
                                                                  frictionCoeffStatic,
                                                                  frictionCoeffDynamic);
    }
		
}


// Publish the minimum distances to rigid bodies in the simulation message
void FabricSimulator::publishMinDistancesToRigidBodies(const ros::TimerEvent& e){
    // With some kind of self lock to prevent collision with rendering
    boost::recursive_mutex::scoped_lock lock(mtx_);
    
    std::vector<std::vector<utilities::CollisionHandler::MinDistanceData>> min_distances_mt;
    collision_handler_->computeMinDistancesToRigidBodies(min_distances_mt);

    // // Print min distance data for debug
    // // Iterate through all threads
    // for (size_t threadIdx = 0; threadIdx < min_distances_mt.size(); ++threadIdx) {
    //     // std::cout << "Thread " << threadIdx << ":" << std::endl;        
    //     // Iterate through all minimum distance data in the current thread
    //     for (size_t i = 0; i < min_distances_mt[threadIdx].size(); ++i) {
    //         const auto& minDistData = min_distances_mt[threadIdx][i];
    //         std::cout << "  Min Distance Data " << i << ":" << std::endl;
    //         // std::cout << "    Type: " << static_cast<int>(minDistData.m_type) << std::endl;
    //         // std::cout << "    Index on Object 1: " << minDistData.m_index1 << std::endl;
    //         // std::cout << "    Index on Object 2: " << minDistData.m_index2 << std::endl;
    //         std::cout << "    Minimum Distance: " << minDistData.m_minDistance << std::endl;
    //         // std::cout << "    Point on Object 1: (" 
    //                 //   << minDistData.m_pointOnObject1.x() << ", "
    //                 //   << minDistData.m_pointOnObject1.y() << ", "
    //                 //   << minDistData.m_pointOnObject1.z() << ")" << std::endl;
    //         // std::cout << "    Point on Object 2: (" 
    //                 //   << minDistData.m_pointOnObject2.x() << ", "
    //                 //   << minDistData.m_pointOnObject2.y() << ", "
    //                 //   << minDistData.m_pointOnObject2.z() << ")" << std::endl;
    //     }
    // }

    //Create a publisher for the min_distances data
    fabric_simulator::MinDistanceDataArray min_distances_msg;

    for (const auto& thread_data : min_distances_mt) {
        for (const auto& minDistData : thread_data) {
            fabric_simulator::MinDistanceData msg_data;

            // Fill the message fields
            msg_data.header.stamp = ros::Time::now();

            msg_data.type = minDistData.m_type;
            
            // msg_data.index1 = minDistData.m_index1; // index of the particle in dlo
            msg_data.index1 = 0; // index of the dlo in the scene, when there is single dlo in the simulation, it's always 0. 
            msg_data.index2 = minDistData.m_index2;

            msg_data.pointOnObject1.x = minDistData.m_pointOnObject1[0];
            msg_data.pointOnObject1.y = minDistData.m_pointOnObject1[1];
            msg_data.pointOnObject1.z = minDistData.m_pointOnObject1[2];

            msg_data.pointOnObject2.x = minDistData.m_pointOnObject2[0];
            msg_data.pointOnObject2.y = minDistData.m_pointOnObject2[1];
            msg_data.pointOnObject2.z = minDistData.m_pointOnObject2[2];

            // For the normal vector
            msg_data.normal.x = minDistData.m_normal.x();
            msg_data.normal.y = minDistData.m_normal.y();
            msg_data.normal.z = minDistData.m_normal.z();

            msg_data.minDistance = minDistData.m_minDistance;

            min_distances_msg.data.push_back(msg_data);
        }
    }

    pub_min_dist_to_rb_.publish(min_distances_msg);

    //-------------------------------------------------------------------------
    // Visualize the min distance line segments if desired:
    if (visualize_min_distances_) {
        publishMinDistLineMarkers(min_distances_mt);
    }
}

void FabricSimulator::publishMinDistLineMarkers(
    const std::vector<std::vector<utilities::CollisionHandler::MinDistanceData>>& min_distances_mt) {

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker line_marker;
    setupMinDistLineMarker(line_marker);

    int marker_id = 0;
    for (const auto& thread_data : min_distances_mt) {
        for (const auto& minDistData : thread_data) {
            line_marker.id = marker_id++;

            geometry_msgs::Point p1, p2;
            p1.x = minDistData.m_pointOnObject1[0];
            p1.y = minDistData.m_pointOnObject1[1];
            p1.z = minDistData.m_pointOnObject1[2];

            p2.x = minDistData.m_pointOnObject2[0];
            p2.y = minDistData.m_pointOnObject2[1];
            p2.z = minDistData.m_pointOnObject2[2];

            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);

            marker_array.markers.push_back(line_marker);
            line_marker.points.clear();
        }
    }

    pub_min_dist_marker_array_.publish(marker_array);
}

void FabricSimulator::setupMinDistLineMarker(visualization_msgs::Marker &marker){
    marker.header.frame_id = fabric_frame_id_;
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.005 * min_dist_line_marker_scale_multiplier_;

    marker.color.r = min_dist_line_marker_color_rgba_[0];
    marker.color.g = min_dist_line_marker_color_rgba_[1];
    marker.color.b = min_dist_line_marker_color_rgba_[2];
    marker.color.a = min_dist_line_marker_color_rgba_[3];
}

void FabricSimulator::publishFabricState(const ros::TimerEvent& e){
    // With some kind of self lock to prevent collision with simulation
    boost::recursive_mutex::scoped_lock lock(mtx_);

    fabric_simulator::SegmentStateArray states_msg;

    // Set the fabric_id 
    states_msg.fabric_id = 0;  // TODO: Replace in future with fabric_id obtaining method

    const Eigen::Matrix<Real,Eigen::Dynamic,3> &pos_ptr = *fabric_.getPosPtr();
    const Eigen::Matrix<Real,Eigen::Dynamic,3> &vel_ptr = *fabric_.getVelPtr();
    const Eigen::Matrix<Real,Eigen::Dynamic,3> &for_ptr = *fabric_.getForPtr();

    for (size_t i = 0; i < pos_ptr.rows(); i++) {
        fabric_simulator::SegmentState segment_state;

        // Set the segment id
        segment_state.id = i;

        // Set the header
        segment_state.header.stamp = ros::Time::now();
        segment_state.header.frame_id = fabric_frame_id_;

        // Set pose (position and orientation)
        // Set pose position
        segment_state.pose.position.x = pos_ptr(i, 0);
        segment_state.pose.position.y = pos_ptr(i, 1);
        segment_state.pose.position.z = pos_ptr(i, 2);

        // Set pose orientation (NULL)
        segment_state.pose.orientation.x = 0.0;
        segment_state.pose.orientation.y = 0.0;
        segment_state.pose.orientation.z = 0.0;
        segment_state.pose.orientation.w = 1.0;

        // Set twist (linear and angular velocity)
        // Set linear velocity
        segment_state.twist.linear.x = vel_ptr(i, 0);
        segment_state.twist.linear.y = vel_ptr(i, 1);
        segment_state.twist.linear.z = vel_ptr(i, 2);

        // Set angular velocity (NULL)
        segment_state.twist.angular.x = 0.0;
        segment_state.twist.angular.y = 0.0;
        segment_state.twist.angular.z = 0.0;

        // Set wrench (Force and Torque) 
        // Set force
        segment_state.wrench.force.x = for_ptr(i, 0);
        segment_state.wrench.force.y = for_ptr(i, 1);
        segment_state.wrench.force.z = for_ptr(i, 2);

        // Set torque (NULL)
        segment_state.wrench.torque.x = 0.0;
        segment_state.wrench.torque.y = 0.0;
        segment_state.wrench.torque.z = 0.0;

        // Add the segment state to the message
        states_msg.states.push_back(segment_state);
    }

    // Publish the message
    pub_fabric_state_.publish(states_msg);
}