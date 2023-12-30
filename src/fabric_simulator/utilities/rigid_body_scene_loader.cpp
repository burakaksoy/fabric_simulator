#include "fabric_simulator/utilities/rigid_body_scene_loader.h"
#include <iostream>
#include <fstream>
#include <ros/ros.h> // needed for log err info

using namespace utilities;

void RigidBodySceneLoader::readScene(const std::string &fileName, std::vector<RigidBodyData> &m_rigidBodyData)
{
	try
	{
		std::ifstream input_file(fileName);
		if (!input_file.is_open())
		{
			throw std::runtime_error("Could not open file " + fileName);
			return;
		}
		m_json << input_file;

		std::string basePath = fileName;

		//////////////////////////////////////////////////////////////////////////
		// read rigid bodies
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("RigidBodies") != m_json.end())
			readRigidBodies(m_json, "RigidBodies", basePath, m_rigidBodyData);

	}
	catch (std::exception& e)
	{
		ROS_ERROR("%s", e.what()); 
		exit(1);
	}
}


void RigidBodySceneLoader::readRigidBodies(const nlohmann::json &j, const std::string &key, const std::string &basePath, std::vector<RigidBodyData> &m_rigidBodyData)
{
	const nlohmann::json &child = j[key];

	// m_rigidBodyData.reserve(5000);

	for (auto& rigidBody : child)
	{
		std::string geomFileName;
		if (readValue<std::string>(rigidBody, "geometryFile", geomFileName))
		{

			m_rigidBodyData.emplace_back(RigidBodyData());
			RigidBodyData &rbd = m_rigidBodyData.back();
			rbd.m_modelFile = geomFileName;

			// id
			rbd.m_id = 0;
			readValue(rigidBody, "id", rbd.m_id);

			// is dynamic body
			rbd.m_isDynamic = true;
			readValue(rigidBody, "isDynamic", rbd.m_isDynamic);


			// density
			rbd.m_density = 1.0;
			readValue(rigidBody, "density", rbd.m_density);

			// translation
			rbd.m_x.setZero();
			readVector(rigidBody, "translation", rbd.m_x);
			
			// rotation axis
			Eigen::Matrix<Real, 1, 3> axis;
			axis.setZero();
			Real angle = 0.0;
			if (readVector(rigidBody, "rotationAxis", axis) &&
				readValue<Real>(rigidBody, "rotationAngle", angle))
			{
				axis.normalize();
				rbd.m_q = Eigen::Quaternion<Real>(Eigen::AngleAxis<Real>(angle, axis)).normalized();
			}
			else
				rbd.m_q = Eigen::Quaternion<Real>(1.0, 0.0, 0.0, 0.0).normalized();
				
			// scale
			rbd.m_scale = Eigen::Matrix<Real, 1, 3>(1.0, 1.0, 1.0);
			readVector(rigidBody, "scale", rbd.m_scale);

			// velocity
			rbd.m_v.setZero();
			readVector(rigidBody, "velocity", rbd.m_v);

			// angular velocity
			rbd.m_omega.setZero();
			readVector(rigidBody, "angularVelocity", rbd.m_omega);

			// restitution
			rbd.m_restitutionCoeff = 0.6;
			readValue(rigidBody, "restitution", rbd.m_restitutionCoeff);

			// friction
			rbd.m_frictionCoeffStatic = 0.2;
			rbd.m_frictionCoeffDynamic = 0.2;
			readValue(rigidBody, "frictionStatic", rbd.m_frictionCoeffStatic);
			readValue(rigidBody, "frictionDynamic", rbd.m_frictionCoeffDynamic);


			rbd.m_collisionObjectFileName = "";
			readValue(rigidBody, "collisionObjectFileName", rbd.m_collisionObjectFileName);


			// Collision object scale
			rbd.m_collisionObjectScale = Eigen::Matrix<Real, 1,3>(1.0, 1.0, 1.0);
			readVector(rigidBody, "collisionObjectScale", rbd.m_collisionObjectScale);

			rbd.m_resolutionSDF = Eigen::Matrix<unsigned int, 1,3>(10, 10, 10);
			readVector(rigidBody, "resolutionSDF", rbd.m_resolutionSDF);

			rbd.m_invertSDF = false;
			readValue(rigidBody, "invertSDF", rbd.m_invertSDF);
			
			rbd.m_json = rigidBody;
		}
	}
}

template <>
bool RigidBodySceneLoader::readValue<bool>(const nlohmann::json &j, const std::string &key, bool &v)
{
	if (j.find(key) == j.end())
		return false;

	if (j[key].is_number_integer())
	{
		int val = j[key].get<int>();
		v = val != 0;
	}
	else
		v = j[key].get<bool>();
	return true;
}



