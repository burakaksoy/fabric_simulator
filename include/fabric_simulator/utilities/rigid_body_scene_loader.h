#ifndef __RIGID_BODY_SCENE_LOADER_H__
#define __RIGID_BODY_SCENE_LOADER_H__

#include <string>
#include "extern/json/json.hpp"

#include "fabric_simulator/utilities/cloth.h"
#include "Discregrid/All"

namespace utilities
{
	class RigidBodySceneLoader
	{
	protected:
		nlohmann::json m_json;

	public:
		using Grid = Discregrid::CubicLagrangeDiscreteGrid;
		using GridPtr = std::shared_ptr<Discregrid::CubicLagrangeDiscreteGrid>;

		struct RigidBodyData
		{			
			unsigned int m_id;
			std::string m_modelFile;

			bool m_isDynamic;

			Real m_density;

			Eigen::Matrix<Real, 1, 3> m_x; // translation
			Eigen::Quaternion<Real> m_q; // rotation quaternion
			Eigen::Matrix<Real, 1, 3> m_scale;	

			Eigen::Matrix<Real, 1, 3> m_v; // linear velocity
			Eigen::Matrix<Real, 1, 3> m_omega; //angular velocity
			
			Real m_restitutionCoeff;
			Real m_frictionCoeffStatic;
			Real m_frictionCoeffDynamic;
			
			std::string m_collisionObjectFileName;
			Eigen::Matrix<Real, 1, 3> m_collisionObjectScale;
			
			Eigen::Matrix<unsigned int, 1, 3> m_resolutionSDF;
			
			bool m_invertSDF;

			pbd_object::Mesh m_mesh;

			GridPtr m_discregrid_ptr;

			nlohmann::json m_json;

			bool m_isVisible;
		};

		virtual ~RigidBodySceneLoader() {}

		void readScene(const std::string &fileName, std::vector<RigidBodyData> &m_rigidBodyData);

		void readRigidBodies(const nlohmann::json &child, const std::string &key, const std::string &basePath, std::vector<RigidBodyData> &m_rigidBodyData);
		
		template <typename T>
		static bool readValue(const nlohmann::json &j, const std::string &key, T &v)
		{
			if (j.find(key) == j.end())
				return false;

			v = j[key].get<T>();
			return true;
		}

		template <typename T, int size>
		static bool readVector(const nlohmann::json &j, const std::string &key, Eigen::Matrix<T, 1, size> &vec)
		{
			if (j.find(key) == j.end())
				return false;

			std::vector<T> values = j[key].get<std::vector<T>>();
			for (unsigned int i = 0; i < values.size(); i++)
				vec[i] = values[i];
			return true;
		}

	};

	template <>
	bool RigidBodySceneLoader::readValue<bool>(const nlohmann::json &j, const std::string &key, bool &v);	

}

#endif
