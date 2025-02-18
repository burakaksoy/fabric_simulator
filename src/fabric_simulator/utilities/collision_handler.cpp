/*
 * Author: Burak Aksoy
 * handles the collisions, ie. contacts between cloth and rigid bodies to prevent penetration into each other
 */

#include "fabric_simulator/utilities/collision_handler.h"
// #include <ros/ros.h> // Added for debug timing

using namespace utilities;

const unsigned int CollisionHandler::RigidBodyContactType = 0;
const unsigned int CollisionHandler::ParticleContactType = 1;
const unsigned int CollisionHandler::ParticleRigidBodyContactType = 2;

const unsigned int CollisionHandler::CollisionObject::RigidBodyCollisionObjectType = 0;
const unsigned int CollisionHandler::CollisionObject::TriangleModelCollisionObjectType = 1;

// CollisionHandler::CollisionHandler(): 
// {
// 	commonInit(); // common initialization
// }

// Constructor with references to fabric and rigid bodies
CollisionHandler::CollisionHandler(pbd_object::Cloth &fabric, 
								   std::vector<RigidBodySceneLoader::RigidBodyData> &rigid_bodies): 
	fabric_(fabric), 
	rigid_bodies_(rigid_bodies) 
{
    commonInit(); // common initialization 
}

CollisionHandler::~CollisionHandler(){
	// Delete all the collision object handled by this class
    cleanupCollisionObjects();
}

void CollisionHandler::updateAABB(pbd_object::Cloth &fabric_,
                                  const std::vector<RigidBodySceneLoader::RigidBodyData> &rigid_bodies_, 
                                  CollisionHandler::CollisionObject *co){
    if (co->m_bodyType == CollisionHandler::CollisionObject::RigidBodyCollisionObjectType)
    {
        const RigidBodySceneLoader::RigidBodyData &rbd = rigid_bodies_[co->m_bodyIndex];
        const pbd_object::Mesh &mesh = rbd.m_mesh;
        const Eigen::Matrix<Real,Eigen::Dynamic,3> &vertices = mesh.vertices;

        co->m_aabb.m_p[0] = vertices.row(0).transpose();
        co->m_aabb.m_p[1] = vertices.row(0).transpose();

        for (int j = 1; j < vertices.rows(); j++)
        {
            updateAABB(vertices.row(j).transpose(), co->m_aabb);
        }
    }
	else if (co->m_bodyType == CollisionHandler::CollisionObject::TriangleModelCollisionObjectType)
	{
		const Eigen::Matrix<Real,Eigen::Dynamic,3> *pos_ptr = fabric_.getPosPtr();

		co->m_aabb.m_p[0] = pos_ptr->row(0).transpose();
		co->m_aabb.m_p[1] = pos_ptr->row(0).transpose();
		
        for (int j = 1; j < pos_ptr->rows(); j++)
		{
			updateAABB(pos_ptr->row(j).transpose(), co->m_aabb);
		}
	}

	// Extend AABB by tolerance in all xyz directions
	co->m_aabb.m_p[0][0] -= m_tolerance;
	co->m_aabb.m_p[0][1] -= m_tolerance;
	co->m_aabb.m_p[0][2] -= m_tolerance;
	co->m_aabb.m_p[1][0] += m_tolerance;
	co->m_aabb.m_p[1][1] += m_tolerance;
	co->m_aabb.m_p[1][2] += m_tolerance;
}


void CollisionHandler::collisionDetection(){
	// ros::Time start_time = ros::Time::now();

	// deletes all the elements inside the vectors that contains the constraint data.
	resetContacts();

	// generate pairs of collision objects that need to be checked for potential collisions.
	// coPairs vector is used to store indices of collision objects that will be compared for possible collisions.
	std::vector < std::pair<unsigned int, unsigned int>> coPairs;
	for (unsigned int i = 0; i < m_collisionObjects.size(); i++)
	{
		CollisionHandler::CollisionObject *co1 = m_collisionObjects[i];
		for (unsigned int k = 0; k < m_collisionObjects.size(); k++)
		{
			CollisionHandler::CollisionObject *co2 = m_collisionObjects[k];
			if ((i != k))
			{
				// ToDo: self collisions for deformables
				coPairs.push_back({ i, k });
			}
		}
	}

	// Structure to store contact data for collisions detected between objects. 
	std::vector<std::vector<ContactData> > contacts_mt;	

	// retrieve max num of threads that OpenMP can utilize for parallel processing.
	const unsigned int maxThreads = omp_get_max_threads();
	
	// resizes the contacts_mt vector so that its size matches num of available threads. 
	// This ensures that each thread can store its contact data independently.
	contacts_mt.resize(maxThreads);

	// directive to starts a parallel region. 
	// 'default(shared)': variables are shared by default among the threads.
	#pragma omp parallel default(shared)
	{
		// Update Bounding Volume Hierarchies (BVHs)
		// We have Bounding Sphere Hierachy (BSH) for fabric
		// Axis Aligned Bounding Box for all (i.e., fabric and rigid bodies)
		
		// distribute the iterations of the following loop across the available threads 
		// in a static manner (each thread gets a fixed number of iterations).
		#pragma omp for schedule(static) 
		// Update each of the collision objects the AABB's and the fabric's m_bvh
		for (int i = 0; i < (int)m_collisionObjects.size(); i++)
		{
			CollisionHandler::CollisionObject *co = m_collisionObjects[i];
			updateAABB(fabric_, rigid_bodies_, co);

			/* We only update the fabric's m_bvh but not other (rigid) objects 
				because the SDF distance readings are transformed to the local frames of them*/
			if (co->m_bodyType == CollisionHandler::CollisionObject::TriangleModelCollisionObjectType){
				co->m_bvh.update();}
		}

		// Iterate through all potential contact pairs to check whether they are actually penetrating.
		// After this loop the contacts_mt is filled with ContactData 
		#pragma omp for schedule(static)
		for (int i = 0; i < (int)coPairs.size(); i++)
		{
			std::pair<unsigned int, unsigned int> &coPair = coPairs[i];
			CollisionHandler::CollisionObject *co1 = m_collisionObjects[coPair.first];
			CollisionHandler::CollisionObject *co2 = m_collisionObjects[coPair.second];

			// Enforce that co1 can be either fabric or rigid body, but co2 can only be a rigid body.
			if (co2->m_bodyType != CollisionHandler::CollisionObject::RigidBodyCollisionObjectType ||
				!AABB::intersection(co1->m_aabb, co2->m_aabb))
				continue; // Even their AABB's are not instersecting yet, no collision is possible.

			// Collision detection between 2 rigid bodies
			if ((co1->m_bodyType == CollisionHandler::CollisionObject::RigidBodyCollisionObjectType) &&
				(co2->m_bodyType == CollisionHandler::CollisionObject::RigidBodyCollisionObjectType) &&
				(co1->m_testMesh))
			{
				RigidBodySceneLoader::RigidBodyData &rb1 = rigid_bodies_[co1->m_bodyIndex];
				RigidBodySceneLoader::RigidBodyData &rb2 = rigid_bodies_[co2->m_bodyIndex];

				// Set (Calculate) the CONTACT's restitution & friction coefficients
				const Real &restitutionCoeff = rb1.m_restitutionCoeff * rb2.m_restitutionCoeff;
				const Real &frictionCoeffStatic = rb1.m_frictionCoeffStatic + rb2.m_frictionCoeffStatic;
				const Real &frictionCoeffDynamic = rb1.m_frictionCoeffDynamic + rb2.m_frictionCoeffDynamic;
				
				// co1 and co2's AABB are intersecting, check if they are actually in contact
				// with the help of BVH KD Tree DepthFirst search 
				// and co2's distance and collisionTest functions.
				// If they indeed in collision, add the related ContactData to contacts_mt.
				collisionDetectionRigidBodies(rb1, 
											  co1, 
											  rb2, 
											  co2,
											  restitutionCoeff, 
											  frictionCoeffStatic,
											  frictionCoeffDynamic,
											  contacts_mt);
			}
			// Collision detection between a fabric and a rigid body
			else if ((co1->m_bodyType == CollisionHandler::CollisionObject::TriangleModelCollisionObjectType) &&
					 (co2->m_bodyType == CollisionHandler::CollisionObject::RigidBodyCollisionObjectType) &&
					 (co1->m_testMesh))
			{
				RigidBodySceneLoader::RigidBodyData &rb2 = rigid_bodies_[co2->m_bodyIndex];

				// Set (Calculate) the CONTACT's restitution & friction coefficients
				// Assume the fabric's restitution coeff = 1 and friction coeff = 0.0
				const Real &restitutionCoeff = rb2.m_restitutionCoeff;
				const Real &frictionCoeffStatic = rb2.m_frictionCoeffStatic;
				const Real &frictionCoeffDynamic = rb2.m_frictionCoeffDynamic;
				
				// co1 and co2's AABB are intersecting, check if they are actually in contact
				// with the help of BVH KD Tree DepthFirst search 
				// and co2's distance and collisionTest functions.
				// If they indeed in collision, add the related ContactData to contacts_mt.
				collisionDetectionRBSolid(fabric_,
										  co1,
										  rb2,
										  co2,
										  restitutionCoeff,
										  frictionCoeffStatic,
										  frictionCoeffDynamic,
										  contacts_mt);
			}
		}

	}

	// Iterate through all the detected contacts and add them to the correspong Contact Types 
	// by binding their contact callback function
	for (unsigned int i = 0; i < contacts_mt.size(); i++)
	{
		for (unsigned int j = 0; j < contacts_mt[i].size(); j++)
		{
			if (contacts_mt[i][j].m_type == CollisionHandler::ParticleRigidBodyContactType)
			{
				addParticleRigidBodyContact(contacts_mt[i][j].m_index1, 
											contacts_mt[i][j].m_index2,
											contacts_mt[i][j].m_cp1,
											contacts_mt[i][j].m_cp2, 
											contacts_mt[i][j].m_normal,
											contacts_mt[i][j].m_dist,
											contacts_mt[i][j].m_restitution,
											contacts_mt[i][j].m_frictionCoeffStatic,
											contacts_mt[i][j].m_frictionCoeffDynamic);
			}
			else if (contacts_mt[i][j].m_type == CollisionHandler::RigidBodyContactType)
			{
				addRigidBodyContact(contacts_mt[i][j].m_index1, 
									contacts_mt[i][j].m_index2,
									contacts_mt[i][j].m_cp1, 
									contacts_mt[i][j].m_cp2, 
									contacts_mt[i][j].m_normal,
									contacts_mt[i][j].m_dist,
									contacts_mt[i][j].m_restitution,
									contacts_mt[i][j].m_frictionCoeffStatic,
									contacts_mt[i][j].m_frictionCoeffDynamic);
			}
		}
	}
	
	// ros::Time finish_time = ros::Time::now();
	// ros::Duration elapsed_time = finish_time - start_time;
	// ROS_INFO("[Collision Handler]: %-4.2lf ms per Collision Detection iteration", elapsed_time.toSec()*1000);
}

void CollisionHandler::collisionDetectionRigidBodies(RigidBodySceneLoader::RigidBodyData &rb1, 
                                    CollisionObject *co1, 
                                    RigidBodySceneLoader::RigidBodyData &rb2,
                                    CollisionObject *co2,
                                    const Real &restitutionCoeff, 
                                    const Real &frictionCoeffStatic, 
                                    const Real &frictionCoeffDynamic, 
                                    std::vector<std::vector<ContactData> > &contacts_mt)
{
	// TODO: THIS FUNCTION IS NOT ACTIVE SINCE THE RIGID BODIES ARE ASSUMED TO BE NON-DYNAMIC
	if (!(rb1.m_isDynamic) && !(rb2.m_isDynamic))
		return;

	// if ((rb1->getMass() == 0.0) && (rb2->getMass() == 0.0))
	// 	return;

	// const VertexData &vd = rb1->getGeometry().getVertexData();

	// const Eigen::Matrix<Real, 3, 1> &com2 = rb2->getPosition();

	// // remove the rotation of the main axis transformation that is performed
	// // to get a diagonal inertia tensor since the distance function is 
	// // evaluated in local coordinates
	// //
	// // transformation world to local:
	// // p_local = R_initial^T ( R_MAT R^T (p_world - x) - x_initial + x_MAT)
	// // 
	// // transformation local to:
	// // p_world = R R_MAT^T (R_initial p_local + x_initial - x_MAT) + x
	// //
	// const Eigen::Matrix<Real, 3, 3> &R = rb2->getTransformationR();
	// const Eigen::Matrix<Real, 3, 1> &v1 = rb2->getTransformationV1();
	// const Eigen::Matrix<Real, 3, 1> &v2 = rb2->getTransformationV2();

	// const PointCloudBSH &bvh = ((DistanceFieldCollisionDetection::DistanceFieldCollisionObject*) co1)->m_bvh;
	// std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth)
	// {
	// 	const BoundingSphere &bs = bvh.hull(node_index);
	// 	const Eigen::Matrix<Real, 3, 1> &sphere_x = bs.x();
	// 	const Eigen::Matrix<Real, 3, 1> sphere_x_w = rb1->getRotation() * sphere_x + rb1->getPosition();

	// 	Eigen::AlignedBox<Real, 3> box3f;
	// 	box3f.extend(co2->m_aabb.m_p[0]);
	// 	box3f.extend(co2->m_aabb.m_p[1]);
	// 	const Real dist = box3f.exteriorDistance(sphere_x_w);

	// 	// Test if center of bounding sphere intersects AABB
	// 	if (dist < bs.r())
	// 	{
	// 		// Test if distance of center of bounding sphere to collision object is smaller than the radius
	// 		const Eigen::Matrix<Real, 3, 1> x = R * (sphere_x_w - com2) + v1;
	// 		const double dist2 = co2->distance(x.template cast<double>(), m_tolerance);
	// 		if (dist2 == std::numeric_limits<double>::max())
	// 			return true;
	// 		if (dist2 < bs.r())
	// 			return true;
	// 	}
	// 	return false;
	// };
	// std::function<void(unsigned int, unsigned int)> cb = [&](unsigned int node_index, unsigned int depth)
	// {
	// 	auto const& node = bvh.node(node_index);
	// 	if (!node.is_leaf())
	// 		return;

	// 	for (auto i = node.begin; i < node.begin + node.n; ++i)
	// 	{
	// 		unsigned int index = bvh.entity(i);
	// 		const Eigen::Matrix<Real, 3, 1> &x_w = vd.getPosition(index);
	// 		const Eigen::Matrix<Real, 3, 1> x = R * (x_w - com2) + v1;
	// 		Eigen::Matrix<Real, 3, 1> cp, n;
	// 		Real dist;
	// 		if (co2->collisionTest(x, m_tolerance, cp, n, dist))
	// 		{
	// 			const Eigen::Matrix<Real, 3, 1> cp_w = R.transpose() * cp + v2;
	// 			const Eigen::Matrix<Real, 3, 1> n_w = R.transpose() * n;

	// 			// int tid = 0; debug
	// 			int tid = omp_get_thread_num();

	// 			contacts_mt[tid].push_back({ CollisionHandler::RigidBodyContactType, 
	// 										 co1->m_bodyIndex, 
	// 										 co2->m_bodyIndex, 
	// 										 x_w, 
	// 										 cp_w, 
	// 										 n_w, 
	// 										 dist, 
	// 										 restitutionCoeff, 
	// 										 frictionCoeffStatic,
	// 										 frictionCoeffDynamic });
	// 		}
	// 	}
	// };
	// bvh.traverse_depth_first(predicate, cb);

}

void CollisionHandler::collisionDetectionRBSolid(pbd_object::Cloth &fabric_,
								CollisionObject *co1, 
								RigidBodySceneLoader::RigidBodyData &rb2, 
								CollisionObject *co2, 
								const Real &restitutionCoeff, 
								const Real &frictionCoeffStatic, 
								const Real &frictionCoeffDynamic, 
								std::vector< std::vector<ContactData> > &contacts_mt)
{

	const Eigen::Matrix<Real,Eigen::Dynamic,3> *pos_ptr = fabric_.getPosPtr();

	// const Eigen::Matrix<Real, 3, 1> &com2 = rb2->getPosition();
	const Eigen::Matrix<Real, 3, 1> &com2 = rb2.m_x.transpose(); // Since the RB is non-dynamic, its COM is at the translation location

	// remove the rotation of the main axis transformation that is performed
	// to get a diagonal inertia tensor since the distance function is 
	// evaluated in local coordinates
	//
	// transformation world to local:
	// p_local = R_initial^T ( R_MAT R^T (p_world - x) - x_initial + x_MAT)
	// 
	// transformation local to:
	// p_world = R R_MAT^T (R_initial p_local + x_initial - x_MAT) + x
	//

	// Rotation Matrix from world to rigid body
	const Eigen::Matrix<Real, 3, 3> &R = rb2.m_q.normalized().toRotationMatrix();
	// const Eigen::Matrix<Real, 3, 1> &v1 = rb2->getTransformationV1();
	// const Eigen::Matrix<Real, 3, 1> &v2 = rb2->getTransformationV2();

	const PointCloudBSH &bvh = co1->m_bvh;

	std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth)
	{
		const BoundingSphere &bs = bvh.hull(node_index);
		const Eigen::Matrix<Real, 3, 1> &sphere_x_w = bs.x();

		Eigen::AlignedBox<Real, 3> box3f;
		box3f.extend(co2->m_aabb.m_p[0]);
		box3f.extend(co2->m_aabb.m_p[1]);
		const Real dist = box3f.exteriorDistance(sphere_x_w);

		// Test if center of bounding sphere of fabric intersects with rigid body's AABB
		if (dist < bs.r())
		{
			// Test if distance of center of bounding sphere to collision object is smaller than the radius

			// Get the vector from COM to sphere center in RB COM local frame.
			// This is needed because the sdf is set wrt to the local frame
			// const Eigen::Matrix<Real, 3, 1> x = R * (sphere_x_w - com2) + v1;
			const Eigen::Matrix<Real, 3, 1> x = R.transpose() * (sphere_x_w - com2);
			
			// Use SDF to find the distance between the COM and the sphere center
			const double dist2 = co2->distance(x.template cast<double>(), m_tolerance);
			if (dist2 == std::numeric_limits<double>::max())
				return true;
			if (dist2 < bs.r())
				return true;
		}
		return false;
	};

	std::function<void(unsigned int, unsigned int)> cb = [&](unsigned int node_index, unsigned int depth)
	{
		auto const& node = bvh.node(node_index);
		if (!node.is_leaf())
			return;

		for (auto i = node.begin; i < node.begin + node.n; ++i)
		{
			unsigned int index = bvh.entity(i);
			// const Eigen::Matrix<Real, 3, 1> &x_w = pd.getPosition(index);
			const Eigen::Matrix<Real, 3, 1> &x_w = pos_ptr->row(index).transpose();
			
			// Local vector in RB frame from COM to particle position
			// const Eigen::Matrix<Real, 3, 1> x = R * (x_w - com2) + v1;
			const Eigen::Matrix<Real, 3, 1> x = R.transpose() * (x_w - com2);

			Eigen::Matrix<Real, 3, 1> cp, n;
			Real dist;
			if (co2->collisionTest(x, m_tolerance, cp, n, dist))
			{
				// Convert back the local coordinates of contact point (cp) and normal vectors to world frame
				// const Eigen::Matrix<Real, 3, 1> cp_w = R.transpose() * cp + v2;
				// const Eigen::Matrix<Real, 3, 1> n_w = R.transpose() * n;
				const Eigen::Matrix<Real, 3, 1> cp_w = R * cp + com2;
				const Eigen::Matrix<Real, 3, 1> n_w = R * n;

				// int tid = 0; debug
				int tid = omp_get_thread_num();		
				contacts_mt[tid].push_back({ CollisionHandler::ParticleRigidBodyContactType, 
											 index, 
											 co2->m_bodyIndex, 
											 x_w, 
											 cp_w, 
											 n_w, 
											 dist, 
											 restitutionCoeff, 
											 frictionCoeffStatic,
											 frictionCoeffDynamic });
			}
		}
	};

	bvh.traverse_depth_first(predicate, cb);
}								

void CollisionHandler::computeMinDistancesToRigidBodies(std::vector<std::vector<MinDistanceData>> &min_distances_mt) 
{
	// This function finds the minimum distances from fabric_ to the rigid bodies.
	// The data is stored in MinDistanceData vector min_distances.

	// Retrieve the maximum number of threads that OpenMP can utilize.
	#ifdef _DEBUG
		const unsigned int maxThreads = 1;
	#else
		const unsigned int maxThreads = omp_get_max_threads();
	#endif

	// Resize the container to hold results for each thread.
	min_distances_mt.resize(maxThreads);

	// Collect only rigid bodies and a single fabric object.
	std::vector<CollisionObject*> rigidBodies;
	CollisionObject* fabric = nullptr;

	for (auto& co : m_collisionObjects) 
	{
		if (co->m_bodyType == CollisionHandler::CollisionObject::RigidBodyCollisionObjectType) 
		{
			rigidBodies.push_back(co);
		}
		else if(co->m_bodyType == CollisionHandler::CollisionObject::TriangleModelCollisionObjectType 
				&& !fabric)
		{
			fabric = co;
			/* We only update the fabric's m_bvh but not other (rigid) objects 
				because the SDF distance readings are transformed to the local frames of them*/
			co->m_bvh.update();
		}
	}

	if (!fabric) return; // No fabric found

	#pragma omp parallel default(shared)
	{
		int tid = omp_get_thread_num();

		#pragma omp for schedule(static) 
		for (int i = 0; i < rigidBodies.size(); i++) 
		{
			CollisionObject *rigidBody = rigidBodies[i];
			RigidBodySceneLoader::RigidBodyData &rbData = rigid_bodies_[rigidBody->m_bodyIndex];

			MinDistanceData minDistanceData;
			computeMinDistanceRBSolid(fabric_, fabric, rbData, rigidBody, minDistanceData);

			// Push back the minimum distance data only when necessary
			if (minDistanceData.m_minDistance < std::numeric_limits<Real>::max())
			{
				min_distances_mt[tid].push_back(minDistanceData);
			}
		}
	}
}

void CollisionHandler::computeMinDistanceRBSolid(pbd_object::Cloth &fabric_,
                                                 CollisionObject *co1, 
                                                 RigidBodySceneLoader::RigidBodyData &rb2, 
                                                 CollisionObject *co2,
                                                 MinDistanceData& minDistanceData)
{
	//find the minimum distance between a fabric object and a given rigid body 
	
    const Eigen::Matrix<Real,Eigen::Dynamic,3> *pos_ptr = fabric_.getPosPtr();
    const Eigen::Matrix<Real, 3, 1> &com2 = rb2.m_x.transpose();
    const Eigen::Matrix<Real, 3, 3> &R = rb2.m_q.normalized().toRotationMatrix();

    const PointCloudBSH& bvh = co1->m_bvh;

    minDistanceData.m_minDistance = std::numeric_limits<Real>::max();

    // Predicate Function
    auto predicate = [&](unsigned int, unsigned int) -> bool {
        return true; // Always traverse all nodes
    };

    // Callback Function
    auto callback = [&](unsigned int node_index, unsigned int) {
        auto const& node = bvh.node(node_index);
        if (!node.is_leaf()) return;

        for (auto i = node.begin; i < node.begin + node.n; ++i) {
            unsigned int index = bvh.entity(i);
            const Eigen::Matrix<Real, 3, 1>& x_w = pos_ptr->row(index).transpose(); // point on cloth
            const Eigen::Matrix<Real, 3, 1> x = R.transpose() * (x_w - com2); // represented on rb

			const Eigen::Matrix<Real, 3, 1> scaled_x = x.cwiseProduct(co2->m_scale.cwiseInverse());			

            Eigen::Vector3d normal;	
            double d_scaled = co2->m_sdf->interpolate(0, scaled_x.template cast<double>(), &normal);

			if (static_cast<Real>(d_scaled) == std::numeric_limits<Real>::max())
				continue;

			normal = co2->m_invertSDF * normal;
			if (normal.squaredNorm() > 1.0e-6)
				normal.normalize();

			Eigen::Matrix<Real, 3, 1> cp_scaled = (scaled_x - (d_scaled * normal).template cast<Real>());
			Eigen::Matrix<Real, 3, 1> cp = cp_scaled.cwiseProduct(co2->m_scale); // cp at actual object surface

			// Real dist = ((x-cp).dot(normal.template cast<Real>()) - m_tolerance); // dist from the tolerance surface
			Real dist = (x-cp).dot(normal.template cast<Real>()); // dist from the tolerance surface

			// cp = x - dist * normal.template cast<Real>(); // update cp to tolerance surface

            if (dist < minDistanceData.m_minDistance) {
				minDistanceData.m_type   = CollisionHandler::ParticleRigidBodyContactType;
				minDistanceData.m_index1 = index;
				minDistanceData.m_index2 = co2->m_bodyIndex;
				minDistanceData.m_pointOnObject1 = x_w;
                minDistanceData.m_pointOnObject2 = R * cp + com2; // closest point in world frame
                minDistanceData.m_normal = R * normal.template cast<Real>(); // normal in world frame
				minDistanceData.m_minDistance = dist;
            }
        }
    };

    // Perform the depth-first traversal
    bvh.traverse_depth_first(predicate, callback);
}

/////////////////////////////////////////////////////////////////////////
////////// IMPLEMENTATIONS OF THE COLLISIONOBJECT STRUCT FUNCTIONS //////
/////////////////////////////////////////////////////////////////////////


double CollisionHandler::CollisionObject::distance(const Eigen::Vector3d &x, 
												   const Real tolerance)
{
	// note that this function is applied in local coordinates of the collision sdf object
	const Eigen::Vector3d scaled_x = x.cwiseProduct(m_scale.template cast<double>().cwiseInverse());

	Eigen::Vector3d normal;	
	double d_scaled = m_sdf->interpolate(0, scaled_x, &normal);

	if (d_scaled == std::numeric_limits<double>::max())
		return d_scaled;

	normal = m_invertSDF * normal;
	if (normal.squaredNorm() > 1.0e-6)
		normal.normalize();

	Eigen::Vector3d cp_scaled = (scaled_x - (d_scaled * normal));
	Eigen::Vector3d cp = cp_scaled.cwiseProduct(m_scale);
	
	// return (x-cp).dot(normal) - static_cast<Real>(tolerance); // signed distance from the tolerance surface
	return (x-cp).dot(normal); // signed distance from the actual surface
}

bool CollisionHandler::CollisionObject::collisionTest(
												const Eigen::Matrix<Real, 3, 1> &x, 
												const Real tolerance,
												Eigen::Matrix<Real, 3, 1> &cp, 
												Eigen::Matrix<Real, 3, 1> &n, 
												Real &dist, 
												const Real maxDist /* = 0.0*/)
{
	// note that this function is applied in local coordinates of the collision sdf object
	const Eigen::Matrix<Real, 3, 1> scaled_x = x.cwiseProduct(m_scale.cwiseInverse());

	Eigen::Vector3d normal;	
	double d_scaled = m_sdf->interpolate(0, scaled_x.template cast<double>(), &normal);
	
	if (static_cast<Real>(d_scaled) == std::numeric_limits<Real>::max())
		return false;
	
	normal = m_invertSDF * normal;
	if (normal.squaredNorm() > 1.0e-6)
		normal.normalize();

	Eigen::Matrix<Real, 3, 1> cp_scaled = (scaled_x - (d_scaled * normal).template cast<Real>());
	Eigen::Matrix<Real, 3, 1> cp_temp = cp_scaled.cwiseProduct(m_scale);

	// dist = ((x-cp_temp).dot(normal.template cast<Real>()) - tolerance); // signed distance from the tolerance surface
	dist = ((x-cp_temp).dot(normal.template cast<Real>())); // signed distance from the actual surface
	
	if (dist < maxDist)
	{
		n = normal.template cast<Real>();
		// cp = (x - dist * n); // cp is on the tolerance surface
		cp = cp_temp; // cp is on the actual surface

		return true;
	}
	
	return false;
}

////////////////////////////////////////////////////////////////////////////////////
//////// IMPLEMENTATIONS OF THE RigidBodyContactConstraint STRUCT FUNCTIONS ////////
////////////////////////////////////////////////////////////////////////////////////
bool CollisionHandler::RigidBodyContactConstraint::initConstraint(
														pbd_object::Cloth &fabric_,
                                						std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
														const unsigned int &rbIndex1,
														const unsigned int &rbIndex2, 
														const Eigen::Matrix<Real, 3, 1> &cp1, 
														const Eigen::Matrix<Real, 3, 1> &cp2, 
														const Eigen::Matrix<Real, 3, 1> &normal, 
														const Real &dist, 
														const Real &restitutionCoeff, 
														const Real &frictionCoeffStatic,
														const Real &frictionCoeffDynamic)
{
	
	m_frictionCoeffStatic = frictionCoeffStatic;
	m_frictionCoeffDynamic = frictionCoeffDynamic;

	// indices of the linked (contacted) bodies
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;

    // std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_ = fabricSimulator.getRigidBodies();
	RigidBodySceneLoader::RigidBodyData &rb1 = rigid_bodies_[m_bodies[0]];
	RigidBodySceneLoader::RigidBodyData &rb2 = rigid_bodies_[m_bodies[1]];


	// TODO: When the rigid bodies are dynamic, this may need to be handled differently!!!!

	// return PositionBasedRigidBodyDynamics::init_RigidBodyContactConstraint(
	// rb1.getInvMass(),				 // const Real invMass1,								
	// rb1.getPosition(),				 // const Eigen::Matrix<Real, 3, 1> &x1,				// center of mass of body 1
	// rb1.getVelocity(),				 // const Eigen::Matrix<Real, 3, 1> &v1,				// velocity of body 1
	// rb1.getInertiaTensorInverseW(),	 // const Eigen::Matrix<Real, 3, 3> &inertiaInverseW1,	// inverse inertia tensor (world space) of body 1
	// rb1.getRotation(),				 // const Eigen::Quaternion<Real> &q1,					// rotation of body 1	
	// rb1.getAngularVelocity(),		 // const Eigen::Matrix<Real, 3, 1> &omega1,			// angular velocity of body 1
	
	// rb2.getInvMass(),				 // const Real invMass2,							    
	// rb2.getPosition(),				 // const Eigen::Matrix<Real, 3, 1> &x2,				// center of mass of body 2
	// rb2.getVelocity(),				 // const Eigen::Matrix<Real, 3, 1> &v2,				// velocity of body 2
	// rb2.getInertiaTensorInverseW(),	 // const Eigen::Matrix<Real, 3, 3> &inertiaInverseW2,	// inverse inertia tensor (world space) of body 2
	// rb2.getRotation(),				 // const Eigen::Quaternion<Real> &q2,					// rotation of body 2
	// rb2.getAngularVelocity(),		 // const Eigen::Matrix<Real, 3, 1> &omega2,			// angular velocity of body 2
	
	// cp1, 							 // const Eigen::Matrix<Real, 3, 1> &cp1,				// contact point of body 1
	// cp2, 							 // const Eigen::Matrix<Real, 3, 1> &cp2,				// contact point of body 2
	// normal, 						 	 // const Eigen::Matrix<Real, 3, 1> &normal,			// contact normal in body 2
	// restitutionCoeff, 				 // const Real restitutionCoeff,						// coefficient of restitution

	// m_constraintInfo contains
	// 0:	contact point in body 1 (global)
	// 1:	contact point in body 2 (global)
	// 2:	contact normal in body 2 (global)
	// 3:	contact tangent (global)
	// 0,4:  1.0 / normal^T * K * normal
	// 1,4: maximal impulse in tangent direction
	// 2,4: goal velocity in normal direction after collision

	// Set the needed parameters for STATIC (NON-DYNAMIC) rigid bodies
	const bool &isDynamic1 = rb1.m_isDynamic;
	const Real invMass1 = 0.0;
	const Eigen::Matrix<Real, 3, 1> &x1 = rb1.m_x.transpose();
	const Eigen::Matrix<Real, 3, 1> v1 = Eigen::Matrix<Real, 3, 1>::Zero();              // Set zero on initialization
	const Eigen::Matrix<Real, 3, 3> inertiaInverseW1 = Eigen::Matrix<Real, 3, 3>::Identity(); // Set to identity
	// const Eigen::Quaternion<Real> &q1 = rb1.m_q;
	const Eigen::Matrix<Real, 3, 1> omega1 = Eigen::Matrix<Real, 3, 1>::Zero();          // Set zero on initialization

	const bool &isDynamic2 = rb2.m_isDynamic;
	const Real invMass2 = 0.0;
	const Eigen::Matrix<Real, 3, 1> &x2 = rb2.m_x.transpose();
	const Eigen::Matrix<Real, 3, 1> v2 = Eigen::Matrix<Real, 3, 1>::Zero();              // Set zero on initialization
	const Eigen::Matrix<Real, 3, 3> inertiaInverseW2 = Eigen::Matrix<Real, 3, 3>::Identity(); // Set to identity
	// const Eigen::Quaternion<Real> &q2 = rb2.m_q;   
	const Eigen::Matrix<Real, 3, 1> omega2 = Eigen::Matrix<Real, 3, 1>::Zero();          // Set zero on initialization
	

	// compute goal velocity in normal direction after collision
	const Eigen::Matrix<Real, 3, 1> r1 = cp1 - x1;
	const Eigen::Matrix<Real, 3, 1> r2 = cp2 - x2;

	const Eigen::Matrix<Real, 3, 1> u1 = v1 + omega1.cross(r1);
	const Eigen::Matrix<Real, 3, 1> u2 = v2 + omega2.cross(r2);
	const Eigen::Matrix<Real, 3, 1> u_rel = u1 - u2;
	const Real u_rel_n = normal.dot(u_rel);

	m_constraintInfo.col(0) = cp1;
	m_constraintInfo.col(1) = cp2;
	m_constraintInfo.col(2) = normal;

	// tangent direction
	Eigen::Matrix<Real, 3, 1> t = u_rel - u_rel_n*normal;
	Real tl2 = t.squaredNorm();
	if (tl2 > 1.0e-6)
		t *= static_cast<Real>(1.0) / sqrt(tl2);

	m_constraintInfo.col(3) = t;

	// determine K matrix
	Eigen::Matrix<Real, 3, 3> K1, K2;
	if (isDynamic1){ CollisionHandler::computeMatrixK(cp1, invMass1, x1, inertiaInverseW1, K1);}
	else{K1.setZero();}
	if (isDynamic2){ CollisionHandler::computeMatrixK(cp2, invMass2, x2, inertiaInverseW2, K2);}
	else{K2.setZero();}
	Eigen::Matrix<Real, 3, 3> K = K1 + K2;

	m_constraintInfo(0, 4) = static_cast<Real>(1.0) / (normal.dot(K*normal));

	// maximal impulse in tangent direction
	m_constraintInfo(1, 4) = static_cast<Real>(1.0) / (t.dot(K*t)) * u_rel.dot(t);

	// goal velocity in normal direction after collision
	m_constraintInfo(2, 4) = 0.0;
	if (u_rel_n < 0.0)
		m_constraintInfo(2, 4) = -restitutionCoeff * u_rel_n;

	return true;
}

bool CollisionHandler::RigidBodyContactConstraint::solvePositionConstraint(
														pbd_object::Cloth &fabric_,
                                						std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
														const Real &dt)
{
	// Update the linear and angular velocities of the rigid bodies
	// TODO: Implement this function when the rigib bodies are dynamic
	return false; // because both assumed to have 0 mass
}

bool CollisionHandler::RigidBodyContactConstraint::solveVelocityConstraint(
														pbd_object::Cloth &fabric_,
                                						std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
														const Real &dt)
{
	// Update the linear and angular velocities of the rigid bodies
	// TODO: Implement this function when the rigib bodies are dynamic
	// Hint: Look at PositionBasedDynamics Github repo's Constraints.cpp:
	// bool RigidBodyContactConstraint::solveVelocityConstraint(SimulationModel &model, const unsigned int iter)
	// and PositionBasedRigidBodyDynamics.cpp's
	// bool PositionBasedRigidBodyDynamics::velocitySolve_RigidBodyContactConstraint(...) functions
	return false; // because both assumed to have 0 mass
}


////////////////////////////////////////////////////////////////////////////////////////
////// IMPLEMENTATIONS OF THE ParticleRigidBodyContactConstraint STRUCT FUNCTIONS //////
////////////////////////////////////////////////////////////////////////////////////////
bool CollisionHandler::ParticleRigidBodyContactConstraint::initConstraint(
														pbd_object::Cloth &fabric_,
                                						std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
														const unsigned int &particleIndex, 
														const unsigned int &rbIndex,
														const Eigen::Matrix<Real, 3, 1> &cp1,
														const Eigen::Matrix<Real, 3, 1> &cp2,
														const Eigen::Matrix<Real, 3, 1> &normal,
														const Real &dist,
														const Real &restitutionCoeff,
														const Real &frictionCoeffStatic,
														const Real &frictionCoeffDynamic)
{
	m_frictionCoeffStatic = frictionCoeffStatic;
	m_frictionCoeffDynamic = frictionCoeffDynamic;

	// indices of the linked (contacted) bodies
	m_bodies[0] = particleIndex;
	m_bodies[1] = rbIndex;

	// pbd_object::Cloth &fabric_ = fabricSimulator.getFabric();
	const Eigen::Matrix<Real,Eigen::Dynamic,3> *pos_ptr = fabric_.getPosPtr();
	const Eigen::Matrix<Real,Eigen::Dynamic,3> *vel_ptr = fabric_.getVelPtr();
	const Eigen::Matrix<Real,1,Eigen::Dynamic> *inv_mass_ptr = fabric_.getInvMassPtr();
	
	// std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_ = fabricSimulator.getRigidBodies();
	RigidBodySceneLoader::RigidBodyData &rb = rigid_bodies_[m_bodies[1]];

	// TODO: When the rigid bodies are dynamic, this may need to be handled differently!

	// return PositionBasedRigidBodyDynamics::init_ParticleRigidBodyContactConstraint(
	// pd.getInvMass(particleIndex),	// const Real invMass1,							 		
	// pd.getPosition(particleIndex),	// const Eigen::Matrix<Real, 3, 1> &x1,					// center of mass of body 1
	// pd.getVelocity(particleIndex),	// const Eigen::Matrix<Real, 3, 1> &v1,					// velocity of body 1
	
	// rb.getInvMass(),					// const Real invMass2,									
	// rb.getPosition(),				// const Eigen::Matrix<Real, 3, 1> &x2,					// center of mass of body 2
	// rb.getVelocity(),				// const Eigen::Matrix<Real, 3, 1> &v2,					// velocity of body 2
	// rb.getInertiaTensorInverseW(),	// const Eigen::Matrix<Real, 3, 3> &inertiaInverseW2,	// inverse inertia tensor (world space) of body 2
	// rb.getRotation(),				// const Eigen::Quaternion<Real> &q2,					// rotation of body 2	
	// rb.getAngularVelocity(),			// const Eigen::Matrix<Real, 3, 1> &omega2,				// angular velocity of body 2

	// cp1, 							// const Eigen::Matrix<Real, 3, 1> &cp1,				// contact point of body 1
	// cp2, 							// const Eigen::Matrix<Real, 3, 1> &cp2,				// contact point of body 2
	// normal, 							// const Eigen::Matrix<Real, 3, 1> &normal,				// contact normal in body 2
	// restitutionCoeff,				// const Real restitutionCoeff,							// coefficient of restitution
		
	// m_constraintInfo contains
	// 0:	contact point in body 1 (global)
	// 1:	contact point in body 2 (global)
	// 2:	contact normal in body 2 (global)
	// 3:	contact tangent (global)
	// 0,4:  1.0 / normal^T * K * normal
	// 1,4: maximal impulse in tangent direction
	// 2,4: goal velocity in normal direction after collision

	// Set the needed parameters for DYNAMIC FABRIC
	const bool &isDynamic1 = fabric_.isDynamicParticle(m_bodies[0]);
	const Real invMass1 = (*inv_mass_ptr)(m_bodies[0]); 
	const Eigen::Matrix<Real, 3, 1> &x1 = pos_ptr->row(m_bodies[0]).transpose();
	const Eigen::Matrix<Real, 3, 1> &v1 = vel_ptr->row(m_bodies[0]).transpose();

	// Set the needed parameters for STATIC (NON-DYNAMIC) the rigid body (note that the fabric is dynamic)
	const bool &isDynamic2 = rb.m_isDynamic;
	const Real invMass2 = 0.0;
	const Eigen::Matrix<Real, 3, 1> &x2 = rb.m_x.transpose();
	const Eigen::Matrix<Real, 3, 1> v2 = Eigen::Matrix<Real, 3, 1>::Zero();              // Set zero on initialization
	const Eigen::Matrix<Real, 3, 3> inertiaInverseW2 = Eigen::Matrix<Real, 3, 3>::Identity(); // Set to identity
	// const Eigen::Quaternion<Real> &q2 = rb.m_q;   
	const Eigen::Matrix<Real, 3, 1> omega2 = Eigen::Matrix<Real, 3, 1>::Zero();          // Set zero on initialization

	// compute goal velocity in normal direction after collision
	const Eigen::Matrix<Real, 3, 1> r2 = cp2 - x2;

	const Eigen::Matrix<Real, 3, 1> u2 = v2 + omega2.cross(r2);
	const Eigen::Matrix<Real, 3, 1> u_rel = v1 - u2;
	const Real u_rel_n = normal.dot(u_rel);

	m_constraintInfo.col(0) = cp1;
	m_constraintInfo.col(1) = cp2;
	m_constraintInfo.col(2) = normal;

	// tangent direction
	Eigen::Matrix<Real, 3, 1> t = u_rel - u_rel_n*normal;
	Real tl2 = t.squaredNorm();
	if (tl2 > 1.0e-6)
		t *= static_cast<Real>(1.0) / sqrt(tl2);

	m_constraintInfo.col(3) = t;

	// determine K matrix
	Eigen::Matrix<Real, 3, 3> K;
	if (isDynamic2){ CollisionHandler::computeMatrixK(cp2, invMass2, x2, inertiaInverseW2, K);}
	else {K.setZero();}

	if (isDynamic1)
	{
		K(0, 0) += invMass1;
		K(1, 1) += invMass1;
		K(2, 2) += invMass1;
	}

	m_constraintInfo(0, 4) = static_cast<Real>(1.0) / (normal.dot(K*normal));

	// maximal impulse in tangent direction
	m_constraintInfo(1, 4) = static_cast<Real>(1.0) / (t.dot(K*t)) * u_rel.dot(t);

	// goal velocity in normal direction after collision
	m_constraintInfo(2, 4) = 0.0;
	if (u_rel_n < 0.0){
		m_constraintInfo(2, 4) = -restitutionCoeff * u_rel_n;}

	return true;
}

bool CollisionHandler::ParticleRigidBodyContactConstraint::solvePositionConstraint(
														pbd_object::Cloth &fabric_,
                                						std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
														const Real &dt)
{
	m_lambda_n = 0.0;
	m_lambda_t = 0.0;

	Eigen::Matrix<Real,Eigen::Dynamic,3> *pos_ptr = fabric_.getPosPtr();
	const Eigen::Matrix<Real,Eigen::Dynamic,3> *prev_pos_ptr = fabric_.getPrevPosPtr();
	const Eigen::Matrix<Real,1,Eigen::Dynamic> *inv_mass_ptr = fabric_.getInvMassPtr();
	
	RigidBodySceneLoader::RigidBodyData &rb = rigid_bodies_[m_bodies[1]];

	// Set the needed parameters for DYNAMIC FABRIC
	const bool &isDynamic1 = fabric_.isDynamicParticle(m_bodies[0]);
	const Real invMass1 = (*inv_mass_ptr)(m_bodies[0]); 
	const Eigen::Matrix<Real, 3, 1> &x1 = pos_ptr->row(m_bodies[0]).transpose();
	const Eigen::Matrix<Real, 3, 1> &x1_prev = prev_pos_ptr->row(m_bodies[0]).transpose();

	// Set the needed parameters for STATIC (NON-DYNAMIC) the rigid body (note that the fabric is dynamic)
	const bool &isDynamic2 = rb.m_isDynamic;
	const Real invMass2 = 0.0;
	const Eigen::Matrix<Real, 3, 1> &x2 = rb.m_x.transpose();
	const Eigen::Matrix<Real, 3, 1> &x2_prev = x2;
	
	const Eigen::Matrix<Real, 3, 3> inertiaInverseW2 = Eigen::Matrix<Real, 3, 3>::Identity(); // Set to identity
	// const Eigen::Quaternion<Real> &q2 = rb.m_q;   

	// Create correction positions (Recall that only the second object can be a rigid body)
	// Hence we will ignore the corrections for the rigid body since we assume they are non-dynamic
	// TODO: Implement this properly when the rigid bodies are dynamic.
	Eigen::Matrix<Real, 3, 1> corr_x1; // linear position correction for the particle of the fabric
	// Eigen::Matrix<Real, 3, 1> corr_x2; // linear position correction for the rigid body
	// Eigen::Matrix<Real, 3, 1> corr_q2; // orientation correction for the rigid body

	// If both the particle and the rigid body is static, return, there is nothing to correct
	if ((!isDynamic1) && (!isDynamic2))
		return false;

	Real m_contactCompliance = 0.0; // You can also Get this as a parameter, but always zero makes sense.
	Real alpha = m_contactCompliance / (dt*dt);

	// Calculate the corrections

	// Parse the constraintInfo matrix
	// Recall m_constraintInfo contains
	// 0:	contact point in body 1 (global)
	// 1:	contact point in body 2 (global)
	// 2:	contact normal in body 2 (global)
	// 3:	contact tangent (global)
	// 0,4:  1.0 / normal^T * K * normal
	// 1,4: maximal impulse in tangent direction
	// 2,4: goal velocity in normal direction after collision
	const Eigen::Matrix<Real, 3, 1> &connector1 = m_constraintInfo.col(0);
	const Eigen::Matrix<Real, 3, 1> &connector2 = m_constraintInfo.col(1);
	const Eigen::Matrix<Real, 3, 1> &normal 	= m_constraintInfo.col(2);
	const Eigen::Matrix<Real, 3, 1> &tangent 	= m_constraintInfo.col(3);

	// 1.0 / normal^T * K * normal
	const Real &nKn_inv = m_constraintInfo(0, 4); // corresponds to Mass1 when the Rigid body is static.

	// penetration depth 
	const Real d = normal.dot(connector1 - connector2);

	// If there is no penetration, d is positive, there is nothing to correct
	if (d > 0.0)
		return false; 


	// normal direction lamdba
	m_lambda_n = -d * (1.0 / ( (1.0/nKn_inv) + alpha ));

	// To handle static friction, 
	// Relative motion of contact points
	Eigen::Matrix<Real, 3, 1> delta_x = (x1-x1_prev); // - (x2-x2_prev); RB is static

	// Tangential component of the relative motion
	Eigen::Matrix<Real, 3, 1> delta_x_tangent = delta_x - (delta_x.dot(normal))*normal;
	
	// tangent direction lambda
	m_lambda_t = -delta_x_tangent.norm() * (1.0 / ( (1.0/nKn_inv) + alpha ));
	// m_lambda_t = -delta_x.dot(tangent) * (1.0 / ( (1.0/nKn_inv) + alpha ));


	if (isDynamic1)
	{
		// Normal Direction Correction
		corr_x1 = invMass1*m_lambda_n*normal;
		// corr_x1 = -d*normal; // since alpha=0 and 1/nKn_inv=Mass1 when the RB is static.

		// Tangent Direction Correction
		if ( -m_lambda_t < m_frictionCoeffStatic*m_lambda_n ) // if tangential movement is less than the static friction effect.
			corr_x1 += -delta_x_tangent;// cancel out the tangential movement s
	}

	if (isDynamic2)
	{
		// NO CORRECTION FOR THE RIGID BODY because it is STATIC
		// corr_x2 = TODO;
		// corr_q2 = TODO;
	}
	
	// Apply the corrections
	if (isDynamic1){
		pos_ptr->row(m_bodies[0]) += corr_x1.transpose(); 
	}
	if (isDynamic2){
		// NO CORRECTION FOR THE RIGID BODY because it is STATIC
		// rb.getPosition() += corr_x2;
		// rb.getOrientation() += corr_q2;
	}
	return true;
}

bool CollisionHandler::ParticleRigidBodyContactConstraint::solveVelocityConstraint(
														pbd_object::Cloth &fabric_,
                                						std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
														const Real &dt)
{
	const Eigen::Matrix<Real,Eigen::Dynamic,3> *pos_ptr = fabric_.getPosPtr();
	Eigen::Matrix<Real,Eigen::Dynamic,3> *vel_ptr = fabric_.getVelPtr();
	const Eigen::Matrix<Real,1,Eigen::Dynamic> *inv_mass_ptr = fabric_.getInvMassPtr();
	
	RigidBodySceneLoader::RigidBodyData &rb = rigid_bodies_[m_bodies[1]];

	// Set the needed parameters for DYNAMIC FABRIC
	const bool &isDynamic1 = fabric_.isDynamicParticle(m_bodies[0]);
	const Real invMass1 = (*inv_mass_ptr)(m_bodies[0]); 
	const Eigen::Matrix<Real, 3, 1> &x1 = pos_ptr->row(m_bodies[0]).transpose();
	const Eigen::Matrix<Real, 3, 1> &v1 = vel_ptr->row(m_bodies[0]).transpose();

	// Set the needed parameters for STATIC (NON-DYNAMIC) the rigid body (note that the fabric is dynamic)
	const bool &isDynamic2 = rb.m_isDynamic;
	const Real invMass2 = 0.0;
	const Eigen::Matrix<Real, 3, 1> &x2 = rb.m_x.transpose();
	const Eigen::Matrix<Real, 3, 1> v2 = Eigen::Matrix<Real, 3, 1>::Zero();              // Set zero on initialization
	const Eigen::Matrix<Real, 3, 3> inertiaInverseW2 = Eigen::Matrix<Real, 3, 3>::Identity(); // Set to identity
	// const Eigen::Quaternion<Real> &q2 = rb.m_q;   
	const Eigen::Matrix<Real, 3, 1> omega2 = Eigen::Matrix<Real, 3, 1>::Zero();          // Set zero on initialization

	// Create correction velocities (Recall that only the second object can be a rigid body)
	// Hence we will ignore the corrections for the rigid body since we assume they are non-dynamic
	// TODO: Implement this properly when the rigid bodies are dynamic.
	Eigen::Matrix<Real, 3, 1> corr_v1; // linear velocity correction for the particle of the fabric
	Eigen::Matrix<Real, 3, 1> corr_v2; // 
	Eigen::Matrix<Real, 3, 1> corr_omega2;

	// If both the particle and the rigid body is static, return, there is nothing to correct
	if ((!isDynamic1) && (!isDynamic2))
		return false;

	// Calculate the corrections

	// Parse the constraintInfo matrix
	// Recall m_constraintInfo contains
	// 0:	contact point in body 1 (global)
	// 1:	contact point in body 2 (global)
	// 2:	contact normal in body 2 (global)
	// 3:	contact tangent (global)
	// 0,4:  1.0 / normal^T * K * normal
	// 1,4: maximal impulse in tangent direction
	// 2,4: goal velocity in normal direction after collision
	const Eigen::Matrix<Real, 3, 1> &connector1 = m_constraintInfo.col(0);
	const Eigen::Matrix<Real, 3, 1> &connector2 = m_constraintInfo.col(1);
	const Eigen::Matrix<Real, 3, 1> &normal 	= m_constraintInfo.col(2);
	const Eigen::Matrix<Real, 3, 1> &tangent 	= m_constraintInfo.col(3);

	// 1.0 / normal^T * K * normal
	const Real &nKn_inv = m_constraintInfo(0, 4);

	// goal velocity in normal direction after collision
	const Real &goal_u_rel_n = m_constraintInfo(2, 4);

	const Eigen::Matrix<Real, 3, 1> r2 = connector2 - x2;
	const Eigen::Matrix<Real, 3, 1> u2 = v2 + omega2.cross(r2);

	const Eigen::Matrix<Real, 3, 1> u_rel = v1 - u2;
	const Real u_rel_n = u_rel.dot(normal);

	//---------------------------------------------------
	// NORMAL direction velocity correction
	const Real delta_u_reln = goal_u_rel_n - u_rel_n;

	Real correctionMagnitude_n = nKn_inv * delta_u_reln;
	// Note that correctionMagnitude_n Corresponds to 
	// the impulse (m delta_v) for the particle in
	// normal direction when the rigid body is static 

	// Normal direction impulse p_n
	Eigen::Matrix<Real, 3, 1> p_n(correctionMagnitude_n * normal);
	//---------------------------------------------------

	//---------------------------------------------------
	// TANGENT direction velocity correction

	// Goal Velocity in tangent direction
	const Real u_rel_t = m_constraintInfo(1, 4) / nKn_inv; // when second object is not static, update this with tKt_inv version

	// const Real delta_u_relt = min(dt*m_frictionCoeffDynamic*abs(m_lambda_n/(dt*dt)), u_rel_t);
	Real value = dt * m_frictionCoeffDynamic * (m_lambda_n / (dt * dt));
	// Implementing abs functionality using if-else
	if (value < 0) {
		value = -value;
	}
	// Implementing min function using if-else
	const Real delta_u_relt = (value < u_rel_t) ? value : u_rel_t;
	
	Real correctionMagnitude_t = nKn_inv * delta_u_relt;

	// Tangent direction impulse p_t
	Eigen::Matrix<Real, 3, 1> p_t(-correctionMagnitude_t * tangent);
	//---------------------------------------------------

	if (isDynamic1)
	{
		// Normal direction correction
		corr_v1 = invMass1*p_n;		

		// Tangent direction correction
		corr_v1 += invMass1*p_t;		
	}

	if (isDynamic2)
	{
		// NO CORRECTION FOR THE RIGID BODY because it is STATIC
		// corr_v2 = -invMass2*p;
		// corr_omega2 = inertiaInverseW2 * (r2.cross(-p));
	}
	
	// Apply the corrections
	if (isDynamic1){
		vel_ptr->row(m_bodies[0]) += corr_v1.transpose(); 
	}
	if (isDynamic2){
		// NO CORRECTION FOR THE RIGID BODY because it is STATIC
		// rb.getVelocity() += corr_v2;
		// rb.getAngularVelocity() += corr_omega2;
	}
	return true;
}