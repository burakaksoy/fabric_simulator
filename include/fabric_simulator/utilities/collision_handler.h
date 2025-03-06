/*
 * Author: Burak Aksoy
 * handles the collisions, ie. contacts between cloth and rigid bodies
 */

#ifndef COLLISION_HANDLER_H
#define COLLISION_HANDLER_H

// #include "Discregrid/All" // rigid_body_scene_loader.h already includes it
#include "fabric_simulator/utilities/AABB.h"
#include "fabric_simulator/utilities/BoundingSphereHierarchy.h"

#include "fabric_simulator/Common.h"
// #include "fabric_simulator/utilities/cloth.h"
namespace pbd_object {
    class Cloth; // forward declare
}

#include "fabric_simulator/utilities/rigid_body_scene_loader.h"

#include <memory> // needed for std::shared_ptr

namespace utilities
{
    class CollisionHandler
    {
    public:
        static const unsigned int RigidBodyContactType;			// = 0;
        static const unsigned int ParticleContactType;			// = 1;
        static const unsigned int ParticleRigidBodyContactType; // = 2;

        // typedef: This keyword is used  to create an alias for a type. 
		// It makes complex types easier to read and use.
		// Define a pointer to a function (ContactCallbackFunction) 
		// that matches this specific signature (return type and parameters).
		// This function pointer is used to store a callback function. 
		// This callback function is intended to be called whenever a contact
		// is detected.
		// The actual function pointed to by this pointer can be defined 
		// anywhere in the program, as long as it matches the specified 
		// signature. When it's time to handle a contact, the 
		// CollisionDetection class will call this function, 
		// passing along all the necessary parameters.
		typedef void (*ContactCallbackFunction)(const unsigned int contactType, 
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



        using Grid = Discregrid::CubicLagrangeDiscreteGrid;
		using GridPtr = std::shared_ptr<Discregrid::CubicLagrangeDiscreteGrid>;


        struct CollisionObject
        {
            static const unsigned int RigidBodyCollisionObjectType;		// = 0;
            static const unsigned int TriangleModelCollisionObjectType; // = 1;

            AABB m_aabb;
            unsigned int m_bodyIndex;
            unsigned int m_bodyType;

            bool m_testMesh;
			Real m_invertSDF;
			PointCloudBSH m_bvh;

            std::string m_sdfFile;
			Eigen::Matrix<Real, 3, 1> m_scale;
			GridPtr m_sdf;
            
            // Functions
            CollisionObject() 
            {
                m_testMesh = true; 
                m_invertSDF = 1.0; 
            }
            
            ~CollisionObject() {}

            bool collisionTest(const Eigen::Matrix<Real, 3, 1> &x, 
                               const Real tolerance, 
                               Eigen::Matrix<Real, 3, 1> &cp, 
                               Eigen::Matrix<Real, 3, 1> &n, 
                               Real &dist, 
                               const Real maxDist = 0.0);

			double distance(const Eigen::Vector3d &x, 
                            const Real tolerance); // Why double and why not Real?


        };

        struct ContactData
		{
            // Contact Type
			char m_type; // i.e., one of ContactTypes defined above 
            //(e.g, RigidBodyContactType, ParticleRigidBodyContactType)

            // index of the particle in the collision objects (?)
			unsigned int m_index1; // e.g., 34th vertex in the cloth mesh
			unsigned int m_index2; // e.g., 2nd  rigid body 
			
            // Contact points on each collision object
            Eigen::Matrix<Real, 3, 1> m_cp1;
			Eigen::Matrix<Real, 3, 1> m_cp2;

			// Contact normal 
            Eigen::Matrix<Real, 3, 1> m_normal;
			
            // Distance to measure penetration of the contact
            Real m_dist;
            
            // Restitution and friction coefficients of the contact
			Real m_restitution;
			Real m_frictionCoeffStatic;
			Real m_frictionCoeffDynamic;
		};

        struct MinDistanceData {
            // Contact Type
			char m_type; // i.e., one of ContactTypes defined above 
            //(e.g, RigidBodyContactType, ParticleRigidBodyContactType)

            // index of the particle in the collision objects 
			unsigned int m_index1; // e.g., 34th vertex in the cloth mesh
			unsigned int m_index2; // e.g., 2nd  rigid body 
            
            Eigen::Matrix<Real, 3, 1> m_pointOnObject1;
            Eigen::Matrix<Real, 3, 1> m_pointOnObject2;

            Eigen::Matrix<Real, 3, 1> m_normal;

            Real m_minDistance;
        };


        struct RigidBodyContactConstraint
        {
            /** indices of the linked (contacted) bodies */
            std::array<unsigned int, 2> m_bodies;
            
            Real m_frictionCoeffStatic;
            Real m_frictionCoeffDynamic;

            Eigen::Matrix<Real, 3, 5> m_constraintInfo;
            // constraintInfo contains
            // 0:	contact point in body 0 (global)
            // 1:	contact point in body 1 (global)
            // 2:	contact normal in body 1 (global)
            // 3:	contact tangent (global)
            // 0,4:  1.0 / normal^T * K * normal
            // 1,4: maximal impulse in tangent direction
            // 2,4: goal velocity in normal direction after collision

            Real m_lambda_n;
            Real m_lambda_t;

            RigidBodyContactConstraint() {}
            ~RigidBodyContactConstraint() {}

            bool initConstraint(
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
                                const Real &frictionCoeffDynamic);

            bool solvePositionConstraint(pbd_object::Cloth &fabric_,
                                        std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
                                        const Real &dt);

            bool solveVelocityConstraint(pbd_object::Cloth &fabric_,
                                        std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
                                        const Real &dt);
        };

        struct ParticleRigidBodyContactConstraint
        {
            
            /** indices of the linked bodies */
            std::array<unsigned int, 2> m_bodies;
            
            Real m_frictionCoeffStatic;
            Real m_frictionCoeffDynamic;

            Eigen::Matrix<Real, 3, 5> m_constraintInfo;
            // constraintInfo contains
            // 0:	contact point in body 0 (global)
            // 1:	contact point in body 1 (global)
            // 2:	contact normal in body 1 (global)
            // 3:	contact tangent (global)
            // 0,4:  1.0 / normal^T * K * normal
            // 1,4: maximal impulse in tangent direction
            // 2,4: goal velocity in normal direction after collision

            Real m_lambda_n;
            Real m_lambda_t;

            ParticleRigidBodyContactConstraint() {}
            ~ParticleRigidBodyContactConstraint() {}

            bool initConstraint(
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
                                const Real &frictionCoeffDynamic);

            bool solvePositionConstraint(pbd_object::Cloth &fabric_,
                                        std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
                                        const Real &dt);

            bool solveVelocityConstraint(pbd_object::Cloth &fabric_,
                                        std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_,
                                        const Real &dt);
        };

        struct NearestSurfaceData
        {
            bool found = false;
            Real distance = std::numeric_limits<Real>::max();
            Eigen::Matrix<Real, 3, 1> closestPointWorld;
            Eigen::Matrix<Real, 3, 1> normalWorld;
            unsigned int rigidBodyIndex = 0; 
        };

        // Functions
        // CollisionHandler(); // Default constructor
        // Constructor with references to fabric and rigid bodies
        CollisionHandler(pbd_object::Cloth &fabric, 
                         std::vector<RigidBodySceneLoader::RigidBodyData> &rigid_bodies);

        ~CollisionHandler(); // Calls cleanupCollisionObjects()

        // Deletes all the collision object handled by this class
        void cleanupCollisionObjects()
		{
			for (unsigned int i = 0; i < m_collisionObjects.size(); i++)
				delete m_collisionObjects[i];
			m_collisionObjects.clear();
		}

        // deletes all the elements inside the vectors that contains the constraint data.
        void resetContacts()
        {
            m_rigidBodyContactConstraints.clear();
	        m_particleRigidBodyContactConstraints.clear();
        }

        // Getter and Setter for m_tolerance
        Real getContactTolerance() const 
		{ return m_tolerance; }
		void setContactTolerance(Real val) 
		{ m_tolerance = val; }

        // Setter for the contact callback function (ContactCallbackFunction) pointer
        void setContactCallback(CollisionHandler::ContactCallbackFunction val, void *userData)
		{
			m_contactCB = val;
			m_contactCBUserData = userData;
		}

        // Getter for m_collisionObjects
        std::vector<CollisionObject *> &getCollisionObjects() 
		{ return m_collisionObjects; } 

        // Register Rigid Body Contact to the appropriate handler
        void addRigidBodyContact(const unsigned int rbIndex1, 
                                const unsigned int rbIndex2,
                                const Eigen::Matrix<Real, 3, 1> &cp1, 
                                const Eigen::Matrix<Real, 3, 1> &cp2,
                                const Eigen::Matrix<Real, 3, 1> &normal, 
                                const Real dist,
                                const Real restitutionCoeff, 
                                const Real frictionCoeffStatic,
                                const Real frictionCoeffDynamic)
        {
            // If the callback function is set, it is called with the contact information.
            // In other words, 'addRigidBodyContact' function detects contact between two 
            // rigid bodies and triggers a callback function to handle this event, passing 
            // along all relevant information about the contact.
            if (m_contactCB)
                m_contactCB(RigidBodyContactType, 
                            rbIndex1, 
                            rbIndex2, 
                            cp1, 
                            cp2, 
                            normal, 
                            dist, 
                            restitutionCoeff, 
                            frictionCoeffStatic,
                            frictionCoeffDynamic,
                            m_contactCBUserData);
        }

        // Register Particle - Rigid Body Contact to the appropriate handler
        void addParticleRigidBodyContact(const unsigned int particleIndex, 
                                        const unsigned int rbIndex,
                                        const Eigen::Matrix<Real, 3, 1> &cp1, 
                                        const Eigen::Matrix<Real, 3, 1> &cp2,
                                        const Eigen::Matrix<Real, 3, 1> &normal, 
                                        const Real dist,
                                        const Real restitutionCoeff, 
                                        const Real frictionCoeffStatic,
                                        const Real frictionCoeffDynamic)
        {
            // If the callback function is set, it is called with the contact information.
            // In other words, 'addParticleRigidBodyContact' function detects contact between a
            // particle and a rigid body, and triggers a callback function to handle this event, 
            // passing along all relevant information about the contact.
            if (m_contactCB)
                m_contactCB(ParticleRigidBodyContactType, 
                            particleIndex, 
                            rbIndex, 
                            cp1, 
                            cp2, 
                            normal, 
                            dist, 
                            restitutionCoeff, 
                            frictionCoeffStatic,
                            frictionCoeffDynamic,
                            m_contactCBUserData);
        }        

        // Given the collision object (co), update its AABB with its current state
        // (fabric_ and rigid_bodies_  are used to read the current state of the 
        // collision object, whichever corresponds to the co)
        void updateAABB(pbd_object::Cloth &fabric_,
                        const std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_, 
                        CollisionHandler::CollisionObject *co);

        // Function to detect the collisions (contacts)
        void collisionDetection();


        // Getters for the contact constraint vectors
        std::vector<RigidBodyContactConstraint> & getRigidBodyContactConstraints(){
            return m_rigidBodyContactConstraints;
        }

        std::vector<ParticleRigidBodyContactConstraint> & getParticleRigidBodyContactConstraints(){
            return m_particleRigidBodyContactConstraints;
        }
        
        // Contact Constraint Adder Functions
        bool addRigidBodyContactConstraint(const unsigned int rbIndex1, 
                                           const unsigned int rbIndex2, 
                                           const Eigen::Matrix<Real, 3, 1> &cp1, 
                                           const Eigen::Matrix<Real, 3, 1> &cp2, 
                                           const Eigen::Matrix<Real, 3, 1> &normal, 
                                           const Real dist,
                                           const Real restitutionCoeff, 
                                           const Real frictionCoeffStatic,
                                           const Real frictionCoeffDynamic)
        {
            m_rigidBodyContactConstraints.emplace_back(RigidBodyContactConstraint());
            RigidBodyContactConstraint &cc = m_rigidBodyContactConstraints.back();
            const bool res = cc.initConstraint(fabric_,
                                               rigid_bodies_,
                                               rbIndex1, 
                                               rbIndex2, 
                                               cp1, 
                                               cp2, 
                                               normal, 
                                               dist, 
                                               restitutionCoeff, 
                                               frictionCoeffStatic,
                                               frictionCoeffDynamic);
            if (!res)
                m_rigidBodyContactConstraints.pop_back();
            return res;
        }

        bool addParticleRigidBodyContactConstraint(const unsigned int particleIndex, 
                                                   const unsigned int rbIndex, 
                                                   const Eigen::Matrix<Real, 3, 1> &cp1, 
                                                   const Eigen::Matrix<Real, 3, 1> &cp2, 
                                                   const Eigen::Matrix<Real, 3, 1> &normal, 
                                                   const Real dist,
                                                   const Real restitutionCoeff, 
                                                   const Real frictionCoeffStatic,
                                                   const Real frictionCoeffDynamic)
        {
            m_particleRigidBodyContactConstraints.emplace_back(ParticleRigidBodyContactConstraint());
            ParticleRigidBodyContactConstraint &cc = m_particleRigidBodyContactConstraints.back();
            const bool res = cc.initConstraint(fabric_,
                                               rigid_bodies_,
                                               particleIndex,
                                               rbIndex, 
                                               cp1, 
                                               cp2, 
                                               normal, 
                                               dist, 
                                               restitutionCoeff, 
                                               frictionCoeffStatic,
                                               frictionCoeffDynamic);
            if (!res)
                m_particleRigidBodyContactConstraints.pop_back();
            return res;
        }

        static void computeMatrixK(const Eigen::Matrix<Real, 3, 1> &connector,
                            const Real invMass,
                            const Eigen::Matrix<Real, 3, 1> &x,
                            const Eigen::Matrix<Real, 3, 3> &inertiaInverseW,
                            Eigen::Matrix<Real, 3, 3> &K)
        {
            if (invMass != 0.0)
            {
                const Eigen::Matrix<Real, 3, 1> v = connector - x;
                const Real a = v[0];
                const Real b = v[1];
                const Real c = v[2];

                // J is symmetric
                const Real j11 = inertiaInverseW(0,0);
                const Real j12 = inertiaInverseW(0,1);
                const Real j13 = inertiaInverseW(0,2);
                const Real j22 = inertiaInverseW(1,1);
                const Real j23 = inertiaInverseW(1,2);
                const Real j33 = inertiaInverseW(2,2);

                K(0,0) = c*c*j22 - b*c*(j23 + j23) + b*b*j33 + invMass;
                K(0,1) = -(c*c*j12) + a*c*j23 + b*c*j13 - a*b*j33;
                K(0,2) = b*c*j12 - a*c*j22 - b*b*j13 + a*b*j23;
                K(1,0) = K(0,1);
                K(1,1) = c*c*j11 - a*c*(j13 + j13) + a*a*j33 + invMass;
                K(1,2) = -(b*c*j11) + a*c*j12 + a*b*j13 - a*a*j23;
                K(2,0) = K(0,2);
                K(2,1) = K(1,2);
                K(2,2) = b*b*j11 - a*b*(j12 + j12) + a*a*j22 + invMass;
            }
            else
                K.setZero();
        }


        void addCubicSDFCollisionObject(const unsigned int bodyIndex, 
                                        const unsigned int bodyType, 
                                        const Eigen::Matrix<Real,Eigen::Dynamic,3> *vertices, 
                                        const unsigned int numVertices,
                                        GridPtr sdf, 
                                        const Eigen::Matrix<Real, 3, 1> &scale, 
                                        const bool testMesh /*= true*/, 
                                        const bool invertSDF /*= false*/)
        {
            CollisionHandler::CollisionObject *co = new CollisionHandler::CollisionObject();
            co->m_bodyIndex = bodyIndex;
            co->m_bodyType = bodyType;

            co->m_sdfFile = "";
            co->m_scale = scale;
            co->m_sdf = sdf;
            
            co->m_bvh.init(vertices, numVertices);
            co->m_bvh.construct();
            
            co->m_testMesh = testMesh;
            co->m_invertSDF = 1.0;
            if (invertSDF)
                co->m_invertSDF = -1.0;
            
            m_collisionObjects.push_back(co);
        }

        void addCollisionObjectWithoutGeometry(const unsigned int bodyIndex, 
                                                const unsigned int bodyType,
                                                const Eigen::Matrix<Real,Eigen::Dynamic,3> *vertices,
                                                const unsigned int numVertices,
                                                const bool testMesh /*= true*/)
        {
            CollisionHandler::CollisionObject *co = new CollisionHandler::CollisionObject();
            co->m_bodyIndex = bodyIndex;
            co->m_bodyType = bodyType;
            
            // m_scale is not set here is NOT a problem
            // since scale is only used for rigid bodies in
            // distance() and collisionTest() functions

            co->m_bvh.init(vertices, numVertices);
            co->m_bvh.construct();
            
            co->m_testMesh = testMesh;
            co->m_invertSDF = 1.0;
            
            m_collisionObjects.push_back(co);
        }

        void solveContactPositionConstraints(const Real &dt)
        {
            for (unsigned int i = 0; i < m_rigidBodyContactConstraints.size(); i++)
            {
                m_rigidBodyContactConstraints[i].solvePositionConstraint(fabric_,
                                                                         rigid_bodies_, dt);
            }
            for (unsigned int i = 0; i < m_particleRigidBodyContactConstraints.size(); i++)
            {
                m_particleRigidBodyContactConstraints[i].solvePositionConstraint(fabric_,
                                                                                 rigid_bodies_, dt);
            }
        }

        void solveContactVelocityConstraints(const Real &dt)
        {
            for (unsigned int i = 0; i < m_rigidBodyContactConstraints.size(); i++)
            {
                m_rigidBodyContactConstraints[i].solveVelocityConstraint(fabric_,
                                                                         rigid_bodies_, dt);
            }
            for (unsigned int i = 0; i < m_particleRigidBodyContactConstraints.size(); i++)
            {
                m_particleRigidBodyContactConstraints[i].solveVelocityConstraint(fabric_,
                                                                                 rigid_bodies_, dt);
            }
        }

        void computeMinDistancesToRigidBodies(std::vector<std::vector<MinDistanceData>> &min_distances_mt);

        // This projects a single point onto the nearest rigid-body surface
        // among all collision objects. Returns data in nearestData.
        bool projectPointOnRigidBodySurface(const Eigen::Matrix<Real, 3, 1> &pointWorld,
                                            NearestSurfaceData &nearestData);

    private:
        // Variables
        Real m_tolerance;

		ContactCallbackFunction m_contactCB;
		void *m_contactCBUserData; // fabric_simulator is passed here with this user data
		std::vector<CollisionObject *> m_collisionObjects;

        // Constraint data vectors
        std::vector<RigidBodyContactConstraint> m_rigidBodyContactConstraints;
        std::vector<ParticleRigidBodyContactConstraint> m_particleRigidBodyContactConstraints;

        pbd_object::Cloth &fabric_;
        std::vector<utilities::RigidBodySceneLoader::RigidBodyData> &rigid_bodies_;

        
        // Functions

        // Common initialization code for the constructors
        void commonInit() {
            m_collisionObjects.reserve(1000);
            m_contactCB = NULL;
            m_tolerance = static_cast<Real>(0.1); // default 0.1
        }

        // update the minimum and maximum corners of the AABB to include the given point. 
        void updateAABB(const Eigen::Matrix<Real, 3, 1> &p, AABB &aabb)
        {
            // If the current MINIMUM coordinates of the AABB (m_p[0])
            // is GREATER than the coordinate of the point p, 
            // update the coordinate of the AABB's minimum point 
            // with the value from p.
            if (aabb.m_p[0][0] > p[0]) // x
                aabb.m_p[0][0] = p[0];
            if (aabb.m_p[0][1] > p[1]) // y
                aabb.m_p[0][1] = p[1];
            if (aabb.m_p[0][2] > p[2]) // z
                aabb.m_p[0][2] = p[2];
            // If the current MAXIMUM coordinates of the AABB (m_p[1])
            // is LESS than the coordinate of the point p, 
            // update the coordinate of the AABB's maximum point 
            // with the value from p.
            if (aabb.m_p[1][0] < p[0]) // x
                aabb.m_p[1][0] = p[0]; 
            if (aabb.m_p[1][1] < p[1]) // y
                aabb.m_p[1][1] = p[1];
            if (aabb.m_p[1][2] < p[2]) // z
                aabb.m_p[1][2] = p[2];
        }

        void collisionDetectionRigidBodies(RigidBodySceneLoader::RigidBodyData &rb1, 
                                    CollisionObject *co1, 
                                    RigidBodySceneLoader::RigidBodyData &rb2,
                                    CollisionObject *co2,
                                    const Real &restitutionCoeff, 
                                    const Real &frictionCoeffStatic, 
                                    const Real &frictionCoeffDynamic, 
                                    std::vector<std::vector<ContactData> > &contacts_mt);

		void collisionDetectionRBSolid(pbd_object::Cloth &fabric_,
									   CollisionObject *co1, 
									   RigidBodySceneLoader::RigidBodyData &rb2, 
									   CollisionObject *co2, 
									   const Real &restitutionCoeff, 
									   const Real &frictionCoeffStatic, 
									   const Real &frictionCoeffDynamic, 
									   std::vector< std::vector<ContactData> > &contacts_mt);

        void computeMinDistanceRBSolid(pbd_object::Cloth &fabric_,
                                       CollisionObject *co1, 
                                       RigidBodySceneLoader::RigidBodyData &rb2, 
                                       CollisionObject *co2,
                                       MinDistanceData& minDistanceData);

    };

} // namespace utilities

#endif /* !COLLISION_HANDLER_H */