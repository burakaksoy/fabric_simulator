# MinDistanceData.msg

std_msgs/Header header

# Contact Type
# RigidBodyContactType;			// = 0;
# ParticleContactType;			// = 1;
# ParticleRigidBodyContactType; // = 2;
int8 type

# Indexes of the objects in collision
uint32 index1 # fabric index, when there is single fabric in the simulation, it's always 0.
uint32 index2 # rigid body index

# Points on the objects
geometry_msgs/Point pointOnObject1 # point on the fabric
geometry_msgs/Point pointOnObject2 # point on the rigid body

# Normal vector, from the rigid body in world frame
geometry_msgs/Vector3 normal

# Minimum distance between the rigid body and the fabric
float64 minDistance
