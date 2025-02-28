# SyncParticleUpdateData.msg

# Particle Update Type Enum
int8 SIMPLE_UPDATE = 0 # this is a simple update to make a particle dynamic or static, does not add/remove odom and cmd_vel topics to the particle, only needs position and basic staticity/dynamicity of the particle to be synced.
int8 CHANGE_PARTICLE_DYNAMICITY = 1 # this also adds/removes odom and cmd_vel topics to the particle in addition to making it dynamic or static with position
int8 ATTACH_EXTERNAL_ODOM_FRAME = 2 # this attaches an external odometry frame to the particle, and adds the odom topic to the particle

int8 type

bool is_dynamic # is particle being made dynamic or static

# Deformable object ID 
uint32 fabric_id # fabic id, when there is single fabric in the simulation, it's always 0.

# Particle ID
uint32 particle_id # particle id in the fabric

# Position of the particle in world frame
geometry_msgs/Point position # position of the particle in world frame

string odom_topic # topic of the odometry message to request to subscribe to and attach/detach to/from
string cmd_vel_topic # topic of the cmd_vel message to request to subscribe to and attach/detach to/from

string wrench_frame_id # wrench frame id, this is the frame in which the wrench is applied to the particle (needed for attaching external odometry frame)
geometry_msgs/Point attached_rel_pos # position of the particle in world frame, this is needed for attaching external odometry frame
geometry_msgs/Quaternion attached_orientation # orientation of the particle in world frame, this is needed for attaching external odometry frame