#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist

"""
NOT IMPLEMENTED YET!!!!!!!!!!!!!!!!!!!!!!!!!!!!
COPIED FROM DLO SIMULATOR, NEEDS TO BE MODIFIED FOR FABRIC SIMULATOR
"""

def publish_velocity(publishers_count):
    rospy.init_node('custom_velocity_publisher', anonymous=True)

    # Publishers for Twist messages
    # Specific numbers for each publisher
    # pub_numbers = [0,30,930,960]
    pub_numbers = [  0,15,352,367]

    # Creating only the specified number of publishers
    pubs = [rospy.Publisher(f'/custom_static_particles_cmd_vel_{pub_numbers[i]}', Twist, queue_size=1) 
            for i in range(publishers_count)]
    
    rate = rospy.Rate(100)  # 10 Hz

    # Time tracking
    start_time = rospy.Time.now()
    last_time = start_time

    # Phase durations in seconds
    max_velocity = 1
    zero_velocity_duration = 4
    ramp_up_duration = 0.1
    constant_velocity_duration = 0.4
    ramp_down_duration = 0.1
    ramp_down_start = zero_velocity_duration + ramp_up_duration + constant_velocity_duration

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        total_elapsed = (current_time - start_time).to_sec()

        # Velocity update logic
        if total_elapsed < zero_velocity_duration:
            linear_velocity = 0.0
        elif total_elapsed < zero_velocity_duration + ramp_up_duration:
            linear_velocity = (total_elapsed - zero_velocity_duration) / ramp_up_duration * max_velocity
        elif total_elapsed < ramp_down_start:
            linear_velocity = max_velocity
        else:
            # Ramp down logic
            ramp_down_end = ramp_down_start + ramp_down_duration
            if total_elapsed < ramp_down_end:
                linear_velocity = max_velocity * (1.0 - (total_elapsed - ramp_down_start) / ramp_down_duration)
            else:
                linear_velocity = 0.0

        
        # Update and publish for each active publisher
        for i, pub in enumerate(pubs):
            # Prepare and publish Twist message
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            # Set other velocities if needed..

            pub.publish(twist_msg)
        
        last_time = current_time
        rate.sleep()

if __name__ == '__main__':
    try:
        # Default to 4 publishers if no argument is given
        num_publishers = int(sys.argv[1]) if len(sys.argv) > 1 else 4
        publish_velocity(num_publishers)
    except rospy.ROSInterruptException:
        pass
    except ValueError:
        print("Invalid number of publishers. Please provide a valid number.")