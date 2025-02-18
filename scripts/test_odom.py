#!/usr/bin/env python3
import sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

def publish_odometry(publishers_count):
    rospy.init_node('custom_odom_publisher', anonymous=True)

    # Specific numbers for each publisher
    # pub_numbers = [0,30,930,960]
    pub_numbers = [  0,15,352,367]

    # Creating only the specified number of publishers
    pubs = [rospy.Publisher(f'/custom_static_particles_odom_{pub_numbers[i]}', Odometry, queue_size=1) 
            for i in range(publishers_count)]
    rate = rospy.Rate(100)  # 10 Hz

    # Different starting positions for each publisher
    positions = [
        Point(1, 1, 1.7),
        Point(1, -1, 1.7),
        Point(-1, 1, 1.7),
        Point(-1, -1, 1.7)
    ]

    orientation = Quaternion(0, 0.7071081, 0, 0.7071055)

    # Time tracking
    start_time = rospy.Time.now()
    last_time = start_time

    # Phase durations in seconds
    max_velocity = 1
    zero_velocity_duration = 4
    ramp_up_duration = 1
    constant_velocity_duration = 2
    ramp_down_duration = 1
    ramp_down_start = zero_velocity_duration + ramp_up_duration + constant_velocity_duration

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = (current_time - last_time).to_sec()
        total_elapsed = (current_time - start_time).to_sec()

        # Velocity update logic
        if total_elapsed < zero_velocity_duration:
            velocity = 0.0
        elif total_elapsed < zero_velocity_duration + ramp_up_duration:
            velocity = (total_elapsed - zero_velocity_duration) / ramp_up_duration * max_velocity
        elif total_elapsed < ramp_down_start:
            velocity = max_velocity
        else:
            # Ramp down logic
            ramp_down_end = ramp_down_start + ramp_down_duration
            if total_elapsed < ramp_down_end:
                velocity = max_velocity * (1.0 - (total_elapsed - ramp_down_start) / ramp_down_duration)
            else:
                velocity = 0.0

        # Update and publish for each active publisher
        for i, pub in enumerate(pubs):
            positions[i].x += velocity * elapsed_time

            # Prepare Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = f"base_link_{pub_numbers[i]}"
            odom_msg.pose.pose.position = positions[i]
            odom_msg.pose.pose.orientation = orientation
            odom_msg.twist.twist.linear.x = 0.0 # velocity

            pub.publish(odom_msg)

        last_time = current_time
        rate.sleep()

if __name__ == '__main__':
    try:
        # Default to 4 publishers if no argument is given
        num_publishers = int(sys.argv[1]) if len(sys.argv) > 1 else 4
        publish_odometry(num_publishers)
    except rospy.ROSInterruptException:
        pass
    except ValueError:
        print("Invalid number of publishers. Please provide a valid number.")

