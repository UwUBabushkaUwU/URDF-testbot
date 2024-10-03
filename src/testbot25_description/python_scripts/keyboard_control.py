#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import sys
import tty
import termios
from select import select

# Create a dictionary to keep track of the current value of 'i' for each joint
joint_values = {
    'Revolute_15': 0,
    'Revolute_16': 0,
    'Revolute_17': 0,
    'Revolute_18': 0
}

def getKey(settings, timeout):
    """Handles reading a single key press across platforms."""
    if sys.platform == 'win32':
        import msvcrt
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publish_commands():
    # Initialize the ROS node
    rospy.init_node('multi_position_publisher', anonymous=True)

    # Define publishers for multiple position controller topics
    pub_revolute_15 = rospy.Publisher('/testbot25/Revolute_15_position_controller/command', Float64, queue_size=10)
    pub_revolute_16 = rospy.Publisher('/testbot25/Revolute_16_position_controller/command', Float64, queue_size=10)
    pub_revolute_17 = rospy.Publisher('/testbot25/Revolute_17_position_controller/command', Float64, queue_size=10)
    pub_revolute_18 = rospy.Publisher('/testbot25/Revolute_18_position_controller/command', Float64, queue_size=10)

    # Save current terminal settings
    settings = termios.tcgetattr(sys.stdin)

    # Set the frequency (100 Hz)
    rate = rospy.Rate(100)

    print("Press 'q' for Revolute_15, 'w' for Revolute_16, 'e' for Revolute_17, and 'r' for Revolute_18.")
    print("Press 's' to stop.")

    while not rospy.is_shutdown():
        key = getKey(settings, 0.1)  # 0.1-second timeout for key presses

        # Check key presses and adjust joint values accordingly
        if key == 'q':
            joint_values['Revolute_15'] += 1
            pub_revolute_15.publish(joint_values['Revolute_15'] / 10.0)
            rospy.loginfo(f"Published to Revolute_15: {joint_values['Revolute_15'] / 10.0}")
            
        elif key == 'w':
            joint_values['Revolute_16'] += 1
            pub_revolute_16.publish(joint_values['Revolute_16'] / 10.0)
            rospy.loginfo(f"Published to Revolute_16: {joint_values['Revolute_16'] / 10.0}")
            
        elif key == 'e':
            joint_values['Revolute_17'] += 1
            pub_revolute_17.publish(joint_values['Revolute_17'] / 10.0)
            rospy.loginfo(f"Published to Revolute_17: {joint_values['Revolute_17'] / 10.0}")
            
        elif key == 'r':
            joint_values['Revolute_18'] += 1
            pub_revolute_18.publish(joint_values['Revolute_18'] / 10.0)
            rospy.loginfo(f"Published to Revolute_18: {joint_values['Revolute_18'] / 10.0}")

        # Stop if 's' is pressed
        elif key == 's':
            print("Stopping the publisher.")
            break

        rate.sleep()

    # Restore terminal settings when the loop exits
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    try:
        publish_commands()
    except rospy.ROSInterruptException:
        pass
