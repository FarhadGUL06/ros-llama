#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def publisher():
    rospy.init_node('llama_input_publisher', anonymous=True)
    pub = rospy.Publisher('/llama_parser/input', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    rospy.loginfo("LLaMA input publisher node started. Type your commands and press Enter.")

    while not rospy.is_shutdown():
        try:
            user_input = input("You: ")
            if user_input.lower() in ['exit', 'quit']:
                rospy.loginfo("Exiting publisher node.")
                break

            pub.publish(user_input)
            rate.sleep()
        except rospy.ROSInterruptException:
            break
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    publisher()
