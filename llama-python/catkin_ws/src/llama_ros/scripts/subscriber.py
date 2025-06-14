#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Received commands: {msg.data}")
    with open("/root/commands.log", "a") as f:
        f.write(msg.data + "\n")

def subscriber():
    rospy.init_node('llama_output_subscriber', anonymous=True)
    rospy.Subscriber('/llama_parser/commands', String, callback)
    rospy.loginfo("LLaMA output subscriber node started, listening to commands...")
    rospy.spin()

if __name__ == '__main__':
    subscriber()
