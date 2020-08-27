#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

def encoder_timer_callback(event):

	encoder_pub = rospy.Publisher('wheels', Int32MultiArray, queue_size=0)
	data_to_send = Int32MultiArray()
	data_to_send.data = [0,0]
	encoder_pub.publish(data_to_send)


def main():

	# initialize node
	rospy.init_node('encoder_spoof', anonymous=True)

	# encoder timer	at 10 Hz
	rospy.Timer(rospy.Duration(0.1), encoder_timer_callback) 


	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	main()

