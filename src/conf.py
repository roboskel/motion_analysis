#!/usr/bin/env python
import roslib, rospy
from std_msgs.msg import String

def start():
	rospy.init_node("motion_analysis_conf")
	keypress_topic = rospy.get_param("motion_analysis/configuration_keypress_topic", "motion_analysis/configuration/keypress")
	publisher = rospy.Publisher(keypress_topic, String, queue_size=10)
	while not rospy.is_shutdown():
		print " 'h+' to increase STANDING_PERSON_HEIGHT"
		print " 'h-' to decrease STANDING_PERSON_HEIGHT"
		print " 'bl-' to move OUTOFBED_LEFT to the left "
	        print " 'bl+' to move OUTOFBED_LEFT to the right "
        	print " 'br-' to move OUTOFBED_RIGHT to the left "
        	print " 'br+' to move OUTOFBED_RIGHT to the right"
        	print " 'cx-' to move CUPX to the left "
        	print " 'cx+' to move CUPX to the right "
        	print " 'cy+' to move CUPY upwards "
        	print " 'cy-' to move CUPY downwards "
        	print " 'cr+' to increase CUPR"
        	print " 'cr-' to decrease CUPR"
		print " 's ##' to set SENSITIVITY to ##"
		print " 'ct ##' to set CUPTHRESHOLD to ##"
		print " 'cc ##' to set CUPTHRSCOUNT to ##"
		print "'save' to save the current configuration"

		com = raw_input()
		publisher.publish(str(com))


if __name__ == '__main__':
	start()
