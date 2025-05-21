#! /usr/bin/env python

import rospy
import tf2_ros
from std_msgs.msg import String
import rospkg
import yaml
from geometry_msgs.msg import PoseStamped
import tf as tf
from known_locations_parser.srv import *

def read_yaml(known_locations_file = '/known_locations.yaml'):
	rospack = rospkg.RosPack()
	file_path = rospack.get_path('config_files') + known_locations_file

	with open(file_path, 'r') as file:
		content = yaml.safe_load(file)
	return content

def match_location(location):
	content = read_yaml()
	try:
		return True, content[location]
	except:
		return False, 'No location found'


class location_server():

	def __init__(self):
		self.goal = PoseStamped()
		self.goal.header.frame_id = 'map'
		self.goal.pose.position.z = 0

		self.positionXYT = [0.0, 0.0, 0.0]

		self.service_loc = rospy.Service('/knowledge/known_locations_parser_server', Locate_server, self.callback_locations)


	def callback_locations(self, req):
		succ,loc = match_location(req.location_name.data)
		resp = Locate_serverResponse()
		if succ:
			print (req.location_name.data)
			self.positionXYT = loc[:3]
			self._fill_msg()
			resp.pose_stamped = self.goal
			resp.success.data = True
		else:
			print(loc)
			resp.success.data = False
		
		
		return resp

	def _fill_msg(self):
		self.goal.header.stamp = rospy.Time.now()
		self.goal.pose.position.x = self.positionXYT[0]['x']
		self.goal.pose.position.y = self.positionXYT[1]['y']
		
		quat = tf.transformations.quaternion_from_euler(0, 0, self.positionXYT[2]['theta'])

		self.goal.pose.orientation.x = quat[0]
		self.goal.pose.orientation.y = quat[1]
		self.goal.pose.orientation.z = quat[2]
		self.goal.pose.orientation.w = quat[3]



def main():
	rospy.init_node('known_locations_parser_server')

	loc_server = location_server()

	rospy.spin()

if __name__ == '__main__':
	main()
