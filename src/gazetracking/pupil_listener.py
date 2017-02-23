#!/usr/bin/env python
import zmq
from msgpack import loads
import json
import rospy
from gazetracking.msg import PupilInfo, GazeInfo

context = zmq.Context()
# open a req port to talk to pupil
addr = '127.0.0.1'  # remote ip or localhost
req_port = "50020"  # same as in the pupil remote gui
req = context.socket(zmq.REQ)
req.connect("tcp://{}:{}".format(addr, req_port))
# ask for the sub port
req.send_string('SUB_PORT')
sub_port = req.recv_string()

# open a sub port to listen to pupil
sub = context.socket(zmq.SUB)
sub.connect("tcp://{}:{}".format(addr, sub_port))
sub.setsockopt_string(zmq.SUBSCRIBE, u'')

# Create publishers
pub_pupil = rospy.Publisher('/pupil_info', PupilInfo, queue_size=10)
pub_gaze = rospy.Publisher('/gaze_info', GazeInfo, queue_size=10)

def parse_pupil_pos(msg):
	''' Parse pupil_position into a PupilInfo message.

	Return PupilInfo message, or -1 if msg argument was empty.
	'''
	#msg may be a list of pupil positions
	if len(msg) < 1: return -1

	outmsg = PupilInfo()

	# Parse message data
	outmsg.timestamp = msg['timestamp']
	#outmsg.index = m['index']
	outmsg.confidence = msg['confidence']
	outmsg.norm_pos = msg['norm_pos']
	outmsg.diameter = msg['diameter']
	outmsg.method = msg['method']

	# Parse optional data
	if 'ellipse' in msg:
		outmsg.ellipse_center = msg['ellipse']['center']
		outmsg.ellipse_axis = msg['ellipse']['axes']
		outmsg.ellipse_angle = msg['ellipse']['angle']

	# Warn that 3D data hasn't been parsed
	# TODO: parse 3D data
	if 'method' == '3d c++':
		rospy.logwarn("3D information parser not yet implemented,\
			3D data from JSON message not included in ROS message.")

	return outmsg

def parse_gaze_pos(msg):
	''' Parse gaze_positions into a GazeInfo message.

	Return GazeInfo message, or -1 if msg argument was empty.
	'''
	#msg may be a list of gaze positions
	if len(msg) < 1: return -1
	
	outmsg = GazeInfo()

	# Parse message data
	outmsg.timestamp = msg['timestamp']
	#outmsg.index = m['index']
	outmsg.confidence = msg['confidence']
	outmsg.norm_pos = msg['norm_pos']

	return outmsg

# Helper function that prints topic and message in a readable format
def prettyprint(topic, msg):
	string = "\n\n" + str(topic) + ":\n" + str(msg)
	return string

if __name__ == "__main__":

	rospy.loginfo("Starting pupil listener.")
	print "Starting pupil listener"

	rospy.init_node('pupillistener')

	print "listening for socket message...."
	while not rospy.is_shutdown():
		# Receive message from socket, convert it to Python dict
		topic, msgstr = sub.recv_multipart()
		msg = loads(msgstr)
		# print "" + str(topic) + ": " + str(msg)

		# Convert message to ROS message
		if "pupil" in topic:
			rospy.logdebug("Reading pupil position: \n" + prettyprint(topic, msg))
			
			# Parse and publish
			outmsg = parse_pupil_pos(msg)
			if not outmsg == -1: pub_pupil.publish(outmsg)

		elif topic == "gaze":
			rospy.logdebug("Reading pupil position: \n" + prettyprint(topic, msg))
			
			# Parse and publish
			outmsg = parse_gaze_pos(msg)
			if not outmsg == -1: pub_gaze.publish(outmsg)

		elif topic == "dt":
			rospy.logdebug("Ignoring dt: " + str(msg))

		else:
			rospy.logerr("Unrecognized topic from socket: " + topic)


