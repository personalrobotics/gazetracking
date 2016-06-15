import zmq
import json
import rospy
from gazetracking.msg import PupilInfo, GazeInfo

port = "5000"
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:" + port)

# Receive all messages
socket.setsockopt(zmq.SUBSCRIBE, '')

# Create publishers
pub_pupil = rospy.Publisher('/pupil_info', PupilInfo, queue_size=10)
pub_gaze = rospy.Publisher('/gaze_info', GazeInfo, queue_size=10)

def parse_pupil_pos(msg):
	''' Parse pupil_position into a PupilInfo message.

	Return PupilInfo message, or -1 if msg argument was empty.
	'''
	#msg may be a list of pupil positions
	if len(msg) < 1: return -1

	for m in msg:
		outmsg = PupilInfo()

		# Parse message data
		outmsg.timestamp = m['timestamp']
		outmsg.index = m['index']
		outmsg.confidence = m['confidence']
		outmsg.norm_pos = m['norm_pos']
		outmsg.diameter = m['diameter']
		outmsg.method = m['method']

		# Parse optional data
		if 'ellipse' in msg:
			outgoing.ellipse_center = m['ellipse']['center']
			outgoing.ellipse_axis = m['ellipse']['axes']
			outgoing.ellipse_angle = m['ellipse']['angle']

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
	
	for m in msg:
		outmsg = GazeInfo()

		# Parse message data
		outmsg.timestamp = m['timestamp']
		outmsg.index = m['index']
		outmsg.confidence = m['confidence']
		outmsg.norm_pos = m['norm_pos']

		return outmsg

# Helper function that prints topic and message in a readable format
def prettyprint(topic, msg):
	string = "\n\n" + str(topic) + ":\n" + str(msg)
	return string

if __name__ == "__main__":

	rospy.loginfo("Starting pupil listener.")
	print "Starting pupil listener"

	rospy.init_node('pupillistener')

	while not rospy.is_shutdown():
		# Receive JSON message from socket, convert it to Python dict
		topic, msgstr = socket.recv_multipart()
		msg = json.loads(msgstr)
		# print prettyprint(topic, msg)

		# Convert JSON message to ROS message
		if topic == "pupil_positions":
			rospy.logdebug("Reading pupil position: \n" + prettyprint(topic, msg))
			
			# Parse and publish
			outmsg = parse_pupil_pos(msg)
			if not outmsg == -1: pub_pupil.publish(outmsg)

		elif topic == "gaze_positions":
			rospy.logdebug("Reading pupil position: \n" + prettyprint(topic, msg))
			
			# Parse and publish
			outmsg = parse_gaze_pos(msg)
			if not outmsg == -1: pub_gaze.publish(outmsg)

		elif topic == "dt":
			rospy.logdebug("Ignoring dt: " + str(msg))

		else:
			rospy.logerr("Unrecognized topic from socket: " + topic)


