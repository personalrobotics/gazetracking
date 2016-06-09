import zmq
import json

port = "5000"
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:" + port)

# Receive all messages
socket.setsockopt(zmq.SUBSCRIBE, '')

# Create publishers
pub_pupil = rospy.Publisher('/pupil_info', PupilInfo)
pub_gaze = rospy.Publisher('/gaze_info', GazeInfo)



while True:
	# Receive JSON message from socket, convert it to Python dict
	topic, msgstr = socket.recv_multipart()
	msg = json.loads(msgstr)
	#print prettyprint(topic, msg)

	# Convert JSON message to ROS message
	if topic == "pupil_positions":
		rospy.logdebug("Reading pupil position: \n" + prettyprint(topic, msg))
		
		# Parse and publish
		outmsg = parse_pupil_pos(msg)
		pub_pupil.publish(outmsg)

	elif topic == "gaze_positions":
		rospy.logdebug("Reading pupil position: \n" + prettyprint(topic, msg))
		
		# Parse and publish
		outmsg = parse_gaze_pos(msg)
		pub_gaze.publish(outmsg)

	elif topic == "dt":
		rospy.logdebug("Ignoring dt: " + msg)

	else:
		rospy.logerr("Unrecognized topic from socket: " + topic)


def parse_pupil_pos(msg):
	outmsg = PupilInfo()

	# Parse message data
	outmsg.timestamp = msg['timestamp']
	outmsg.index = msg['index']
	outmsg.confidence = msg['confidence']
	outmsg.norm_pos = msg['norm_pos']
	outmsg.diameter = msg['diameter']
	outmsg.method = msg['method']

	# Parse optional data
	if 'ellipse' in msg:
		outgoing.ellipse_center = msg['ellipse']['center']
		outgoing.ellipse_axis = msg['ellipse']['axes']
		outgoing.ellipse_angle = msg['ellipse']['angle']

	# Warn that 3D data hasn't been parsed
	# TODO: parse 3D data
	if 'method' == '3d c++':
		rospy.logwarn("3D information parser not yet implemented,\
			3D data from JSON message not included in ROS message.")

	return outmsg

def parse_gaze_pos(msg):
	outmsg = GazeInfo()

	# Parse message data
	outmsg.timestamp = msg['timestamp']
	outmsg.index = msg['index']
	outmsg.confidence = msg['confidence']
	outmsg.norm_pos = msg['norm_pos']



# Helper function that prints topic and message in a readable format
def prettyprint(topic, msg):
	string = "\n\n" + topic + ":\n" + msg
	return string