import roslaunch, rospy, time, sys, math
from std_msgs.msg import Empty
from os.path import expanduser
from gazebo_msgs.srv import GetLinkState, GetModelState, SetModelState, SetPhysicsProperties, GetPhysicsProperties
from gazebo_msgs.msg import ModelState, ODEPhysics
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from wrecs_msgs.msg import field_command
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf import transformations as t 
import numpy as np

class SimReset:
	def __init__(self, launch_names, delay_between_launches=0):
		#Subscribers and publishers
		self.simResetSub = rospy.Subscriber("sim_reset", field_command, self.simResetArrived)
		self.simRestartSub = rospy.Subscriber("sim_restart", Empty, self.simRestartArrived)

		self.simResetArrived = False
		self.simRestartArrived = True
		self.restartDonePub = rospy.Publisher("restart_done", Empty, queue_size=10)
		self.resetDonePub = rospy.Publisher("reset_done", Empty, queue_size=10)
		self.fieldCommandPub = rospy.Publisher("field_command", field_command, queue_size=10)
		self.recordPerformancePub = rospy.Publisher("sim_perf", Float32, queue_size=10)

		#Restart functionality properties
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		self.launched = {}
		self.launch_names = launch_names
		self.delay_between_launches = delay_between_launches

		#Reset functionality properties
		self.foot_z_offsets = []
		self.link_names = ["atlas::l_foot", "atlas::r_foot"]
		self.model_name = "atlas"
		self.world_frame = "world"

		self.atlas_model_ind = None
		self.link_inds = []

		self.joints = None

	def simResetArrived(self, msg):
		self.simResetArrived = True
		self.resetMsg = msg
		print "Reset command received."

	def simRestartArrived(self, msg):
		self.simRestartArrived = True
		print "Restart command received."

	def processRobotPose(self, msg):
		self.joints = msg.position

	def recordPerformance(self):
		performance_score = 0
		start_time = time.time()
		start_position = self.get_model_state(self.model_name, self.world_frame).pose.position

		#Constantly check, if the robot has fallen
		prevPosition = self.get_model_state(self.model_name, self.world_frame).pose.position
		extremeAngleReached = False
		confirmFallCounter = 1
		while not rospy.is_shutdown():
			time.sleep(0.5)
			
			currentPosition = self.get_model_state(self.model_name, self.world_frame).pose.position
			traveledDistance = self.getTraveledDistance(currentPosition, prevPosition)

			if not extremeAngleReached:
				#Check traveled distance:
				if traveledDistance > 10:
					break

				#Check for fall:
				currentOrient = self.get_model_state(self.model_name, self.world_frame).pose.orientation
				currentOrientEuler = t.euler_from_quaternion([currentOrient.x, currentOrient.y, currentOrient.z, currentOrient.w])
				if math.fabs(currentOrientEuler[0]) > 1 or math.fabs(currentOrientEuler[1]) > 1:
					extremeAngleReached = True

			else:
				if traveledDistance < 0.2:
					if confirmFallCounter < 0:
						break

					confirmFallCounter -= 1
				else:
					confirmFallCounter = 0
					extremeAngleReached = False
				

			prevPosition = currentPosition


		if confirmFallCounter < 0:
			performance_score = -1/(time.time() - start_time) # minus in front indicates that this is the reward for the failure map. The more robot can keep balance the better.
		else:
			performance_score = 5/(time.time() - start_time) # The faster the robot can reach the destination the better


		self.recordPerformancePub.publish(Float32(performance_score))

	def getTraveledDistance(self, currentPosition, prevPosition):
		return math.sqrt((currentPosition.x-prevPosition.x)**2 + (currentPosition.y-prevPosition.y)**2)

	def reset(self, msg):
		#save old physics properties turn off gravity
		physics_properties = self.get_physics_properties()
		old_gravity = physics_properties.gravity.z
		physics_properties.gravity.z = 0
		self.set_physics_properties(physics_properties.time_step, physics_properties.max_update_rate, physics_properties.gravity, physics_properties.ode_config)

		#teleport robot
		self.set_model_state(ModelState(self.model_name, Pose(Point(0, 0, 2), Quaternion()), Twist(), self.world_frame))

		#set initial positions, wait for the joints to converge
		self.fieldCommandPub.publish(msg)
		robot_pose_sub = rospy.Subscriber("/gazebo/joint_states", JointState, self.processRobotPose)
		prev_joint_position = msg.position
		while not rospy.is_shutdown():
			time.sleep(0.1)
			diff = np.mean([(math.fabs(msg.position[i] - self.joints[i]) + math.fabs(msg.position[i] - prev_joint_position[i])) for i in xrange(len(msg.setThisJoint)) if msg.setThisJoint[i]])
			
			print "Diff: ", diff

			if diff < 0.025:
				break

		robot_pose_sub.unregister()

		#rotate the robot, so that feet will get on the ground
		newAtlasPose = self.get_model_state(self.model_name, self.world_frame).pose
		footOrientation = self.get_link_state(self.link_names[0], self.world_frame).link_state.pose.orientation

		a = [newAtlasPose.orientation.x, newAtlasPose.orientation.y, newAtlasPose.orientation.z, newAtlasPose.orientation.w] 
		b = [footOrientation.x, footOrientation.y, footOrientation.z, footOrientation.w]
		res = t.quaternion_multiply(a, t.quaternion_inverse(b))

		newAtlasPose.orientation = Quaternion(res[0], res[1], res[2], res[3])
		self.set_model_state(ModelState(self.model_name, newAtlasPose, Twist(), self.world_frame))	

		#extract foot positions and offset them by foot_z_offset
		newAtlasPose = self.get_model_state(self.model_name, self.world_frame).pose
		footPosition = self.get_link_state(self.link_names[0], self.world_frame).link_state.pose.position
		newAtlasPose.position.z = newAtlasPose.position.z - footPosition.z + self.foot_z_offsets[0] + 0.01

		#teleport the robot down
		self.set_model_state(ModelState(self.model_name, newAtlasPose, Twist(), self.world_frame))

		#turn on gravity
		physics_properties.gravity.z = old_gravity
		self.set_physics_properties(physics_properties.time_step, physics_properties.max_update_rate, physics_properties.gravity, physics_properties.ode_config)

		self.resetDonePub.publish(Empty())
		self.recordPerformance()

	def restart(self):
		if bool(self.launched):
			for launch in self.launched.itervalues():
				try:
					launch.shutdown()
				except Exception:
					continue

			print "Restarting..."
		else:
			print "Starting..."
		
		for name in self.launch_names:
			launch = roslaunch.parent.ROSLaunchParent(self.uuid, [name])
			launch.start()
			self.launched[name] = launch
			time.sleep(self.delay_between_launches)

		self.initServices()

		self.foot_z_offsets = [self.get_link_state(link_name, self.world_frame).link_state.pose.position.z for link_name in self.link_names]

		self.restartDonePub.publish(Empty())

	def initServices(self):
		rospy.wait_for_service("/gazebo/get_link_state")
		rospy.wait_for_service("/gazebo/get_model_state")
		rospy.wait_for_service("/gazebo/set_model_state")
		rospy.wait_for_service("/gazebo/set_physics_properties")
		rospy.wait_for_service("/gazebo/get_physics_properties")
		self.get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
		self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
		self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
		self.set_physics_properties = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
		self.get_physics_properties = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)

if __name__ == "__main__":
	#Init subscribers and listeners
	rospy.init_node("simResServer")

	home = expanduser("~")
	launch_names = [home+"/drc_workspace/src/drc/wrecs_simulator_v4_launch/launch/atlas_blank.launch", home+"/drc_workspace/src/drc/wrecs_bringup/launch/offline.launch"]

	if len(sys.argv) <= 1:
		s = SimReset(launch_names)
	else:
		s = SimReset(launch_names, float(sys.argv[1]))

	while not rospy.is_shutdown():
		if s.simResetArrived:
			s.simResetArrived = False
			if not s.launched:
				print "Simulation needs to be started before performing reset."
			else:
				s.reset(s.resetMsg)


		elif s.simRestartArrived:
			s.restart()
			s.simRestartArrived = False

		time.sleep(0.5)

