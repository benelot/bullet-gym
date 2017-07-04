import gym, gym.spaces, gym.utils, gym.utils.seeding
import numpy as np
import pybullet as p
import os


class PybulletMujocoXmlEnv(gym.Env):
	"""
	Base class for MuJoCo .xml actors in a Scene.
	These environments create single-player scenes and behave like normal Gym environments, if
	you don't use multiplayer.
	"""

	metadata = {
		'render.modes': ['human', 'rgb_array'],
		'video.frames_per_second': 60
		}

	def __init__(self, model_xml, robot_name, action_dim, obs_dim):
		self.scene = None

		high = np.ones([action_dim])
		self.action_space = gym.spaces.Box(-high, high)
		high = np.inf*np.ones([obs_dim])
		self.observation_space = gym.spaces.Box(-high, high)
		self._seed()

		self.model_xml = model_xml
		self.robot_name = robot_name

		self.camera = Camera()

	def _seed(self, seed=None):
		self.np_random, seed = gym.utils.seeding.np_random(seed)
		return [seed]

	def getScene(self, bodies):
		parts = {}
		joints = {}
		ordered_joints = []
		robot_body = ""

		dump = 0
		for i in range(len(bodies)):
			for j in range(p.getNumJoints(bodies[i])):
				_,joint_name,_,_,_,_,_,_,_,_,_,_,part_name = p.getJointInfo(bodies[i], j)

				joint_name = joint_name.decode("utf8")
				part_name = part_name.decode("utf8")

				if dump: print("ROBOT PART '%s'" % part_name)
				if dump: print("ROBOT JOINT '%s'" % joint_name) # limits = %+0.2f..%+0.2f effort=%0.3f speed=%0.3f" % ((joint_name,) + j.limits()) )

				joints[joint_name] = Joint(joint_name, bodies, i, j)
				ordered_joints.append(joints[joint_name])

				parts[part_name] = BodyPart(part_name, bodies, i, j)

				if part_name == self.robot_name:
					robot_body = parts[part_name]

					if joint_name[:6] == "ignore":
						joints[joint_name].disable_motor()
						continue

				joints[joint_name].power_coef = 100.0

		return parts, joints, ordered_joints, robot_body

	def _reset(self):
		if self.scene is None:
			self.scene = self.create_single_player_scene()
		if not self.scene.multiplayer:
			self.scene.episode_restart()

		self.ordered_joints = []
		self.frame = 0
		self.done = 0
		self.reward = 0
		dump = 0

		self.parts, self.jdict, self.ordered_joints, self.robot_body = self.getScene(p.loadMJCF(os.path.join(os.path.dirname(__file__), "mujoco_assets", self.model_xml)))

		self.robot_specific_reset()
		s = self.calc_state()	# optimization: calc_state() can calculate something in self.* for calc_potential() to use
		self.potential = self.calc_potential()
		return s

	def _render(self, mode, close):
		pass

	def calc_potential(self):
		return 0

	def HUD(self, state, a, done):
		pass

class Camera:
	def __init__(self):
		pass

	def move_and_look_at(self,i,j,k,x,y,z):
		lookat = [x,y,z]
		distance = 10
		yaw = 10
		p.resetDebugVisualizerCamera(distance, yaw, -20, lookat)

class Pose_Helper: # dummy class to comply to original interface
	def __init__(self, body_part):
		self.body_part = body_part

	def xyz(self):
		return self.body_part.current_position()

	def rpy(self):
		return p.getEulerFromQuaternion(self.body_part.current_orientation())

class BodyPart:
	def __init__(self, body_name, bodies, bodyIndex, bodyPartIndex):
		self.bodies = bodies
		self.bodyIndex = bodyIndex
		self.bodyPartIndex = bodyPartIndex
		self.initialPosition = self.current_position()
		self.initialOrientation = self.current_orientation()
		self.bp_pose = Pose_Helper(self)

	def state_fields_of_pose_of(self, body_id, link_id=-1):  # a method you will most probably need a lot to get pose and orientation
		if link_id == -1:
			(x, y, z), (a, b, c, d) = p.getBasePositionAndOrientation(body_id)
		else:
			(x, y, z), (a, b, c, d), _, _, _, _ = p.getLinkState(body_id, link_id)
		return np.array([x, y, z, a, b, c, d])

	def get_pose(self):
		return self.state_fields_of_pose_of(self.bodies[self.bodyIndex], self.bodyPartIndex)

	def speed(self):
		(x,y,z), (a,b,c,d), _,_,_,_, (vx, vy, vz), (vr,vp,vy) = p.getLinkState(self.bodies[self.bodyIndex], self.bodyPartIndex, computeLinkVelocity=1)
		return np.array([vx,vy,vz])

	def current_position(self):
		return self.get_pose()[:3]

	def current_orientation(self):
		return self.get_pose()[3:]

	def reset_position(self, position):
		p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], position, self.get_orientation())

	def reset_orientation(self, orientation):
		p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], self.get_position(), orientation)

	def reset_pose(self, position, orientation):
		p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], position, orientation)

	def pose(self):
		return self.bp_pose


class Joint:
	def __init__(self, joint_name, bodies, bodyIndex, jointIndex):
		self.bodies = bodies
		self.bodyIndex = bodyIndex
		self.jointIndex = jointIndex
		_,_,_,_,_,_,_,_,self.lowerLimit, self.upperLimit,_,_,_ = p.getJointInfo(self.bodies[self.bodyIndex], self.jointIndex)
		self.power_coeff = 0

	def set_state(self, x, vx):
		p.resetJointState(self.bodies[self.bodyIndex], self.jointIndex, x, vx)

	def current_position(self): # just some synonyme method
		return self.get_state()

	def current_relative_position(self): # should we really calculate some kind of relative position?
		return self.get_state()

	def get_state(self):
		x, vx,_,_ = p.getJointState(self.bodies[self.bodyIndex],self.jointIndex)
		return x, vx

	def set_position(self, position):
		p.setJointMotorControl2(self.bodies[self.bodyIndex],self.jointIndex,p.POSITION_CONTROL, targetPosition=position)

	def set_velocity(self, velocity):
		p.setJointMotorControl2(self.bodies[self.bodyIndex],self.jointIndex,p.VELOCITY_CONTROL, targetVelocity=velocity)

	def set_motor_torque(self, torque): # just some synonyme method
		self.set_torque(torque)

	def set_torque(self, torque):
		p.setJointMotorControl2(self.bodies[self.bodyIndex],self.jointIndex,p.TORQUE_CONTROL, force=torque)

	def reset_current_position(self, position, velocity): # just some synonyme method
		self.reset_position(position, velocity)

	def reset_position(self, position, velocity):
		self.set_position(position)
		self.set_velocity(velocity)
		self.disable_motor()

	def disable_motor(self):
		p.setJointMotorControl2(self.bodies[self.bodyIndex],self.jointIndex,controlMode=p.VELOCITY_CONTROL, force=0)
