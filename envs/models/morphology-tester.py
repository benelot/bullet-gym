#!/usr/bin/env python

import pybullet as p
import time
import math

useRealTime = 0
fixedTimeStep = 0.001

physId = p.connect(p.SHARED_MEMORY)
if (physId<0):
	p.connect(p.GUI)

p.loadURDF("ground.urdf")
p.setGravity(0,0,-9.81)
p.setTimeStep(fixedTimeStep)

p.setRealTimeSimulation(0)
#body = p.loadURDF("simple-snakev0.urdf",1,-2,1)
#body = p.loadURDF("springy-snakev0.urdf",1,-2,1)
body = p.loadURDF("phantomx/phantomx.urdf",0,0,2)

position,orientation = p.getBasePositionAndOrientation(body)
print position

num_joints = p.getNumJoints(body)

minLimit = -math.pi/2
maxLimit = math.pi/2

p_gain =2
speed = 5

t = 0
while True:
	print math.sin(t*speed)/2*(maxLimit-minLimit)+minLimit
	for i in range(num_joints):
		p.setJointMotorControl2(body,i,p.POSITION_CONTROL,math.sin(t*speed)/2*(maxLimit-minLimit),p_gain)	
	p.stepSimulation()
	t = t + fixedTimeStep

	print p.getBasePositionAndOrientation(body)
	
	if t > 2:
		print "resetting position..."
		p.resetBasePositionAndOrientation(body, position, (0,0,0,1))
		
		print "resetting joint states..."
		for i in range(num_joints):
			p.resetJointState(body,i,0)
		t = 0

	print t
