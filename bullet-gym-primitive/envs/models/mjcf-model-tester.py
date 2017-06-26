#!/usr/bin/env python

import pybullet as p
import time
import math
import numpy as np

useRealTime = 0
fixedTimeStep = 0.001

physId = p.connect(p.SHARED_MEMORY)
if (physId<0):
	p.connect(p.GUI)

p.loadURDF("ground.urdf")
p.setGravity(0,0,-9.81)
p.setTimeStep(fixedTimeStep)

p.setRealTimeSimulation(0)
# bodies = p.loadMJCF("mjcf/ant.xml")
# bodies = p.loadMJCF("mjcf/point.xml")
# bodies = p.loadMJCF("mjcf/half_cheetah.xml")
# bodies = p.loadMJCF("mjcf/humanoid.xml")
# bodies = p.loadMJCF("mjcf/humanoidstandup.xml")
# bodies = p.loadMJCF("mjcf/inverted_double_pendulum.xml")
#bodies = p.loadMJCF("mjcf/inverted_pendulum.xml")
# bodies = p.loadMJCF("mjcf/humanoid_symmetric.xml")
# bodies = p.loadMJCF("mjcf/hopper.xml")
bodies = p.loadMJCF("mjcf/reacher.xml")
# bodies = p.loadMJCF("mjcf/swimmer.xml")
#bodies = p.loadMJCF("mjcf/walker2d.xml")

p_gain = 2
speed = 1

#exclude = np.ones([len(bodies),50])
exclude = np.zeros([len(bodies),50])


exclude[0,1] = 0
exclude[1,5] = 1
exclude[1,8] = 0

for j in range(len(bodies)):
	for i in range(p.getNumJoints(bodies[j])):
		p.setJointMotorControl2(bodies[j],i,p.VELOCITY_CONTROL, force=0)
		_,name,_,_,_,_,_,_,minLimit,maxLimit,_,_,linkName = p.getJointInfo(bodies[j],i)
		print "Joint named ", name, "indices:",j, "," , i
		print "Link named" , linkName

print "\n"

print bodies

for x in range(0,p.getNumBodies(0)):
	print "Body: (",x,")", p.getBodyInfo(p.getBodyUniqueId(x))
	

t = 0
while True:
	#print math.sin(t*speed)/2*(maxLimit-minLimit)+minLimit
	for j in range(len(bodies)):
		for i in range(p.getNumJoints(bodies[j])):
			_,name,_,_,_,_,_,_,minLimit,maxLimit,_,_,_ = p.getJointInfo(bodies[j],i)
			if exclude[j,i] == 0:
				p.setJointMotorControl2(bodies[j],i,p.POSITION_CONTROL,math.sin(t*speed)/2*(maxLimit-minLimit),p_gain)
			#print p.getJointState(bodies[j],i)
	p.stepSimulation()
	t = t + fixedTimeStep

	#for j in range(len(bodies)):
		#print p.getBasePositionAndOrientation(bodies[j])

	#print t
