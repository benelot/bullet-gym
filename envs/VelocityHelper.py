#!/usr/bin/env python

import pybullet as p
import numpy as np

class VelocityHelper():
    
    def __init__(self,body_id):
        self.body_id = body_id
        self.num_links = p.getNumJoints(self.body_id)
        
        self.pose = np.zeros((self.num_links+1,7))
        
        self.velocity = np.zeros((self.num_links+1,6))
        
        self.update()
        
    def state_fields_of_pose_of(self, link_id=-1): # a method you will most probably need a lot to get pose and orientation
        if link_id == -1:
            (x,y,z), (a,b,c,d) = p.getBasePositionAndOrientation(self.body_id)
        else:
            (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(self.body_id, link_id)
        return np.array([x,y,z,a,b,c,d])
        
    def update(self):
        linkPose = np.zeros(7)
        for i in range(-1,self.num_links):
            linkPose = self.state_fields_of_pose_of(i)
            vp = linkPose[:3] - self.pose[i][:3]
            av = p.getEulerFromQuaternion(linkPose[3:])
            oav = p.getEulerFromQuaternion(self.pose[i][3:])
            vo = np.subtract(av,oav)
            self.velocity[i+1] = np.concatenate((vp,vo),axis=0)
            self.pose[i+1] = linkPose
            
    def getVelocities(self):
        return self.velocity