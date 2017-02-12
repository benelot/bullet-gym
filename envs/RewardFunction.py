#!/usr/bin/env python
 
import pybullet as p
import numpy as np
 
def state_fields_of_pose_of(body_id, link_id=-1): # a method you will most probably need a lot to get pose and orientation
    if link_id == -1:
        (x,y,z), (a,b,c,d) = p.getBasePositionAndOrientation(body_id)
    else:
        (x,y,z), (a,b,c,d),_,_,_,_ = p.getLinkState(body_id, link_id)
    
    return (x,y,z),(a,b,c,d)
 
class RewardFunction():
    VelocityReward = 0
    PositionReward = 1
    
    XAxis = 2
    YAxis = 3
    ZAxis = 5
 
 
    def __init__(self, body_id, rewardType, config):
        self.rewardType = rewardType
        self.body_id = body_id
        self.config = config
        self.linkQty = p.getNumJoints(self.body_id) # the reward function will be calculated across n links

        if(self.rewardType == self.VelocityReward):
            self.lastPosition = np.zeros((self.linkQty+1,3))
            
            for link_id in range(-1, self.linkQty):
                self.lastPosition[link_id+1], orient =  state_fields_of_pose_of(self.body_id, link_id)

    def getReward(self):
        if(self.rewardType == self.VelocityReward):
            return self.getVelocityReward()
        
        if(self.rewardType == self.PosititionReward):
            return self.getPositionReward()

    def getVelocityReward(self):
        avgLinkVelocity = 0
 
        for link_id in range(-1,self.linkQty):
            (x, y, z),orient = state_fields_of_pose_of(self.body_id, link_id)
            
            sqLength = 0
            if self.config % self.XAxis == 0:
                sqLength += np.square(x-self.lastPosition[link_id+1][0])
                
            if self.config % self.YAxis == 0:
                sqLength += np.square(y-self.lastPosition[link_id+1][1])
            
            if self.config % self.ZAxis == 0:
                sqLength += np.square(z-self.lastPosition[link_id+1][2])
                
            avgLinkVelocity += np.sqrt(sqLength)

        avgLinkVelocity /= self.linkQty
        return avgLinkVelocity
    
    def getPositionReward(self):
        avgLinkPosition = 0
 
        for link_id in range(-1,self.linkQty):
            (x, y, z),_,_,_,_,_ = state_fields_of_pose_of(self.body_id, link_id)
            
            sqLength = 0
            if self.config % self.XAxis == 0:
                sqLength += np.square(x)
                
            if self.config % self.YAxis == 0:
                sqLength += np.square(y)
            
            if self.config % self.ZAxis == 0:
                sqLength += np.square(z)
                
            avgLinkPosition += np.sqrt(sqLength)

        avgLinkPosition /= self.linkQty
        return avgLinkPosition