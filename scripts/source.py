#!/usr/bin/env python3

import rospy
import numpy as np

from rosdot.msg import Iteration
from rosdot.optimizer import optimize

class SourceAgent:

    def __init__(self, num, namespace):

        self.source_num = num
        self.rate = rospy.Rate(10)
        
        self.num_targets = rospy.get_param('target/num_agents')
        self.eta = rospy.get_param('/source/eta')
        self.num_sources = rospy.get_param('/source/num_agents')
        self.node_ub = rospy.get_param(namespace+'/ub')
        self.max_iter = rospy.get_param('/max_iter')
        self.cost = -1 * np.ones(self.num_targets)
        
        self.target_ubs = np.ones(self.num_targets)
        self.pi = np.ones(self.num_targets)
        
        self.source_pubs = list()
        
        for i in range(self.num_targets):
            self.source_pubs.append( rospy.Publisher('source%s_to_target%s'%(num,i), Iteration, queue_size = 10) )
            rospy.Subscriber('/target%s_to_source%s'%(i,num), Iteration, self.target_cb)
            self.source_pubs[i].publish(Iteration())
        
        self.node_tracker = np.ones(self.num_targets, dtype=bool)
        self.target_ubs = np.zeros(self.num_targets)

        self.w = np.ones(self.num_targets)
        self.pi = np.ones(self.num_targets)

        self.iter = 0
            
        self.iter_msg = Iteration()

        self.iter_msg.node_id = num
        #assert len( self.upper_bounds ) == self.num_sources

        for _ in range(1):
            self.initial_publish()
            self.rate.sleep()
            
        self.cycle()

        
    def target_cb(self, msg):
        self.update_params(msg)


    def update_params(self, msg):
        self.node_tracker[msg.node_id] = True
        self.target_ubs[msg.node_id] = msg.upper_bound
        self.pi[msg.node_id] = msg.data
        

    def iterative_update(self):
        update, self.w = optimize(np.ones(self.num_targets), self.pi, self.cost, self.w, self.eta, self.target_ubs, self.node_ub)
        self.pi = 0.5 * (update + self.pi) 
        self.node_tracker = np.zeros(self.num_targets, dtype=bool)
        print(self.pi)
        
    def publish_msg(self):
        self.iter_msg.node_id = self.source_num
        self.iter_msg.upper_bound = self.node_ub

        for i,pub in enumerate(self.source_pubs):
            self.iter_msg.data = self.pi[i]
            pub.publish(self.iter_msg)

            
    def initial_publish(self):
        self.iter_msg.node_id = self.source_num
        self.iter_msg.upper_bound = self.node_ub

        for i,pub in enumerate(self.source_pubs):
            self.iter_msg.data = 0.0
            pub.publish(self.iter_msg)
        
        
    def cycle(self):

        #rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #print(self.node_tracker)
            if np.sum(self.node_tracker) == self.num_targets:
                self.iterative_update()
                self.publish_msg()
            #else:
            #    self.publish_msg()
            self.rate.sleep()

            
if __name__ == "__main__":
    rospy.init_node("source", anonymous=True)
    uid = rospy.get_param(rospy.get_name()+"/uid")
    SourceAgent(int(uid),rospy.get_name())
