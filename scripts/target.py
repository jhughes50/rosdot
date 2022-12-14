#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray
from rosdot.msg import Iteration
from rosdot.optimizer import optimize  

class TargetAgent:

    def __init__(self, num, namespace):

        self.target_num = num
        self.rate = rospy.Rate(10)

        self.convergence = False
        
        self.num_sources = rospy.get_param('source/num_agents')
        self.eta = rospy.get_param('/target/eta')
        self.num_targets = rospy.get_param('/target/num_agents')
        self.node_ub = rospy.get_param(namespace+'/ub')
        self.max_iter = rospy.get_param('/max_iter')
        self.cost = -2 * np.ones(self.num_sources)
        
        self.sys_ubs = np.ones(self.num_sources)
        self.sys_data = np.ones(self.num_sources)
        
        self.target_pubs = list()

        for i in range(self.num_sources):
            self.target_pubs.append( rospy.Publisher("target%s_to_source%s"%(num,i), Iteration, queue_size = 1) )
            rospy.Subscriber("/source%s_to_target%s"%(i,num), Iteration, self.source_cb)

        rospy.Subscriber("/source_costs", Float64MultiArray, self.cost_cb)
            
        self.node_tracker = np.ones(self.num_sources, dtype=bool)
        self.source_ubs = np.ones(self.num_sources)

        self.w = np.ones(self.num_sources)
        self.pi = np.ones(self.num_sources)
        
        self.iter = 0
        self.update = 0.0

        self.iter_msg = Iteration()
        self.iter_msg.node_id = num

        for _ in range(1):
           self.initial_publish()
           self.rate.sleep()
           
        self.cycle()
        
    def source_cb(self, msg):
        self.update_params(msg)


    def cost_cb(self, msg):
        self.cost = np.asarray(msg.data)
        

    def iterative_update(self):
        update, self.w = optimize(np.ones(self.num_sources), self.pi, self.cost, self.w, self.eta, self.source_ubs, self.node_ub)
        
        self.pi = 0.5 * (update + self.pi)
        self.node_tracker = np.zeros(self.num_sources, dtype=bool)

        
    def update_params(self, msg):
        self.node_tracker[msg.node_id] = True
        self.source_ubs[msg.node_id] = msg.upper_bound
        self.pi[msg.node_id] = msg.data

        
    def publish_msg(self):
        self.iter_msg.node_id = self.target_num
        self.iter_msg.upper_bound = self.node_ub
        self.iter_msg.k = self.iter
        
        for i,pub in enumerate(self.target_pubs):
            self.iter_msg.data = self.pi[i]
            pub.publish(self.iter_msg)

        self.iter += 1
        
            
    def initial_publish(self):
        
        self.iter_msg.node_id = self.target_num
        self.iter_msg.upper_bound = self.node_ub

        for i,pub in enumerate(self.target_pubs):
            self.iter_msg.data = 0.0
            pub.publish(self.iter_msg)

            
    def cycle(self):

        while not rospy.is_shutdown():
            
            if np.sum(self.node_tracker) == self.num_sources and self.convergence == False:
                self.iterative_update()
                self.publish_msg()

            if self.iter == self.max_iter:
                self.convergence == True
                rospy.loginfo("Target node %s converged"%self.target_num)
            
            self.rate.sleep()

            
if __name__ == "__main__":
    rospy.init_node("target", anonymous=True)
    uid = rospy.get_param(rospy.get_name()+"/uid")
    TargetAgent(int(uid),rospy.get_name())
