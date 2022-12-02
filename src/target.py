# target node

import rospy
import numpy as np
from rosdot.msg import Iteration

class TargetAgent:

    def __init__(self, num):

        self.target_num = num

        self.num_sources = rospy.get_param('source/num_agents')
        self.eta = rospy.get_param('/target/eta')
        self.num_targets = rospy.get_param('/target/num_agents')
        self.upper_bounds = rospy.get_param('/target/ub')
        
        self.target_pubs = list()

        for i in range(self.num_sources):
            self.target_pubs.append( rospy.Publisher("target%s_to_source%s"%(num,i), Iteration, queue_size = 1) )
            rospy.Subscriber("/source%s_to_target%s"%(i,num), Iteration, self.source_cb)
            
        self.iter = 0
        self.update = 0.0

        self.iter_msg = Iteration()
        self.iter_msg.node_id = num
        self.iter_msg.data = self.update
        self.iter_msg.k = self.iter
        self.target_pubs[0].publish(self.iter_msg)
        
        assert len( self.upper_bounds ) == self.num_targets

        self.cycle()
        
    def source_cb(self, msg):
        rospy.loginfo('heard message from source %s'%(msg.node_id))
        self.iter = msg.k
        self.update = msg.data

    def iterative_update(self):
        pass

    def cycle(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            for publisher in self.target_pubs:
                publisher.publish(self.iter_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("target", anonymous=True)
    uid = rospy.get_param(rospy.get_name()+"/uid")
    TargetAgent(int(uid))
