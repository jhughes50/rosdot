
import rospy
import numpy as np

from rosdot.msg import Iteration


class SourceAgent:

    def __init__(self, num):

        self.source_num = num

        self.num_targets = rospy.get_param('target/num_agents')
        self.eta = rospy.get_param('/source/eta')
        self.num_sources = rospy.get_param('/source/num_agents')
        self.upper_bounds = rospy.get_param('/source/ub')

        self.source_pubs = list()
        
        for i in range(self.num_targets):
            self.source_pubs.append( rospy.Publisher('source%s_to_target%s'%(num,i), Iteration, queue_size = 10) )
            rospy.Subscriber('/target%s_to_source%s'%(i,num), Iteration, self.target_cb)
            self.source_pubs[i].publish(Iteration())

            
        self.iter_msg = Iteration()

        self.iter_msg.node_id = num
        assert len( self.upper_bounds ) == self.num_sources
        
        self.cycle()
        
    def target_cb(self, msg):
        rospy.loginfo('heard message from target %s'%(msg.node_id))


    def cycle(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            for publisher in self.source_pubs:
                publisher.publish(self.iter_msg)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("source", anonymous=True)
    uid = rospy.get_param(rospy.get_name()+"/uid")
    SourceAgent(int(uid))
