import rospy
from rosdot.target import TargetAgent
from rosdot.source import SourceAgent

if __name__ == '__main__':

    rospy.init_node('sim')

    num_targets = rospy.get_param('/target/num_agents')
    num_sources = rospy.get_param('/source/num_agents')

    for i in range(num_targets):
        TargetAgent(i)

    for i in range(num_sources):
        SourceAgent(i)
