#!/usr/bin/env python
import rospy
import rosnode
from std_msgs.msg import Bool

def callback(data):

    #if we get a False, we've finished the mission, can kill all sim-related nodes
    if not data.data:
        print "Simulation has finished! Killing nodes."
        rosnode.kill_nodes(rosnode.get_node_names())

    
def simulationKiller():

    rospy.init_node('simulationKiller', anonymous=True)

    #/rexrov/trajectory_tacking_on returns false if/when dp_controller doesn't have target waypoints remaing
    rospy.Subscriber("/rexrov/trajectory_tracking_on", Bool, callback)

    rospy.spin()

if __name__ == '__main__':
    simulationKiller()