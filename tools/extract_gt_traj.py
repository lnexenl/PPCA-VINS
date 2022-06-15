import rosbag
import ros
import numpy as np
import sys

if __name__ == '__main__':
    bagfile = sys.argv[1]
    gtfile = sys.argv[2]
    
    bag = rosbag.Bag(bagfile, 'r')
    with open(gtfile, 'w') as f:
        for topic, msg, t in bag.read_messages(topics=['/airsim_node/uav/odom_local_ned']):
            print(t)
            f.write('{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n'.format(
                msg.header.stamp.to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
            
    bag.close()
