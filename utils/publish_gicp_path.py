import rospy
import os
from nav_msgs.msg import Path
from rospy_message_converter.json_message_converter import convert_json_to_ros_message as j2r

if __name__ == '__main__':
    rospy.init_node("gicp_publish", anonymous=True)
    gicp_pub = rospy.Publisher("/gicp_path", Path, queue_size=1)

    with open("{}/{}".format(os.path.abspath(os.path.curdir), "path.json"), 'r') as f:
        path = j2r("nav_msgs/Path", f.read(), strict_mode=False)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        gicp_pub.publish(path)
        r.sleep()
