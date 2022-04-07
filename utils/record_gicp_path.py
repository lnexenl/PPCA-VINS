import rospy
import rospy_message_converter.json_message_converter as jmc
from nav_msgs.msg import Path
import json


def path_cb(msg: Path):
    json_msg = jmc.convert_ros_message_to_json(msg)
    with open('/home/lnex/path.json', 'w') as file:
        file.write(json_msg)


if __name__ == '__main__':
    rospy.init_node("asdf", anonymous=True)
    rospy.Subscriber("/ndt_path", Path, callback=path_cb, queue_size=1)
    rospy.spin()
