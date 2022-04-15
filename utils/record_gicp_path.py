import rospy
import rospy_message_converter.json_message_converter as jmc
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import json

ndt_path = Path()


# def ndt_cb(msg: PoseWithCovarianceStamped):
#     tmp_pose = PoseStamped()
#     tmp_pose.header = msg.header
#     tmp_pose.pose = msg.pose.pose
#     ndt_path.poses.append(tmp_pose)
#     ndt_path.header = msg.header
#     json_msg = jmc.convert_ros_message_to_json(ndt_path)
#     with open('/home/lnex/ndt_path.json', 'w') as file:
#         file.write(json_msg)

def gt_cb(msg: Path):
    json_msg = jmc.convert_ros_message_to_json(msg)
    with open('/home/lnex/path/gt.json', 'w') as file:
        file.write(json_msg)


def vins_cb(msg: Path):
    json_msg = jmc.convert_ros_message_to_json(msg)
    with open('/home/lnex/path/stamped_traj_estimate.json', 'w') as file:
        file.write(json_msg)


def global_cb(msg: Path):
    json_msg = jmc.convert_ros_message_to_json(msg)
    with open('/home/lnex/path/stamped_prior_estimate.json', 'w') as file:
        file.write(json_msg)


if __name__ == '__main__':
    rospy.init_node("asdf", anonymous=True)
    # rospy.Subscriber("/vins_estimator/gt_path", Path, callback=gt_cb, queue_size=1)
    rospy.Subscriber("/vins_estimator/path", Path, callback=vins_cb, queue_size=1)
    rospy.Subscriber("/priorOptNode/global_path", Path, callback=global_cb, queue_size=1)
    rospy.spin()
