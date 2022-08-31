#! /usr/bin/python3

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import EmptyRequest
from nav_msgs.msg import Path
from catch_trash.srv import TargetPoint, TargetPointRequest, PursuitRequest
    
rospy.init_node("test_planner")

poses = []
planerSrv = rospy.ServiceProxy("/target_point_planner", TargetPoint)
pathPub = rospy.Publisher("test_path", Path, queue_size=1)

def getTwoPose(msg : PoseWithCovarianceStamped):
    global poses

    p = PoseStamped()
    p.pose = msg.pose.pose
    poses.append(p)
    print(f"get pose x: {p.pose.position.x}, y: {p.pose.position.y}")

    if len(poses) == 2:
        # call
        tar_req = TargetPointRequest()
        tar_req.first_path.poses.append(poses[0])
        tar_req.second_path.poses.append(poses[1])
        tar_req.first_path_name = "start"
        tar_req.second_path_name = "end"
        tar_req.map_name = "lvisam"
        print("about to call planner")
        resp = planerSrv.call(tar_req)
        print(f"path name is {resp.path_name}")
        pathPub.publish(resp.planner_path)
        # reset
        poses = []

rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, getTwoPose, queue_size=1)

rospy.logwarn("prepared!!")
rospy.spin()
