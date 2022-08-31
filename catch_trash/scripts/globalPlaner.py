#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyRequest
from nav_msgs.msg import Path
from catch_trash.srv import TargetPoint, TargetPointRequest, PursuitRequest, Pursuit
from catch_trash.msg import PurePursuitResult
import tf.transformations
import math
# 1. wait for trash appear
# 2. get informed! call A* to get trajectory
# 3. follow the trajectory to a specific points
# 4. inform local perception node
# 5. get informed! call  A* to get back to trash station!!

def constrainAngle(x):
    while x < -math.pi:
        x += 2 * math.pi
    
    while x > math.pi:
        x -= 2 * math.pi

    return x

class globalPlaner:

    def __init__(self) -> None:
        
        self.informLP = True

        self.dst = None
        self.targetPos = None
        self.pos = None
        self.cmdVelPub = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size=1)
        self.informLPPub = rospy.ServiceProxy("/lp_work", Empty)
        self.targetSub = rospy.Subscriber("/target", PoseStamped, self.targetCB, queue_size=5)

        self.trackingSrv = rospy.ServiceProxy("/pursuit", Pursuit)
        self.trackingResSub = rospy.Subscriber("/pure_pursuit/tracking_result", PurePursuitResult, self.trackingResCB, queue_size=1)
        self.odomSub = rospy.Subscriber("/odom", Odometry, self.odomCB, queue_size=1)
        self.doorPub = rospy.Publisher("/rrbot/joint1_position_controller/command", Float64, queue_size=1)
        self.pathPub = rospy.Publisher("astar_path", Path, queue_size=1)
        self.planerSrv = rospy.ServiceProxy("/target_point_planner", TargetPoint)
        self.infoGenModel = rospy.ServiceProxy("/info_gen", Empty)

    def odomCB(self, msg:Odometry):
        rospy.logdebug_once("get odom!!")
        self.pos = PoseStamped()
        self.pos.pose = msg.pose.pose

    def targetCB(self, msg:PoseStamped):
        rospy.logwarn("gp: get target!!")

        # call planer
        tar_req = TargetPointRequest()
        tar_req.first_path.poses.append(self.pos)
        tar_req.second_path.poses.append(msg)
        tar_req.first_path_name = "start"
        tar_req.second_path_name = "end"
        tar_req.map_name = "lvisam"
        resp = self.planerSrv.call(tar_req)
        rospy.logwarn("gp: planner is called!!")
        self.pathPub.publish(resp.planner_path)

        # follow it
        pp_req = PursuitRequest()
        pp_req.command = 1
        pp_req.map = "lvisam"
        pp_req.path = resp.planner_path
        pp_req.mode = "path_tracking_with_path_data"
        resp = self.trackingSrv.call(pp_req)
        rospy.logwarn(f"gp: tracking start({resp.message})!!")
        

    def moveCar(self, x, z):

        tw = Twist()
        tw.linear.x = x
        tw.linear.y = 0
        tw.linear.z = 0
        tw.angular.x = 0
        tw.angular.y = 0
        tw.angular.z = z

        self.cmdVelPub.publish(tw)
        

    def dumpTrash(self):
        
        # open the door
        msg = Float64()
        msg.data = 0.0
        self.doorPub.publish(msg)

        rospy.logwarn("dumpTrash: adjusting its pose")
        pose = rospy.get_param("/robot_pose")
        rpy = tf.transformations.euler_from_quaternion(pose[3:])
        yaw = constrainAngle(rpy[2])
        while abs(yaw) > 0.07:
            if yaw < 0:
                self.moveCar(0, 0.2)
            else:
                self.moveCar(0, -0.2)
            rospy.sleep(rospy.Duration(0, 2e8))
            pose = rospy.get_param("/robot_pose")
            rpy = tf.transformations.euler_from_quaternion(pose[3:])
            yaw = constrainAngle(rpy[2])

        rospy.logwarn("dumpTrash: move backward")
        # move backward a little
        for i in range(30):
            self.moveCar(-1.0, 0)
            rospy.sleep(rospy.Duration(0, 2e8))
        
        pose = rospy.get_param("/robot_pose")
        rpy = tf.transformations.euler_from_quaternion(pose[3:])
        yaw = constrainAngle(rpy[2] + math.pi)
        while abs(yaw) > 0.15:
            if yaw < 0:
                self.moveCar(0, 0.6)
            else:
                self.moveCar(0, -0.6)
            rospy.sleep(rospy.Duration(0, 2e8))
            pose = rospy.get_param("/robot_pose")
            rpy = tf.transformations.euler_from_quaternion(pose[3:])
            yaw = constrainAngle(rpy[2] + math.pi)

        self.infoGenModel.call(EmptyRequest())
        rospy.logwarn("trash is dumped, info gen model!!")

    def trackingResCB(self, msg:PurePursuitResult):

        # inform LP or dump trash
        rospy.logwarn(f"get tracking result({msg.tracking_result})")
        if msg.tracking_result == "Success":
            if self.informLP:
                self.informLPPub.call(EmptyRequest())
                self.informLP = False

                rospy.logdebug("stop pure pursuit!!")
                pp_req = PursuitRequest()
                pp_req.command = 2
                pp_req.map = "lvisam"
                pp_req.mode = "path_tracking_with_path_data"
                resp = self.trackingSrv.call(pp_req)
                rospy.logdebug(f"gp: tracking start({resp.message})!!")

            else:
                self.dumpTrash()
                self.informLP = True
        

rospy.init_node("global_planer_node")
globalPlaner()

rospy.spin()