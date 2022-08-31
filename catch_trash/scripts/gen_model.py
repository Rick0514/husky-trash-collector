#! /usr/bin/python3
import math
import rospy
import random
import cv2
import numpy as np
import rospy, tf.transformations
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest, GetModelState, GetModelStateRequest, GetModelStateResponse
from geometry_msgs.msg import *
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from nav_msgs.msg import Odometry

model_dir = "/home/hgy/hgy/trash/"
map_file = "/home/hgy/.robot/data/maps/lvisam/lvisam.png"
print("read map...")
map_data = cv2.imread(map_file, 0)

ld_w = np.array([-49.430786, -49.828362])
resl = 0.1
map_width = 1040
map_height = 1002
sxzy = np.array([[-12, 34], [18, 34], [-12, -20], [18, -20]])
model_index = 0
balls_coord = None
balls_name = None
get_state = None
car_odom = None
ball_max_id = 12

boundary_x = 0
boundary_y = 0

target_pub = None

def worldToMap(x, y):
    global ld_w, map_width, map_height

    u = int((x - ld_w[0]) / resl)
    v = int(map_height - (y - ld_w[1]) / resl)

    u = max(0, min(u, map_width))
    v = max(0, min(v, map_height))

    return u, v

def genValidCoord(x_range, y_range, cnt):
    global map_data

    if cnt > 50:
        rospy.logerr("recurse too deep !!")
        return -99, -99

    r_x = random.uniform(x_range[0], x_range[1])
    r_y = random.uniform(y_range[0], y_range[1])
    r_u, r_v = worldToMap(r_x, r_y)

    if map_data[r_v][r_u] > 200:
        for i in range(-10, 11):
            v = max(0, min(r_v + i, map_height))
            for j in range(-10, 11):
                u = max(0, min(r_u + j, map_width))
                if map_data[v][u] < 100:
                    return genValidCoord(x_range, y_range, cnt+1)
    else:
        return genValidCoord(x_range, y_range, cnt+1)

    return r_x, r_y


def infoCB(req):
    global model_index, balls_coord, target_pub

    selected_xy = balls_coord[model_index]

    # 1. check the ball is valid
    if selected_xy[0] == -99 or selected_xy[1] == -99:
        rospy.logerr("ball coord is not valid!!")
        return EmptyResponse()

    # 2. find valid direction to get ball
    r = 2
    fx = None
    fy = None
    for e in np.linspace(0, 2*3.1416, 20):
        wx = selected_xy[0] + r * math.cos(e)
        wy = selected_xy[1] + r * math.sin(e)
        r_u, r_v = worldToMap(wx, wy)
        check = True
        for i in range(-10, 11):
            v = max(0, min(r_v + i, map_height))
            for j in range(-10, 11):
                u = max(0, min(r_u + j, map_width))
                if map_data[v][u] < 100:
                    check = False
                    break
            if not check:
                break

        if check:
            fx = wx
            fy = wy
            break

    if fx is None:
        rospy.logerr("infoCB: fx, fy is not generated!!")
        return EmptyResponse()

    # 3. pub the target !!
    msg = PoseStamped()
    msg.pose.position.x = fx
    msg.pose.position.y = fy
    msg.pose.position.z = 0

    diff_x = fx - car_odom.position.x
    diff_y = fy - car_odom.position.y
    msg.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, math.atan2(diff_y, diff_x)))

    target_pub.publish(msg)

    model_index += 1
    model_index %= 12

    return EmptyResponse()


def odomCB(msg : Odometry):
    global car_odom
    car_odom = msg.pose.pose
    pose = [car_odom.position.x, car_odom.position.y, car_odom.position.z, car_odom.orientation.x, car_odom.orientation.y, car_odom.orientation.z, car_odom.orientation.w]
    rospy.set_param("/robot_pose", pose)

def checkBall(event):
    rospy.logdebug("checking ball!!")
    global get_state, balls_coord, balls_name, boundary_x, boundary_y, ball_max_id

    for i in range(12):
        xy = balls_coord[i]

        if xy[0] == -99 or xy[1] == -99:
            continue

        # get ball's state
        req = GetModelStateRequest()
        req.model_name = balls_name[i]
        resp = get_state.call(req)   

        axy = np.array([26.7, 18.0])
        bxy = np.array([resp.pose.position.x, resp.pose.position.y], dtype=np.float)

        if np.linalg.norm(bxy - axy) < 3.0:
            rospy.logwarn(f"ball{i} is dumped, regenerate it!!")
            # respawn ball
            a_i = i // 4
            a_j = i % 4
            x, y = genValidCoord([boundary_x[a_i], boundary_x[a_i+1]], [boundary_y[a_j], boundary_y[a_j+1]], 0)
            if x != -99 and y != -99:
                balls_coord[i] = np.array([x, y])
                item_name = f"ball{ball_max_id}"
                ball_max_id += 1
                orient = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
                item_pose = Pose(Point(x=x, y=y, z=1.0),  orient)
                balls_name[i] = item_name
                rospy.logwarn("checkBall --> Spawning model: %s", item_name)
                spawn_model(item_name, product_xml, "", item_pose, "world")
            else:
                rospy.logwarn("checkBall --> model: %s cant not spawn!!", item_name)



if __name__ == '__main__':

    print("Waiting for gazebo services...")
    rospy.init_node("spawn_model")
    # rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Got it.")
    # delete_model = rospy.ServicePoxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    target_pub = rospy.Publisher("/target", PoseStamped, queue_size=1)
    odomSub = rospy.Subscriber('/odom', Odometry, odomCB, queue_size=1)
    infoSrv = rospy.Service("/info_gen", Empty, infoCB)
    spawnBallTimer = rospy.Timer(rospy.Duration(5), checkBall)


    with open(model_dir + "red_ball/model.sdf", "r") as f:
        product_xml = f.read()


    # 3 block x, 4 block y
    boundary_x = np.linspace(-12, 18, 4)
    boundary_y = np.linspace(-20, 34, 5)

    # 1. firstly, generate 12 balls
    balls_coord = np.zeros((12, 2), dtype=np.float)
    balls_name = []
    # i = j = 0

    for i in range(3):
        for j in range(4):
            x, y = genValidCoord([boundary_x[i], boundary_x[i+1]], [boundary_y[j], boundary_y[j+1]], 0)
            print(f"gen coord x({x}), y({y})")
            balls_coord[i*4 + j, :] = np.array([x, y])
            item_name = f"ball{i*4 + j}"
            orient = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
            item_pose = Pose(Point(x=x, y=y, z=1.0),  orient)
            balls_name.append(item_name)
            if x != -99 and y != -99:
                rospy.logwarn("Spawning model: %s", item_name)
                spawn_model(item_name, product_xml, "", item_pose, "world")
            else:
                rospy.logwarn("model: %s cant not spawn!!", item_name)


    infoCB(EmptyRequest())

    rospy.spin()



          # print(tf.transformations.quaternion_from_euler(0,0,0))
    # orient = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))

    # for num in xrange(0,12):
    #     item_name = "product_{0}_0".format(num)
    #     print("Deleting model:%s", item_name)
    #     delete_model(item_name)


        # req = SpawnModelRequest()
        # req.model_name = item_name
        # req.initial_pose = item_pose
        # req.reference_frame = "world"
        # req.robot_namespace = ""
        # req.model_xml = product_xml
        # spawn_model.call(req)