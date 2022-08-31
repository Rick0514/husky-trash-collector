#! /usr/bin/python3

import rospy

import rospy, tf.transformations
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest
from geometry_msgs.msg import *

model_dir = "/home/hgy/hgy/trash/"

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_model")
    # rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Got it.")
    # delete_model = rospy.ServicePoxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    with open(model_dir + "red_ball/model.sdf", "r") as f:
        product_xml = f.read()

    print(tf.transformations.quaternion_from_euler(0,0,0))

    orient = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))

    # for num in xrange(0,12):
    #     item_name = "product_{0}_0".format(num)
    #     print("Deleting model:%s", item_name)
    #     delete_model(item_name)

    for num in range(0,12):
        bin_y   =   2.8 *   (num    /   6)  -   1.4 
        bin_x   =   0.5 *   (num    %   6)  -   1.5
        item_name   =  f"ball_{num}"
        print("Spawning model:%s", item_name)

        item_pose   =   Pose(Point(x=bin_x, y=bin_y, z=2),  orient)
        # req = SpawnModelRequest()
        # req.model_name = item_name
        # req.initial_pose = item_pose
        # req.reference_frame = "world"
        # req.robot_namespace = ""
        # req.model_xml = product_xml
        # spawn_model.call(req)

        spawn_model(item_name, product_xml, "", item_pose, "world")
