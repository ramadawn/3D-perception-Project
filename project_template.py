#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data

    cloud = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering

    fil = cloud.make_statistical_outlier_filter()
    fil.set_mean_k(20)
    fil.set_std_dev_mul_thresh(0.3)
    filtered_cloud = fil.filter()

    # TODO: Voxel Grid Downsampling

    vox = filtered_cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_vox = vox.filter()

    # TODO: PassThrough Filter

    passthrough = cloud_vox.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.3
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_passthrough = passthrough.filter()
    passthrough = cloud_passthrough.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_passthrough = passthrough.filter()

    # TODO: RANSAC Plane Segmentation

    segment = cloud_passthrough.make_segmenter()
    segment.set_model_type(pcl.SACMODEL_PLANE)
    segment.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    segment.set_distance_threshold(max_distance)
    inliers, coefficients = segment.segment()

    # TODO: Extract inliers and outliers

    cloud_table = cloud_passthrough.extract(inliers, negative=False)
    cloud_objects = cloud_passthrough.extract(inliers, negative=True)

    # TODO: Euclidean Clustering

    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    excluster = white_cloud.make_EuclideanClusterExtraction()
    excluster.set_ClusterTolerance(0.015)
    excluster.set_MinClusterSize(20)
    excluster.set_MaxClusterSize(3000)
    excluster.set_SearchMethod(tree)
    cluster_indices = excluster.Extract()
    cluster_colour = get_color_list(len(cluster_indices))
    colour_cluster_point_list = []

    for a, indices in enumerate(cluster_indices):
        for b, indice in enumerate(indices):
            colour_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_colour[a])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(colour_cluster_point_list)

    # TODO: Convert PCL data to ROS messages

    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages

    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster

        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        colour_histogram = compute_color_histograms(ros_cluster, using_hsv=True)
        normal = get_normals(ros_cluster)
        normal_histogram = compute_normal_histograms(normal)
        features = np.concatenate((colour_histogram, normal_histogram))

        # Make the prediction

        prediction = clf.predict(scaler.transform(features.reshape(1,-1)))
        labels = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(labels)

        # Publish a label into RViz

        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(labels,label_pos, index))

        # Add the detected object to the list of detected objects.

        detect_object = DetectedObject()
        detect_object.label = labels
        detect_object.cloud = ros_cluster
        detected_objects.append(detect_object)

    # Publish the list of detected objects

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    dict_list = []
    centroids = []

    # TODO: Get/Read parameters

    object_list_par = rospy.get_param('/object_list')
    dropbox_par = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables

    dict_dropbox = {}
    for par in dropbox_param:
        dict_dropbox[par['name']] = par['position']

    # TODO: Loop through the pick list

    for obj in object_list_par:

    # TODO: Get the PointCloud for a given object and obtain it's centroid

        object_name = String()
        object_name.data = obj['name']

        pick_pose = Pose()
        pick_pose.position.x = 0
        pick_pose.position.y = 0
        pick_pose.position.z = 0

        pick_pose.orientation.x = 0
        pick_pose.orientation.y = 0
        pick_pose.orientation.z = 0
        pick_pose.orientation.w = 0

        place_pose = Pose()
        place_pose.orientation.x = 0
        place_pose.orientation.y = 0
        place_pose.orientation.z = 0
        place_pose.orientation.w = 0

        for detected_object in object_list:
            if detected_object.label == object_name.data:

                # TODO: Create 'place_pose' for the object

                points_arr = ros_to_pcl(detected_object.cloud).to_array()
                pick_pose_np = np.mean(points_arr, axis=0)[:3]
                pick_pose.position.x = np.asscalar(pick_pose_np[0])
                pick_pose.position.y = np.asscalar(pick_pose_np[1])
                pick_pose.position.z = np.asscalar(pick_pose_np[2])

        # TODO: Assign the arm to be used for pick_place

        if obj['group'] == 'red':
            arm_name.data = 'left'

        elif obj['group'] == 'green':
            arm_name.data = 'right'

        else:
            print "ERROR, group must be green or red!"

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        test_scene_num = Int32()
        test_scene_num.data = 3

        place_pose.position.x = dict_dropbox[arm_name.data][0]
        place_pose.position.y = dict_dropbox[arm_name.data][1]
        place_pose.position.z = dict_dropbox[arm_name.data][2]
        dict_list.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file

        yaml_filename = "output_" + str(test_scene_num.data) + ".yaml"



if __name__ == '__main__':

    # TODO: ROS node initialization

    rospy.init_node('object_recognition', anonymous=True)

    # TODO: Create Subscribers

    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size = 1)

    # TODO: Create Publishers

    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)

    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size = 1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size = 1)

    # TODO: Load Model From disk

    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list

    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown

    while not rospy.is_shutdown():
        rospy.spin()
