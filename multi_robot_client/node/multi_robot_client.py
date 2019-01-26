#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import json
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import paho.mqtt.client as mqtt

def main():
  rospy.init_node("multi_robot_client")
  robot_id_ = rospy.get_param("robot_id", default="iqr_1")
  frame_id_ = rospy.get_param("frame_id", default="map")
  sub_robotpose_name_ = rospy.get_param("robotpose_name", default="amcl_pose")
  pub_goalpose_name_ = rospy.get_param("goalpose_name", default="move_base_simple/goal")
  pub_initialpose_name_ = rospy.get_param("initialpose_name", default="initialpose")
  mqtt_server_ip_ = rospy.get_param("mqtt_server_ip", default="127.0.0.1")
  mqtt_sub_name_ = rospy.get_param("mqtt_sub_name", default="goal_pose")
  mqtt_pub_name_ = rospy.get_param("mqtt_pub_name", default="robot_pose")

  def poseSubCallBack(msg):
    pos_ = dict(x=msg.pose.position.x, y=msg.pose.position.y, z=msg.pose.position.z)
    ori_ = dict(x=msg.pose.orientation.x,y=msg.pose.orientation.y,z=msg.pose.orientation.z,w=msg.pose.orientation.w)
    data_ = dict(robot_id=robot_id_, position=pos_, orientation=ori_)
    payload_ = json.dumps(data_)
    client.publish(mqtt_pub_name_, payload=payload_, qos=0)
    rospy.loginfo("mqtt publish: "+str(payload_))

  robotpose_sub_ = rospy.Subscriber(sub_robotpose_name_, PoseStamped, poseSubCallBack)
  goalpose_pub_ = rospy.Publisher(pub_goalpose_name_, PoseStamped, queue_size=1)
  initialpose_pub_ = rospy.Publisher(pub_initialpose_name_, PoseWithCovarianceStamped, queue_size=1)

  def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

  def on_disconnect(client, userdata, rc):
    print("DisConnected with result code "+str(rc))

  def on_message(client, userdata, msg):
    rospy.loginfo("mqtt listen: "+ msg.topic+" "+str(msg.payload))
    data_ = json.loads(msg.payload)
    if data_["message_type"] == pub_goalpose_name_ and data_["robot_id"] == robot_id_:
      goal_pose_ = PoseStamped()
      goal_pose_.header.stamp = rospy.Time.now()
      goal_pose_.header.frame_id = frame_id_
      goal_pose_.pose.position.x = data_["position"]["x"]
      goal_pose_.pose.position.y = data_["position"]["y"]
      goal_pose_.pose.position.z = data_["position"]["z"]
      goal_pose_.pose.orientation.x = data_["orientation"]["x"]
      goal_pose_.pose.orientation.y = data_["orientation"]["y"]
      goal_pose_.pose.orientation.z = data_["orientation"]["z"]
      goal_pose_.pose.orientation.w = data_["orientation"]["w"]
      goalpose_pub_.publish(goal_pose_)
    if data_["message_type"] == pub_initialpose_name_ and data_["robot_id"] == robot_id_:
      initial_pose_ = PoseWithCovarianceStamped()
      initial_pose_.header.stamp = rospy.Time.now()
      initial_pose_.header.frame_id = frame_id_
      initial_pose_.pose.pose.position.x = data_["position"]["x"]
      initial_pose_.pose.pose.position.y = data_["position"]["y"]
      initial_pose_.pose.pose.position.z = data_["position"]["z"]
      initial_pose_.pose.pose.orientation.x = data_["orientation"]["x"]
      initial_pose_.pose.pose.orientation.y = data_["orientation"]["y"]
      initial_pose_.pose.pose.orientation.z = data_["orientation"]["z"]
      initial_pose_.pose.pose.orientation.w = data_["orientation"]["w"]
      initial_pose_.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
      initialpose_pub_.publish(initial_pose_)

  client = mqtt.Client(robot_id_)
  client.on_connect = on_connect
  client.on_message = on_message
  client.on_disconnect = on_disconnect
  client.connect(mqtt_server_ip_, 1883, 10)
  client.subscribe(mqtt_sub_name_,qos=0)
  client.loop_start()
  rospy.spin()


if __name__ == "__main__":
  main()