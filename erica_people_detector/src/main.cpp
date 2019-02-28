/*
 * main.cpp
 *
 *  Created on: Feb 7, 2019
 *      Author: jay
 */

#include "erica_people_detector/erica_people_detector.h"


//boost::mutex g_mutex;
//
//darknet_ros_msgs::BoundingBoxes g_detected_people_position_array;
//
//
//ros::Publisher g_robot_pose_pub;
//erica_perception_msgs::PeoplePositionArray g_people_position_msg;
//Eigen::Affine3d g_transfrom;
//bool g_get_transfrom = false;
//
//void getDetectedPeoplePixelPositionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
//{
//  g_mutex.lock();
//  g_detected_people_position_array.bounding_boxes.clear();
//
//  g_detected_people_position_array = *msg;
//  //g_detected_people_position_array.header.stamp = ros::Time::now();
//  g_mutex.unlock();
//  return;
//}
//
//void getZedPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
//{
//  g_mutex.lock();
//
//  g_people_position_msg.people_position.clear();
//  g_people_position_msg.box_size.clear();
//  g_people_position_msg.pixel_x.clear();
//  g_people_position_msg.pixel_y.clear();
//  g_people_position_msg.box_width.clear();
//  g_people_position_msg.box_height.clear();
//  g_people_position_msg.box_size.clear();
//
//  g_people_position_msg.img_width.data  = msg->width;
//  g_people_position_msg.img_height.data = msg->height;
//
//  if((g_detected_people_position_array.bounding_boxes.size() == 0)
//      || ((ros::Time::now().toSec() - g_detected_people_position_array.header.stamp.toSec()) > 1.0)
//      || (g_get_transfrom == false))
//  {
//    if(g_detected_people_position_array.bounding_boxes.size() == 0)
//      ROS_INFO("There is no people");
//    else if((msg->header.stamp.toSec() - g_detected_people_position_array.header.stamp.toSec()) > 1.0)
//      ROS_INFO_STREAM("Time Over: " << ros::Time::now().toSec() - g_detected_people_position_array.header.stamp.toSec()
//          << " " << ros::Time::now().toSec() << " " << g_detected_people_position_array.header.stamp.toSec());
//    else
//      ROS_INFO("theris_no_transform");
//
//    g_mutex.unlock();
//    g_robot_pose_pub.publish(g_people_position_msg);
//    return;
//  }
//
//
//  int width = msg->width;
//  int height = msg->height;
//
//  // Image coordinates of the center pixel
//  sensor_msgs::PointCloud cloud;
//  sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);
//
//  geometry_msgs::Point32 person;
//  std_msgs::Int32 size;
//  std_msgs::Int32 pixel_pos_x;
//  std_msgs::Int32 pixel_pos_y;
//  std_msgs::Int32 box_width;
//  std_msgs::Int32 box_height;
//  Eigen::Vector3d person_position;
//
//  for(int idx =0; idx < g_detected_people_position_array.bounding_boxes.size() ; idx++)
//  {
//    int u = (g_detected_people_position_array.bounding_boxes[idx].xmin + g_detected_people_position_array.bounding_boxes[idx].xmax)/2;
//    int v = (g_detected_people_position_array.bounding_boxes[idx].ymin + g_detected_people_position_array.bounding_boxes[idx].ymax)/2;
//
//    int area = (g_detected_people_position_array.bounding_boxes[idx].xmax - g_detected_people_position_array.bounding_boxes[idx].xmin )
//            * (g_detected_people_position_array.bounding_boxes[idx].ymax - g_detected_people_position_array.bounding_boxes[idx].ymin);
//
//    int depthIdx = u + width * v;
//    //    ROS_INFO_STREAM( idx <<" x: " << cloud.points[depthIdx].x
//    //        << " y: " << cloud.points[depthIdx].y
//    //        << " z: " << cloud.points[depthIdx].z
//    //        << " area : " << area);
//
//    person.x = cloud.points[depthIdx].x;
//    person.y = cloud.points[depthIdx].y;
//    person.z = cloud.points[depthIdx].z;
//    size.data = area;
//    box_width.data  = g_detected_people_position_array.bounding_boxes[idx].xmax - g_detected_people_position_array.bounding_boxes[idx].xmin;
//    box_height.data = g_detected_people_position_array.bounding_boxes[idx].ymax - g_detected_people_position_array.bounding_boxes[idx].ymin;
//
//    person_position.coeffRef(0) = person.x;
//    person_position.coeffRef(1) = person.y;
//    person_position.coeffRef(2) = person.z;
//
//    ROS_INFO_STREAM( idx <<" x: " << person.x
//        << " y: " << person.y
//        << " z: " << person.z
//        << " area : " << area);
//
//    person_position = g_transfrom.translation() + g_transfrom.rotation()*person_position;
//
////    ROS_INFO_STREAM(g_transfrom.translation().x()
////        <<" " << g_transfrom.translation().y()
////        <<" " << g_transfrom.translation().z()
////        <<" " << g_transfrom.rotation().coeff(0,0)
////        <<" " << g_transfrom.rotation().coeff(0,1)
////        <<" " << g_transfrom.rotation().coeff(0,2)
////        <<" " << g_transfrom.rotation().coeff(1,0)
////        <<" " << g_transfrom.rotation().coeff(1,1)
////        <<" " << g_transfrom.rotation().coeff(1,2)
////        <<" " << g_transfrom.rotation().coeff(2,0)
////        <<" " << g_transfrom.rotation().coeff(2,1)
////        <<" " << g_transfrom.rotation().coeff(2,2));
//
//    person.x = person_position.x();
//    person.y = person_position.y();
//    person.z = person_position.z();
//
//    pixel_pos_x.data =  u - width/2;
//    pixel_pos_y.data = -v + height/2;
//
//
//    ROS_INFO_STREAM( idx <<" x: " << person.x
//        << " y: " << person.y
//        << " z: " << person.z
//        << " area : " << area
//        << " X :" << pixel_pos_x.data
//        << " y :" << pixel_pos_y.data);
//
//
//    g_people_position_msg.people_position.push_back(person);
//    g_people_position_msg.box_size.push_back(size);
//    g_people_position_msg.box_width.push_back(box_width);
//    g_people_position_msg.box_height.push_back(box_height);
//    g_people_position_msg.pixel_x.push_back(pixel_pos_x);
//    g_people_position_msg.pixel_y.push_back(pixel_pos_y);
//  }
//
//  g_robot_pose_pub.publish(g_people_position_msg);
//  g_mutex.unlock();
//}
//
//
//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "erica_people_detector");
//  ros::NodeHandle nh;
//
//  ros::Subscriber detected_people_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, getDetectedPeoplePixelPositionCallback);
//  ros::Subscriber point_cloud_sub     = nh.subscribe("/zed/point_cloud/cloud_registered", 1, getZedPointCloudCallback);
//
//  g_robot_pose_pub = nh.advertise<erica_perception_msgs::PeoplePositionArray>("/erica/people_position", 1);
//
//  tf2_ros::Buffer tfBuffer;
//  tf2_ros::TransformListener tfListener(tfBuffer);
//
//  ros::Rate rate(30.0);
//  while(ros::ok())
//  {
//    geometry_msgs::TransformStamped transformStamped;
//    try{
//      //get transformation from body_main to zed_left_camera
//      transformStamped = tfBuffer.lookupTransform("body_main", "zed_left_camera_frame",
//          ros::Time(0));
//    }
//    catch (tf2::TransformException &ex) {
//      ROS_WARN("%s",ex.what());
//      g_get_transfrom = false;
//      ros::Duration(1.0).sleep();
//      continue;
//    }
//
//    g_get_transfrom = true;
//    g_mutex.lock();
//    g_transfrom = tf2::transformToEigen(transformStamped);
//    g_mutex.unlock();
//    ROS_INFO_STREAM("tf:" << transformStamped.transform.translation.x << " " << transformStamped.transform.translation.y << " " << transformStamped.transform.translation.z
//        <<" " << transformStamped.transform.rotation.x
//        <<" " << transformStamped.transform.rotation.y
//        <<" " << transformStamped.transform.rotation.z
//        <<" " << transformStamped.transform.rotation.w);
//    ros::spinOnce();
//    rate.sleep();
//  }
//
//  return 0;
//}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "erica_people_detector");
  ros::NodeHandle nh;

  erica::EricaPeopleDetecor erica_people_detector;
  erica_people_detector.pixel_distance_threshold_ = 50; //pixel coordinate
  erica_people_detector.min_size_  = 20000;
  erica_people_detector.refresh_time_threshold_ = 2.0;  //sec
  erica_people_detector.distance_threshold_ = 0.5; //meter
  erica_people_detector.max_distance_ = 5.0;
  erica_people_detector.near_ref_ = 0.9;


  erica_people_detector.initialize();


  double freq = 30.0;
  while(ros::ok())
  {
    if(ros::param::get("/zed/zed_wrapper_node/frame_rate", freq) == true)
    {
      ROS_INFO_STREAM("start detecting " << "[rate : " << freq << "]");
      break;
    }
    else
    {
      ROS_WARN("Check if zed_wrapper started");
      ros::Duration(1.0).sleep();
    }
  }

  ros::Rate rate(freq);
  while(ros::ok())
  {
    erica_people_detector.process2();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}


