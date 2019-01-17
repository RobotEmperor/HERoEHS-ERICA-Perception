/*
 * erica_people_detector.cpp
 *
 *  Created on: Jan 9, 2019
 *      Author: jay
 */


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/thread.hpp>

#include "darknet_ros_msgs/BoundingBoxes.h"


boost::mutex g_mutex;

darknet_ros_msgs::BoundingBoxes g_detected_people_position_array;

void getDetectedPeoplePixelPositionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  g_mutex.lock();
  g_detected_people_position_array.bounding_boxes.clear();

  g_detected_people_position_array = *msg;
  g_mutex.unlock();
  return;
}

void getZedPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  g_mutex.lock();

  if((g_detected_people_position_array.bounding_boxes.size() == 0)
      || ((msg->header.stamp.toSec() - g_detected_people_position_array.header.stamp.toSec()) > 1.0))
  {
    ROS_INFO("There is no people");
    g_mutex.unlock();
    return;
  }

  int width = msg->width;
  int height = msg->height;

//  ROS_INFO_STREAM(msg->fields.size() <<" "
//      << msg->fields[0].name << " "
//      << msg->fields[1].name << " "
//      << msg->fields[2].name << " "
//      << msg->fields[3].name << " "
//      << msg->data.size() << " "
//      << msg->data.size()/4);

  // Image coordinates of the center pixel
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);

  for(int idx =0; idx < g_detected_people_position_array.bounding_boxes.size() ; idx++)
  {
    int u = (g_detected_people_position_array.bounding_boxes[idx].xmin + g_detected_people_position_array.bounding_boxes[idx].xmax)/2;
    int v = (g_detected_people_position_array.bounding_boxes[idx].ymin + g_detected_people_position_array.bounding_boxes[idx].ymax)/2;

    int depthIdx = u + width * v;
    ROS_INFO_STREAM( idx <<" " << cloud.points[depthIdx].x << " "
        << cloud.points[depthIdx].y << " "
        << cloud.points[depthIdx].z );
  }







  ////    ROS_INFO_STREAM("Detect Size : " << g_detected_people_position_array.bounding_boxes.size()
////        << "  header : " << g_detected_people_position_array.header.stamp);
//
//  // Get a pointer to the depth values casting the data
//  // pointer to floating point
//  float* depths = (float*)(&msg->data[0]);
//
//  // Image coordinates of the center pixel
//  int u = (g_detected_people_position_array.bounding_boxes[0].xmin + g_detected_people_position_array.bounding_boxes[0].xmax)/2;
//  int v = (g_detected_people_position_array.bounding_boxes[0].ymin + g_detected_people_position_array.bounding_boxes[0].ymax)/2;
//
//  // Linear index of the center pixel
//  int depthIdx = u + msg->width * v;
//
//  // Output the measure
//  ROS_INFO("Center distance : %g m", depths[depthIdx]);
//
//  double z = depths[depthIdx];
////
////  float x = (( u - cx) * z) / (fx);
////  float y = (( v - cy) * z) / (fy);
//  // fx,fy,cx,cy are corresponding to the left camera parameters,
//  // you can access them from zed.getCameraInformation().calibration_parameters.left_cam
//  // Check the API documentation for more:
//  // https://www.stereolabs.com/developers/documentation/API/latest


  g_mutex.unlock();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "erica_people_detector");
  ros::NodeHandle nh;

  ros::Subscriber detected_people_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, getDetectedPeoplePixelPositionCallback);
  ros::Subscriber point_cloud_sub     = nh.subscribe("/point_cloud/cloud_registered", 1, getZedPointCloudCallback);

  ros::spin();
  return 0;
}

