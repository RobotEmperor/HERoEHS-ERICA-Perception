/*
 * erica_people_detector.h
 *
 *  Created on: Jan 23, 2019
 *      Author: jay
 */

#ifndef ERICA_PEOPLE_DETECTOR_ERICA_PEOPLE_DETECTOR_H_
#define ERICA_PEOPLE_DETECTOR_ERICA_PEOPLE_DETECTOR_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/thread.hpp>

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "erica_perception_msgs/PeoplePositionArray.h"

namespace erica
{

class PersonTrackingInfo
{
public:
  PersonTrackingInfo();
  ~PersonTrackingInfo();

  void initialize();

  double curr_pos_x_;
  double curr_pos_y_;
  double curr_pos_z_;

  double prev_pos_x_;
  double prev_pos_y_;
  double prev_pos_z_;

  int prev_pixel_pos_x_;
  int prev_pixel_pos_y_;

  int curr_pixel_pos_x_;
  int curr_pixel_pos_y_;

  double last_update_time_sec_;
  bool is_updated_;
  bool is_near_;
};

class EricaPeopleDetecor
{
public:
  EricaPeopleDetecor();
  ~EricaPeopleDetecor();

  void initialize();

  void getDetectedPeoplePixelPositionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
  void getZedPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void getTFThreadFunc();

  void process();

  bool checkError();
  void clearMSGtoBePublished();
  void publishPeoplePosition();

  int32_t pixel_distance_threshold_; //pixel coordinate
  int32_t size_threshold_;
  double_t refresh_time_threshold_;  //sec

private:
  boost::thread   tf_thread_;

  boost::mutex process_mutex_;

  darknet_ros_msgs::BoundingBoxes detected_people_position_array_; //people detection from yolo
  sensor_msgs::PointCloud zed_cloud_; //point cloud from zed
  geometry_msgs::TransformStamped tf_base_to_l_cam_stamped_; //tf from robot_state_publisher

  Eigen::Affine3d mat_base_to_lcam_;

  ros::Publisher robot_pose_pub_;
  erica_perception_msgs::PeoplePositionArray people_position_msg_;


  bool is_tracking_;
  bool is_initialized_;
  int32_t img_width_, img_height_;

  PersonTrackingInfo tracked_person_;

  ros::Subscriber detected_people_sub_;
  ros::Subscriber point_cloud_sub_;

  ros::Publisher person_pose_pub_;

};

}

#endif /* ERICA_PEOPLE_DETECTOR_ERICA_PEOPLE_DETECTOR_H_ */
