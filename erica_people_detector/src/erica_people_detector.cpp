/*
 * erica_people_detector.cpp
 *
 *  Created on: Jan 9, 2019
 *      Author: jay
 */


#include "erica_people_detector/erica_people_detector.h"

using namespace erica;



PersonTrackingInfo::PersonTrackingInfo()
{
  curr_pos_x_ = NAN;
  curr_pos_y_ = NAN;
  curr_pos_z_ = NAN;

  prev_pos_x_ = NAN;
  prev_pos_y_ = NAN;
  prev_pos_z_ = NAN;

  prev_pixel_pos_x_ = -100000;
  prev_pixel_pos_y_ = -100000;

  curr_pixel_pos_x_ = -100000;
  curr_pixel_pos_y_ = -100000;

  last_update_time_sec_ = 0;
  is_updated_ = false;
  is_near_ = false;
}

PersonTrackingInfo::~PersonTrackingInfo()
{   }

void PersonTrackingInfo::initialize()
{
  curr_pos_x_ = NAN;
  curr_pos_y_ = NAN;
  curr_pos_z_ = NAN;

  prev_pos_x_ = NAN;
  prev_pos_y_ = NAN;
  prev_pos_z_ = NAN;

  prev_pixel_pos_x_ = -100000;
  prev_pixel_pos_y_ = -100000;

  curr_pixel_pos_x_ = -100000;
  curr_pixel_pos_y_ = -100000;

  last_update_time_sec_ = 0;
  is_updated_ = false;
  is_near_ = false;
}

EricaPeopleDetecor::EricaPeopleDetecor()
{
  pixel_distance_threshold_ = 50; //pixel coordinate
  size_threshold_  = 22500;
  refresh_time_threshold_ = 1.5;  //sec
  distance_threshold_ = 5.0; //meter

  is_tracking_ = false;
  is_initialized_ = false;

  img_width_ = 1280;
  img_height_ = 720;
}

EricaPeopleDetecor::~EricaPeopleDetecor()
{

}

void EricaPeopleDetecor::initialize()
{
  ros::NodeHandle nh;

  detected_people_sub_ = nh.subscribe("/darknet_ros/bounding_boxes", 1, &EricaPeopleDetecor::getDetectedPeoplePixelPositionCallback, this);
  point_cloud_sub_     = nh.subscribe("/zed/point_cloud/cloud_registered", 1, &EricaPeopleDetecor::getZedPointCloudCallback, this);

  person_pose_pub_ = nh.advertise<erica_perception_msgs::PeoplePositionArray>("/erica/people_position", 1);

  tf_thread_ = boost::thread(boost::bind(&EricaPeopleDetecor::getTFThreadFunc, this));

}

void EricaPeopleDetecor::getDetectedPeoplePixelPositionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  process_mutex_.lock();
  detected_people_position_array_.bounding_boxes.clear();

  detected_people_position_array_ = *msg;

  detected_people_position_array_.header.stamp = ros::Time::now();

  process_mutex_.unlock();
  return;
}


void EricaPeopleDetecor::getZedPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  process_mutex_.lock();
  img_width_  = 1280;
  img_height_ = 720;

  // Image coordinates of the center pixel
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, zed_cloud_);

  zed_cloud_.header.stamp = ros::Time::now();

  process_mutex_.unlock();
}


void EricaPeopleDetecor::getTFThreadFunc()
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(30.0);
  while(ros::ok())
  {
    process_mutex_.lock();
    try{
      //get transformation from body_main to zed_left_camera
      tf_base_to_l_cam_stamped_
        = tfBuffer.lookupTransform("body_main", "zed_left_camera_frame", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      process_mutex_.unlock();
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    tf_base_to_l_cam_stamped_.header.stamp = ros::Time::now();

    process_mutex_.unlock();
    rate.sleep();
  }
}

void EricaPeopleDetecor::process()
{
  clearMSGtoBePublished();
  process_mutex_.lock();
  if(checkError() == false)
  {
    process_mutex_.unlock();
    person_pose_pub_.publish(people_position_msg_);
    return;
  }

  erica_perception_msgs::PeoplePositionArray people_position;

  geometry_msgs::Point32 person;
  std_msgs::Int32 size;
  std_msgs::Int32 pixel_pos_x;
  std_msgs::Int32 pixel_pos_y;
  std_msgs::Int32 box_width;
  std_msgs::Int32 box_height;
  Eigen::Vector3d vec_person_position;
  Eigen::Affine3d mat_base_to_l_cam;

  for(int idx = 0; idx < detected_people_position_array_.bounding_boxes.size(); idx++)
  {
    int u = (detected_people_position_array_.bounding_boxes[idx].xmin + detected_people_position_array_.bounding_boxes[idx].xmax)/2;
    int v = (detected_people_position_array_.bounding_boxes[idx].ymin + detected_people_position_array_.bounding_boxes[idx].ymax)/2;

    int area = (detected_people_position_array_.bounding_boxes[idx].xmax - detected_people_position_array_.bounding_boxes[idx].xmin )
            * (detected_people_position_array_.bounding_boxes[idx].ymax - detected_people_position_array_.bounding_boxes[idx].ymin);

    int depthIdx = u + img_width_ * v;

    vec_person_position.coeffRef(0) = zed_cloud_.points[depthIdx].x;
    vec_person_position.coeffRef(1) = zed_cloud_.points[depthIdx].y;
    vec_person_position.coeffRef(2) = zed_cloud_.points[depthIdx].z;
    size.data = area;
    box_width.data  = detected_people_position_array_.bounding_boxes[idx].xmax - detected_people_position_array_.bounding_boxes[idx].xmin;
    box_height.data = detected_people_position_array_.bounding_boxes[idx].ymax - detected_people_position_array_.bounding_boxes[idx].ymin;


    mat_base_to_l_cam = tf2::transformToEigen(tf_base_to_l_cam_stamped_);
    vec_person_position = mat_base_to_l_cam.translation() + mat_base_to_l_cam.rotation()*vec_person_position;

    person.x = vec_person_position.x();
    person.y = vec_person_position.y();
    person.z = vec_person_position.z();

    pixel_pos_x.data =  u - img_width_/2;
    pixel_pos_y.data = -v + img_height_/2;

    people_position_msg_.people_position.push_back(person);
    people_position_msg_.box_size.push_back(size);
    people_position_msg_.box_width.push_back(box_width);
    people_position_msg_.box_height.push_back(box_height);
    people_position_msg_.pixel_x.push_back(pixel_pos_x);
    people_position_msg_.pixel_y.push_back(pixel_pos_y);
  }

  person_pose_pub_.publish(people_position_msg_);
  process_mutex_.unlock();
}

void EricaPeopleDetecor::clearMSGtoBePublished()
{
  people_position_msg_.people_position.clear();
  people_position_msg_.box_size.clear();
  people_position_msg_.pixel_x.clear();
  people_position_msg_.pixel_y.clear();
  people_position_msg_.box_width.clear();
  people_position_msg_.box_height.clear();
  people_position_msg_.box_size.clear();

  people_position_msg_.img_width.data  = img_width_;
  people_position_msg_.img_height.data = img_height_;
}

bool EricaPeopleDetecor::checkError()
{
  // chech yolo
  if(detected_people_position_array_.header.stamp.sec == 0)
  {
    ROS_WARN("Yolo may not be started or you are using wrong topic");
    return false;
  }

  // check zed_wrapper
  if(zed_cloud_.header.stamp.sec == 0)
  {
    ROS_WARN("zed_wrapper may not be started or you are using wrong topic");
    return false;
  }

  if(tf_base_to_l_cam_stamped_.header.stamp.sec == 0)
  {
    ROS_WARN("manager may not be started");
    return false;
  }

  // check time
  if(  (fabs(zed_cloud_.header.stamp.toSec() - detected_people_position_array_.header.stamp.toSec()) > 1.0)
       || (fabs(zed_cloud_.header.stamp.toSec() - tf_base_to_l_cam_stamped_.header.stamp.toSec()) > 1.0)
       || (fabs(tf_base_to_l_cam_stamped_.header.stamp.toSec() - detected_people_position_array_.header.stamp.toSec() > 1.0))
    )
  {
    ROS_WARN_STREAM("[TIME_OVER_ERROR] zed_time : " << zed_cloud_.header.stamp.toSec()
        << " yolo_time : " << detected_people_position_array_.header.stamp.toSec()
        << " tf_time : " << tf_base_to_l_cam_stamped_.header.stamp.toSec());
    return false;
  }

  return true;
}


void EricaPeopleDetecor::process2()
{
  clearMSGtoBePublished();
  process_mutex_.lock();
  if(checkError() == false)
  {
    process_mutex_.unlock();
    return;
  }

  erica_perception_msgs::PeoplePositionArray people_position;

  geometry_msgs::Point32 person;
  std_msgs::Int32 size;
  std_msgs::Int32 pixel_pos_x, pixel_pos_y;
  std_msgs::Int32 box_width, box_height;
  Eigen::Vector3d vec_person_position;
  Eigen::Affine3d mat_base_to_l_cam;

  if(detected_people_position_array_.bounding_boxes.size() == 0)
  {
    ROS_WARN("There is no people");
    return;
  }

  for(int idx = 0; idx < detected_people_position_array_.bounding_boxes.size(); idx++)
  {
    int u = (detected_people_position_array_.bounding_boxes[idx].xmin + detected_people_position_array_.bounding_boxes[idx].xmax)/2;
    int v = (detected_people_position_array_.bounding_boxes[idx].ymin + detected_people_position_array_.bounding_boxes[idx].ymax)/2;

    int area = (detected_people_position_array_.bounding_boxes[idx].xmax - detected_people_position_array_.bounding_boxes[idx].xmin )
            * (detected_people_position_array_.bounding_boxes[idx].ymax - detected_people_position_array_.bounding_boxes[idx].ymin);

    int depthIdx = u + img_width_ * v;

    vec_person_position.coeffRef(0) = zed_cloud_.points[depthIdx].x;
    vec_person_position.coeffRef(1) = zed_cloud_.points[depthIdx].y;
    vec_person_position.coeffRef(2) = zed_cloud_.points[depthIdx].z;
    size.data = area;
    box_width.data  = detected_people_position_array_.bounding_boxes[idx].xmax - detected_people_position_array_.bounding_boxes[idx].xmin;
    box_height.data = detected_people_position_array_.bounding_boxes[idx].ymax - detected_people_position_array_.bounding_boxes[idx].ymin;


    mat_base_to_l_cam = tf2::transformToEigen(tf_base_to_l_cam_stamped_);
    vec_person_position = mat_base_to_l_cam.translation() + mat_base_to_l_cam.rotation()*vec_person_position;

    person.x = vec_person_position.x();
    person.y = vec_person_position.y();
    person.z = vec_person_position.z();

    pixel_pos_x.data =  u - img_width_/2;
    pixel_pos_y.data = -v + img_height_/2;


    vec_person_position.norm();
    people_position_msg_.people_position.push_back(person);
    people_position_msg_.box_size.push_back(size);
    people_position_msg_.box_width.push_back(box_width);
    people_position_msg_.box_height.push_back(box_height);
    people_position_msg_.pixel_x.push_back(pixel_pos_x);
    people_position_msg_.pixel_y.push_back(pixel_pos_y);
  }

  person_pose_pub_.publish(people_position_msg_);
  process_mutex_.unlock();
}
