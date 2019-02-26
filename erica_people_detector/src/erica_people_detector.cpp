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
  curr_pos_.x = NAN;
  curr_pos_.y = NAN;
  curr_pos_.z = NAN;
  curr_pixel_pos_x_ = -100000;
  curr_pixel_pos_y_ = -100000;
  curr_box_size_ = -1;
  curr_box_width_ = -1;
  curr_box_height_ = -1;


  prev_pos_.x = NAN;
  prev_pos_.y = NAN;
  prev_pos_.z = NAN;
  prev_pixel_pos_x_ = -100000;
  prev_pixel_pos_y_ = -100000;
  prev_box_size_ = -1;
  prev_box_width_ = -1;
  prev_box_height_ = -1;


  last_update_time_sec_ = 0;
  is_updated_ = false;
  is_near_ = false;
  near_time_sec_ = 0;
}

PersonTrackingInfo::~PersonTrackingInfo()
{   }

void PersonTrackingInfo::initialize()
{
  curr_pos_.x = NAN;
  curr_pos_.y = NAN;
  curr_pos_.z = NAN;
  curr_pixel_pos_x_ = -100000;
  curr_pixel_pos_y_ = -100000;
  curr_box_size_ = -1;
  curr_box_width_ = -1;
  curr_box_height_ = -1;


  prev_pos_.x = NAN;
  prev_pos_.y = NAN;
  prev_pos_.z = NAN;
  prev_pixel_pos_x_ = -100000;
  prev_pixel_pos_y_ = -100000;
  prev_box_size_ = -1;
  prev_box_width_ = -1;
  prev_box_height_ = -1;

  last_update_time_sec_ = 0;
  is_updated_ = false;
  is_near_ = false;
  near_time_sec_ = 0;
}

EricaPeopleDetecor::EricaPeopleDetecor()
{
  pixel_distance_threshold_ = 50; //pixel coordinate
  min_size_  = 20000;
  refresh_time_threshold_ = 2.0;  //sec
  distance_threshold_ = 0.5; //meter
  max_distance_ = 5.0;

  tracking_start_ = false;
  is_tracking_ = false;
  is_initialized_ = false;

  img_width_ = 1280;
  img_height_ = 720;

  near_points_num_ = -1;
  near_ref_ = 0.8;
}

EricaPeopleDetecor::~EricaPeopleDetecor()
{

}

void EricaPeopleDetecor::initialize()
{
  ros::NodeHandle nh;

  tracking_start_sub_  = nh.subscribe("/erica/people_tracking_command", 1, &EricaPeopleDetecor::getTrackingStartCommandCallback, this);
  detected_people_sub_ = nh.subscribe("/darknet_ros/bounding_boxes", 1, &EricaPeopleDetecor::getDetectedPeoplePixelPositionCallback, this);
  point_cloud_sub_     = nh.subscribe("/zed/point_cloud/cloud_registered", 1, &EricaPeopleDetecor::getZedPointCloudCallback, this);

  person_pose_pub_ = nh.advertise<erica_perception_msgs::PeoplePositionArray>("/erica/people_position", 1);
  near_points_num_pub_ = nh.advertise<std_msgs::Int32>("/erica/near_points_num", 1);

  tf_thread_ = boost::thread(boost::bind(&EricaPeopleDetecor::getTFThreadFunc, this));

  is_tracking_ = false;
  tracked_person_.initialize();
  is_initialized_ = true;
}

void EricaPeopleDetecor::getTrackingStartCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  process_mutex_.lock();
  if(msg->data == "start")
    tracking_start_ = true;
  else
  {
    tracking_start_ = false;
    is_tracking_ = false;
    tracked_person_.initialize();
  }
  process_mutex_.unlock();
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
  process_mutex_.lock();
  clearMSGtoBePublished();
  tracked_person_.prev_pos_ = tracked_person_.curr_pos_;
  tracked_person_.prev_box_size_ = tracked_person_.curr_box_size_;
  tracked_person_.prev_box_width_ = tracked_person_.curr_box_width_;
  tracked_person_.prev_box_height_ = tracked_person_.curr_box_height_;
  tracked_person_.prev_pixel_pos_x_ = tracked_person_.curr_pixel_pos_x_;
  tracked_person_.prev_pixel_pos_y_ = tracked_person_.curr_pixel_pos_y_;

  erica_perception_msgs::PeoplePositionArray detected_people;
  detected_people.img_height.data = img_height_;
  detected_people.img_width.data = img_width_;

  geometry_msgs::Point32 person;
  std_msgs::Int32 size;
  std_msgs::Int32 pixel_pos_x, pixel_pos_y;
  std_msgs::Int32 box_width, box_height;
  Eigen::Vector3d vec_person_position;
  Eigen::Affine3d mat_base_to_l_cam;

  mat_base_to_l_cam = tf2::transformToEigen(tf_base_to_l_cam_stamped_);

  near_points_num_ = 0;
  //ROS_INFO_STREAM("-----------1---------" << zed_cloud_.points.size());
  for(int idx = 0; idx < zed_cloud_.points.size(); idx++)
  {
    vec_person_position.coeffRef(0) = zed_cloud_.points[idx].x;
    vec_person_position.coeffRef(1) = zed_cloud_.points[idx].y;
    vec_person_position.coeffRef(2) = zed_cloud_.points[idx].z;

//    if(std::isnan(zed_cloud_.points[idx].x))
//      continue;
//
//    if(std::isnan(zed_cloud_.points[idx].y))
//      continue;
//
//    if(std::isnan(zed_cloud_.points[idx].z))
//      continue;

    if(zed_cloud_.points[idx].x < near_ref_)
    {
      vec_person_position = mat_base_to_l_cam.translation() + mat_base_to_l_cam.rotation()*vec_person_position;
      double depth = sqrt(vec_person_position.x()*vec_person_position.x() + vec_person_position.y()*vec_person_position.y());
      if(depth < near_ref_)
        near_points_num_++;
    }
  }
  //ROS_INFO("-----------2---------");
  std_msgs::Int32 near_points_num_msgs;
  near_points_num_msgs.data = near_points_num_;
  near_points_num_pub_.publish(near_points_num_msgs);

  if(checkError() == false)
  {
    is_tracking_ = false;
    tracked_person_.initialize();
    person_pose_pub_.publish(people_position_msg_);
    process_mutex_.unlock();
    return;
  }

  if(tracking_start_ == false)
  {
    is_tracking_ = false;
    tracked_person_.initialize();
    person_pose_pub_.publish(people_position_msg_);
    process_mutex_.unlock();
    return;
  }

  if(is_tracking_ == false)
  {
    if(detected_people_position_array_.bounding_boxes.size() == 0)
    {
      ROS_WARN("There is no people");
      is_tracking_ = false;
      tracked_person_.initialize();
      person_pose_pub_.publish(people_position_msg_);
      process_mutex_.unlock();
      return;
    }
  }

  for(int idx = 0; idx < detected_people_position_array_.bounding_boxes.size(); idx++)
  {
    int u = (detected_people_position_array_.bounding_boxes[idx].xmin + detected_people_position_array_.bounding_boxes[idx].xmax)/2;
    int v = (detected_people_position_array_.bounding_boxes[idx].ymin + detected_people_position_array_.bounding_boxes[idx].ymax)/2;

    int area = (detected_people_position_array_.bounding_boxes[idx].xmax - detected_people_position_array_.bounding_boxes[idx].xmin )
            * (detected_people_position_array_.bounding_boxes[idx].ymax - detected_people_position_array_.bounding_boxes[idx].ymin);

    size.data = area;
    box_width.data  = detected_people_position_array_.bounding_boxes[idx].xmax - detected_people_position_array_.bounding_boxes[idx].xmin;
    box_height.data = detected_people_position_array_.bounding_boxes[idx].ymax - detected_people_position_array_.bounding_boxes[idx].ymin;

    int depthIdx = u + img_width_ * v;
    vec_person_position.coeffRef(0) = zed_cloud_.points[depthIdx].x;
    vec_person_position.coeffRef(1) = zed_cloud_.points[depthIdx].y;
    vec_person_position.coeffRef(2) = zed_cloud_.points[depthIdx].z;

    vec_person_position = mat_base_to_l_cam.translation() + mat_base_to_l_cam.rotation()*vec_person_position;

    person.x = vec_person_position.x();
    person.y = vec_person_position.y();
    person.z = vec_person_position.z();

    pixel_pos_x.data =  u - img_width_/2;
    pixel_pos_y.data = -v + img_height_/2;


    double distance_from_robot = sqrt(person.x * person.x + person.y * person.y);

    if(is_tracking_ == false)
    {
      if(distance_from_robot < max_distance_)
      {
        detected_people.people_position.push_back(person);
        detected_people.box_size.push_back(size);

        detected_people.box_width.push_back(box_width);
        detected_people.box_height.push_back(box_height);
        detected_people.pixel_x.push_back(pixel_pos_x);
        detected_people.pixel_y.push_back(pixel_pos_y);
      }
      else
        continue;
    }
    else
    {
      if((distance_from_robot < max_distance_)
          || (std::isnan(distance_from_robot) && size.data > min_size_) )
      {
        detected_people.people_position.push_back(person);
        detected_people.box_size.push_back(size);

        detected_people.box_width.push_back(box_width);
        detected_people.box_height.push_back(box_height);
        detected_people.pixel_x.push_back(pixel_pos_x);
        detected_people.pixel_y.push_back(pixel_pos_y);
      }
      else
        continue;
    }
  }

  for(int idx = 0; idx < detected_people.people_position.size(); idx++)
  {
    ROS_INFO_STREAM("idx : " << idx << " "  << detected_people.people_position[idx].x <<
    " "  << detected_people.people_position[idx].y <<
    " "  << detected_people.people_position[idx].z);
  }

  if(detected_people.box_size.size() == 0)
  {
    if(is_tracking_ == false)
    {
      person_pose_pub_.publish(people_position_msg_);
      process_mutex_.unlock();
      ROS_WARN("There is no people");
      return;
    }
    else //after 2sec, there is no people;
    {
      if( (ros::Time::now().toSec() - tracked_person_.last_update_time_sec_) > refresh_time_threshold_)
      {
        is_tracking_ = false;
        person_pose_pub_.publish(people_position_msg_);
        tracked_person_.initialize();
        process_mutex_.unlock();
        ROS_WARN("missed the tracked person");
        return;
      }
      else
      {
        tracked_person_.is_updated_ = false;

        person = tracked_person_.curr_pos_;
        size.data = tracked_person_.curr_box_size_;
        pixel_pos_x.data = tracked_person_.curr_pixel_pos_x_;
        pixel_pos_y.data = tracked_person_.curr_pixel_pos_y_;
        box_width.data = tracked_person_.curr_box_width_;
        box_height.data = tracked_person_.curr_box_height_;

        people_position_msg_.people_position.push_back(tracked_person_.curr_pos_);
        people_position_msg_.box_size.push_back(size);
        people_position_msg_.box_width.push_back(box_width);
        people_position_msg_.box_height.push_back(box_height);
        people_position_msg_.pixel_x.push_back(pixel_pos_x);
        people_position_msg_.pixel_y.push_back(pixel_pos_y);
        person_pose_pub_.publish(people_position_msg_);
        process_mutex_.unlock();
        ROS_WARN("there is no people, but estimate the person position");
        return;
      }
    }
  }

  //check the nearest person
  if(is_tracking_ == false)
  {
    tracked_person_.curr_pos_.x = detected_people.people_position[0].x;
    tracked_person_.curr_pos_.y = detected_people.people_position[0].y;
    tracked_person_.curr_pos_.z = detected_people.people_position[0].z;

    for(int idx = 1; idx < detected_people.people_position.size(); idx++)
    {
      if((tracked_person_.curr_pos_.x*tracked_person_.curr_pos_.x + tracked_person_.curr_pos_.y*tracked_person_.curr_pos_.y)
          > (detected_people.people_position[idx].x*detected_people.people_position[idx].x + detected_people.people_position[idx].y*detected_people.people_position[idx].y))
      {
        tracked_person_.curr_pos_.x = detected_people.people_position[idx].x;
        tracked_person_.curr_pos_.y = detected_people.people_position[idx].y;
        tracked_person_.curr_pos_.z = detected_people.people_position[idx].z;
        tracked_person_.curr_box_width_ = detected_people.box_width[idx].data;
        tracked_person_.curr_box_height_ = detected_people.box_height[idx].data;
        tracked_person_.curr_box_size_ = detected_people.box_size[idx].data;
        tracked_person_.curr_pixel_pos_x_ = detected_people.pixel_x[idx].data;
        tracked_person_.curr_pixel_pos_y_ = detected_people.pixel_y[idx].data;

        ROS_INFO_STREAM("update2");
      }

      ROS_INFO_STREAM("idx2 : " << idx << " "  << detected_people.people_position[idx].x <<
      " "  << detected_people.people_position[idx].y <<
      " "  << detected_people.people_position[idx].z);
    }

    is_tracking_ = true;

    tracked_person_.is_updated_ = true;
    tracked_person_.last_update_time_sec_ = ros::Time::now().toSec();
    person = tracked_person_.curr_pos_;
    size.data = tracked_person_.curr_box_size_;
    box_width.data = tracked_person_.curr_box_width_;
    box_height.data = tracked_person_.curr_box_height_;
    pixel_pos_x.data = tracked_person_.curr_pixel_pos_x_;
    pixel_pos_y.data = tracked_person_.curr_pixel_pos_y_;

    people_position_msg_.people_position.push_back(person);
    people_position_msg_.box_size.push_back(size);
    people_position_msg_.box_width.push_back(box_width);
    people_position_msg_.box_height.push_back(box_height);
    people_position_msg_.pixel_x.push_back(pixel_pos_x);
    people_position_msg_.pixel_y.push_back(pixel_pos_y);

    person_pose_pub_.publish(people_position_msg_);
    process_mutex_.unlock();

    ROS_WARN("tracking start");
    ROS_INFO_STREAM("Pos : " << tracked_person_.curr_pos_.x  << " " << tracked_person_.curr_pos_.y << " "  << tracked_person_.curr_pos_.z);

    return;
  }
  else
  {
    tracked_person_.is_updated_ = false;
    double prev_dist_diff = 1000000.0;

    if(tracked_person_.is_near_ == false)
    {
      for(int idx = 0; idx < detected_people.people_position.size(); idx++)
      {
        double curr_dist_diff
        = sqrt((detected_people.people_position[idx].x - tracked_person_.prev_pos_.x)*(detected_people.people_position[idx].x - tracked_person_.prev_pos_.x)
            + (detected_people.people_position[idx].y - tracked_person_.prev_pos_.y)*(detected_people.people_position[idx].y - tracked_person_.prev_pos_.y));

        if((curr_dist_diff < distance_threshold_) && (curr_dist_diff < prev_dist_diff))
        {
          tracked_person_.curr_pos_.x = detected_people.people_position[idx].x;
          tracked_person_.curr_pos_.y = detected_people.people_position[idx].y;
          tracked_person_.curr_pos_.z = detected_people.people_position[idx].z;
          tracked_person_.curr_box_width_ = detected_people.box_width[idx].data;
          tracked_person_.curr_box_height_ = detected_people.box_height[idx].data;
          tracked_person_.curr_box_size_ = detected_people.box_size[idx].data;
          tracked_person_.curr_pixel_pos_x_ = detected_people.pixel_x[idx].data;
          tracked_person_.curr_pixel_pos_y_ = detected_people.pixel_y[idx].data;

          tracked_person_.is_updated_ = true;

          prev_dist_diff = curr_dist_diff;
        }
      }

      if(tracked_person_.is_updated_ == true)
      {
        tracked_person_.last_update_time_sec_ = ros::Time::now().toSec();

        double curr_dist = sqrt(tracked_person_.curr_pos_.x * tracked_person_.curr_pos_.x + tracked_person_.curr_pos_.y * tracked_person_.curr_pos_.y);
        if(curr_dist < 0.5)
        {
          tracked_person_.is_near_ = true;
          tracked_person_.near_time_sec_ = ros::Time::now().toSec();
        }

        person = tracked_person_.curr_pos_;
        size.data = tracked_person_.curr_box_size_;
        pixel_pos_x.data = tracked_person_.curr_pixel_pos_x_;
        pixel_pos_y.data = tracked_person_.curr_pixel_pos_y_;
        box_width.data = tracked_person_.curr_box_width_;
        box_height.data = tracked_person_.curr_box_height_;

        people_position_msg_.people_position.push_back(tracked_person_.curr_pos_);
        people_position_msg_.box_size.push_back(size);
        people_position_msg_.box_width.push_back(box_width);
        people_position_msg_.box_height.push_back(box_height);
        people_position_msg_.pixel_x.push_back(pixel_pos_x);
        people_position_msg_.pixel_y.push_back(pixel_pos_y);

        person_pose_pub_.publish(people_position_msg_);
        process_mutex_.unlock();
        ROS_WARN("tracking.... updated pos");
        ROS_INFO_STREAM("Pos : " << tracked_person_.curr_pos_.x  << " " << tracked_person_.curr_pos_.y << " "  << tracked_person_.curr_pos_.z);
        return;
      }
      else
      {

        if( (ros::Time::now().toSec() - tracked_person_.last_update_time_sec_) > refresh_time_threshold_)
        {
          is_tracking_ = false;
          person_pose_pub_.publish(people_position_msg_);
          tracked_person_.initialize();
          process_mutex_.unlock();
          ROS_WARN("missed the tracked person");
          return;
        }

        person = tracked_person_.curr_pos_;
        size.data = tracked_person_.curr_box_size_;
        pixel_pos_x.data = tracked_person_.curr_pixel_pos_x_;
        pixel_pos_y.data = tracked_person_.curr_pixel_pos_y_;
        box_width.data = tracked_person_.curr_box_width_;
        box_height.data = tracked_person_.curr_box_height_;

        people_position_msg_.people_position.push_back(tracked_person_.curr_pos_);
        people_position_msg_.box_size.push_back(size);
        people_position_msg_.box_width.push_back(box_width);
        people_position_msg_.box_height.push_back(box_height);
        people_position_msg_.pixel_x.push_back(pixel_pos_x);
        people_position_msg_.pixel_y.push_back(pixel_pos_y);

        person_pose_pub_.publish(people_position_msg_);
        process_mutex_.unlock();
        ROS_WARN("far tracking fail. we estimate the tracked person in previous position... ");
        return;
      }

    }
    else //near box size ref
    {
      int prev_box_size = detected_people.box_size[0].data;
      if(prev_box_size > 100000)
        tracked_person_.is_updated_ = true;

      for(int idx = 0; idx < detected_people.people_position.size(); idx++)
      {
        int curr_box_size = detected_people.box_size[idx].data;

        if((curr_box_size > prev_box_size) && (curr_box_size > 100000))
        {
          tracked_person_.curr_pos_.x = detected_people.people_position[idx].x;
          tracked_person_.curr_pos_.y = detected_people.people_position[idx].y;
          tracked_person_.curr_pos_.z = detected_people.people_position[idx].z;
          tracked_person_.curr_box_width_ = detected_people.box_width[idx].data;
          tracked_person_.curr_box_height_ = detected_people.box_height[idx].data;
          tracked_person_.curr_box_size_ = detected_people.box_size[idx].data;
          tracked_person_.curr_pixel_pos_x_ = detected_people.pixel_x[idx].data;
          tracked_person_.curr_pixel_pos_y_ = detected_people.pixel_y[idx].data;

          tracked_person_.is_updated_ = true;

          prev_box_size = curr_box_size;
        }
      }

      if(tracked_person_.is_updated_ == false)
      {
        if(ros::Time::now().toSec() - tracked_person_.last_update_time_sec_ > refresh_time_threshold_)
        {
          is_tracking_ = false;
          tracked_person_.initialize();
          person_pose_pub_.publish(people_position_msg_);
          process_mutex_.unlock();
          ROS_WARN("tracking fail");
          return;
        }
        else
        {
          person = tracked_person_.curr_pos_;
          size.data = tracked_person_.curr_box_size_;
          pixel_pos_x.data = tracked_person_.curr_pixel_pos_x_;
          pixel_pos_y.data = tracked_person_.curr_pixel_pos_y_;
          box_width.data = tracked_person_.curr_box_width_;
          box_height.data = tracked_person_.curr_box_height_;

          people_position_msg_.people_position.push_back(tracked_person_.curr_pos_);
          people_position_msg_.box_size.push_back(size);
          people_position_msg_.box_width.push_back(box_width);
          people_position_msg_.box_height.push_back(box_height);
          people_position_msg_.pixel_x.push_back(pixel_pos_x);
          people_position_msg_.pixel_y.push_back(pixel_pos_y);

          person_pose_pub_.publish(people_position_msg_);
          process_mutex_.unlock();
          ROS_WARN("near tracking fail. we estimate the tracked person in previous position...");
          return;
        }
      }
      else //near updated
      {
        //      if(tracked_person_.is_near_ == true)
        //      {
        //       if((ros::Time::now().toSec() - tracked_person_.near_time_sec_) > 10.0)
        //       {
        //         is_tracking_ = false;
        //       }
        //      }
        double curr_dist = sqrt(tracked_person_.curr_pos_.x * tracked_person_.curr_pos_.x + tracked_person_.curr_pos_.y * tracked_person_.curr_pos_.y);
        if((curr_dist > 0.5) && (!(std::isnan(curr_dist))))
        {
          tracked_person_.is_near_ = false;
          tracked_person_.near_time_sec_ = ros::Time::now().toSec();
        }

        tracked_person_.last_update_time_sec_ = ros::Time::now().toSec();
        person = tracked_person_.curr_pos_;
        size.data = tracked_person_.curr_box_size_;
        pixel_pos_x.data = tracked_person_.curr_pixel_pos_x_;
        pixel_pos_y.data = tracked_person_.curr_pixel_pos_y_;
        box_width.data = tracked_person_.curr_box_width_;
        box_height.data = tracked_person_.curr_box_height_;

        people_position_msg_.people_position.push_back(tracked_person_.curr_pos_);
        people_position_msg_.box_size.push_back(size);
        people_position_msg_.box_width.push_back(box_width);
        people_position_msg_.box_height.push_back(box_height);
        people_position_msg_.pixel_x.push_back(pixel_pos_x);
        people_position_msg_.pixel_y.push_back(pixel_pos_y);

        person_pose_pub_.publish(people_position_msg_);
        process_mutex_.unlock();
        ROS_WARN("near tracking...");
        return;
      }
    }
  }

  ROS_ERROR("upexpected_situation");
  person_pose_pub_.publish(people_position_msg_);
  process_mutex_.unlock();
}
