/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com 
******************************************************************************/
# include "controller.h"

/**
@namespace stdr_samples
@brief The main namespace for STDR Samples
**/ 
namespace robo_feup
{
  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  WallFollow::WallFollow(int argc,char **argv)
  {
    if(argc != 3)
    {
      ROS_ERROR("Usage : stdr_obstacle avoidance <robot_frame_id> <laser_frame_id>");
      exit(0);
    }
    laser_topic_ = std::string("/") + std::string(argv[1]) + std::string("/") + std::string(argv[2]);
    
    speeds_topic_ = std::string("/") + std::string(argv[1]) + std::string("/cmd_vel");
      
    subscriber_ = n_.subscribe(laser_topic_.c_str(), 1, &WallFollow::callback,this);
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>(speeds_topic_.c_str(), 1);
  }
  
  /**
  @brief Default destructor
  @return void
  **/
  WallFollow::~WallFollow(void){ }
  
  /**
  @brief Callback for the ros laser message
  @param msg [const sensor_msgs::LaserScan&] The new laser scan message
  @return void
  **/
  void WallFollow::callback(const sensor_msgs::LaserScan& msg)
  {
    scan_ = msg;
    float linear = 0, rotational = 0;

    float min_dist = MAX_FLOAT;
    float min_angle = MAX_FLOAT;
    float min_dist_left = MAX_FLOAT;
    float min_dist_right = MAX_FLOAT;
    float min_cornering_angle_left = MAX_FLOAT;//between 0 and 45º
    float min_cornering_angle_right = MAX_FLOAT;//between 0 and -45º


    for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
    {
      float real_dist = scan_.ranges[i];
      float angle = scan_.angle_min + i * scan_.angle_increment;
      float angle_degrees = MathFuncs::radianToDegree(angle);

      if(real_dist < min_dist){
        min_dist = real_dist;
        min_angle = angle;
      }
      if(angle_degrees <= 0 && angle_degrees >= MIN_ANGLE){//wall on the right side
        if(scan_.ranges[i] < min_dist_right){
          min_dist_right = scan_.ranges[i];
        }
      }
      if(angle_degrees >= 0 && angle_degrees <= MAX_ANGLE){//wall on the left side
        if(scan_.ranges[i] < min_dist_left){
          min_dist_left = scan_.ranges[i];
        }
      }
      linear -= cos(scan_.angle_min + i * scan_.angle_increment) 
        / (1.0 + real_dist * real_dist);
      /*rotational -= sin(scan_.angle_min + i * scan_.angle_increment) 
        / (1.0 + real_dist * real_dist);*/
    }
    
    linear /= scan_.ranges.size();
    //rotational /= scan_.ranges.size();
    //calculate right hand side inclination
    linear = linear > 0.3 ? 0.3 : linear;
    linear = linear < -0.3 ? -0.3 : linear;
    //detect behavior
    int behavior;
    if(scan_.range_max == min_dist){
      behavior = BEHAVIOR_SEARCHING;
    }else{
      behavior = BEHAVIOR_FOLLOWING;
    }
    //behavior
    geometry_msgs::Twist cmd;
    //choose behavior
    switch(behavior){
      case BEHAVIOR_SEARCHING:
        cmd.linear.x = STANDART_SPEED
        cmd.angular.z = 0;
      case BEHAVIOR_FOLLOWING:
        cmd.linear.x = STANDART_SPEED + linear;
        cmd.angular.z = 3*sin(angle);
      default:
        return;
    }
    //send movement command
    cmd_vel_pub_.publish(cmd);
  }
}

static float MathFuncs::degreeToRadian(float degrees){
  return degree * 3.1415926 / 180;
}

static float MathFuncs::radianToDegree(float radian){
  return radian * 180 / 3.1415926;
}
