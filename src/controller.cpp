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
    srand(time(NULL));
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

    float linear = 0;

    float min_dist = MAX_FLOAT;
    float min_angle = MAX_FLOAT;
    float min_dist_left = MAX_FLOAT;
    float min_angle_left = MAX_FLOAT;
    float min_dist_right = MAX_FLOAT;
    float min_angle_right = MAX_FLOAT;

    for(unsigned int i = 0 ; i < scan_.ranges.size() ; i++)
    {
      float real_dist = scan_.ranges[i] > scan_.range_max ? scan_.range_max : scan_.ranges[i];
      float angle = scan_.angle_min + i * scan_.angle_increment;
      float angle_degrees = MathFuncs::radianToDegree(angle);

      if(real_dist <= min_dist){
        min_dist = real_dist;
        min_angle = angle;
      }
      if(angle_degrees <= 0 && angle_degrees >= -MIN_TURN_ANGLE){//wall on the right side
        if(real_dist < min_dist_right){
          min_dist_right = real_dist;
          min_angle_right = angle;
        }
      }
      if(angle_degrees >= 0 && angle_degrees <= MIN_TURN_ANGLE){//wall on the left side
        if(real_dist < min_dist_left){
          min_dist_left = real_dist;
          min_angle_left = angle;
        }
      }
    }
    linear = ((scan_.range_max - min_dist + ROBOT_RADIUS)/scan_.range_max) * cos(min_angle) * STANDART_SPEED * 0.5;
    //detect behavior
    int behavior;
    if(scan_.range_max > min_dist && 
       (min_angle < 0 && min_dist_left < scan_.range_max - 0.1 || //0.1 because the sensors were returning incorrect values
        min_angle > 0 && min_dist_right < scan_.range_max - 0.1)){
      behavior = BEHAVIOR_CORNERING;
    }else if(scan_.range_max > min_dist){
      behavior = BEHAVIOR_FOLLOWING;
    }else{
      behavior = BEHAVIOR_SEARCHING;
    }
    //behavior
    geometry_msgs::Twist cmd;
    //choose behavior
    float ang = 0;//turning angle of the robot
    float orientation = 0;//position of the wall relative to the robot(-1 = left, 1 = right)
    switch(behavior){
      case BEHAVIOR_SEARCHING:
        cmd.linear.x = STANDART_SPEED;
        cmd.angular.z = 0;

        std::cout << "SEARCHING\n";
        break;
      case BEHAVIOR_FOLLOWING:
        cmd.linear.x = STANDART_SPEED - linear;
        orientation = min_angle > 0 ? -1 : 1;
        ang += cos(min_angle) * orientation;
        ang += min_dist >= BEST_WALL_RANGE ? -(abs(BEST_WALL_RANGE-min_dist)/BEST_WALL_RANGE) * orientation : (abs(BEST_WALL_RANGE-min_dist)/BEST_WALL_RANGE) * orientation;
        cmd.angular.z = STANDART_SPEED * ang * PI * 0.75;

        std::cout << "FOLLOWING WALL";
        if(orientation == -1){
          std::cout << " AT LEFT\n";
        }else{
          std::cout << " AT RIGHT\n";
        }
        std::cout << "Distance from wall= " << min_dist << " (Ideal=" << BEST_WALL_RANGE << ")\n";
        break;
      case BEHAVIOR_CORNERING:
        cmd.linear.x = STANDART_SPEED - linear;
        orientation = min_angle > 0 ? -1 : 1;
        ang += orientation == -1 ? orientation * (sin(abs(min_angle - min_angle_right))) : orientation * (sin(abs(min_angle - min_angle_left)));
        cmd.angular.z = STANDART_SPEED * ang * PI * 1.25;

	std::cout << "CORNERING";
	if(orientation == -1){
          std::cout << " TO THE RIGHT\n";
        }else{
          std::cout << " TO THE LEFT\n";
        }
        break;
      default:
        return;
    }
    //send movement command
    std::cout << "|----------------------------|\n";
    cmd_vel_pub_.publish(cmd);
  }


  float MathFuncs::degreeToRadian(float degrees){
    return degrees * PI / 180;
  }

  float MathFuncs::radianToDegree(float radian){
    return radian * 180 / PI;
  }
}
