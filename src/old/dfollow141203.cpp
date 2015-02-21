//http://marksilliman.com/2014/10/06/turtlebots-missing-hello-world-program/
/*
 * Hello World and Crash for Turtlebot
 * Moves Turtlebot forward until you ctrl + c
 * Tested using TurtleBot 2, ROS Indigo, Ubuntu 14.04
 */
/// Read home/kensei4/hydro/src/amcl/src/amcl_node.cpp
// 141203: 何とか追跡できるようになったが、壁を追跡者と間違えたる
// To Do: 人と壁の判別、比例航法の実装


#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

const double follow_max_distance = 2.0; // follow distance
const double follow_min_distance = 0.2; // follow distance 
const double follow_angle = 30;     // 探す範囲は正面のこの角度[deg]  
const double gain_linear = 0.5;            // 比例ゲイン
const double gain_turn  = 5.0;
const double turn_speed          = 1.0;
const double linear_speed        = 0.5;

double laser_data[1080]; // hokuyo lidar UTM-30LX
double target_angle;     // target angle [rad] center is 0 [rad]
double target_distance;  // minimum distance from a robot



void laserCallback(const sensor_msgs::LaserScan laser_scan)
{
  int dataNum = 0;
  int dataCount;

  //ROS_INFO("size[%d]: ", laser_scan.ranges.size());
  //ROS_INFO("angle min=%.2f max=%.2f inc=%f\n",laser_scan.angle_min,
  //	   laser_scan.angle_max, laser_scan.angle_increment);
  //ROS_INFO("range min=%.2f max=%.2f \n",laser_scan.range_min,
  //         laser_scan.range_max);

  dataCount= laser_scan.ranges.size();
  
  for(int i = 0; i < dataCount; i++) {       
    double value = laser_scan.ranges[i];
    //ROS_INFO("value[%d]:%f\n", i,value);
    if ((value >= laser_scan.range_min) && (value <= laser_scan.range_max))
    {
      laser_data[i] = value;
    }
    else {
      laser_data[i] = 999;
    }
  }

  //for(int i = 0; i < dataCount; i++) {
  //  ROS_INFO("myvalue[%d]:%f\n", i,laser_data[i]);                            
  //}       
 
  int center = dataCount/2;  // レーザーの中央
  double init_search_dist  = 2.0; // 追跡する初期距離 [m]
  int search_lines = 1080 *  (follow_angle/270.0); 
  int left=0, right=dataCount;
  int sum_no = 0, no=0;
  target_distance = 999;

  for (int j = center - search_lines/2; j <= center + search_lines/2; j++) {
    if (laser_data[j] == 999) {
      no++;
    }
    else if (laser_data[j] < init_search_dist) {
      sum_no += j; // レーザー光線の番号を足す
      if (target_distance > laser_data[j])  {
	target_distance = laser_data[j];
      }
      no++;
    }
  }

  if (no <=0) {
    ROS_INFO("No target\n");
    target_distance = 999;
    target_angle    = 0.0;
    return;
    // exit(1);
  }

  //ROS_INFO("Target distance=%f \n",target_distance);

  int target_center = sum_no/no; // 光線の番号
  target_angle 
    = (laser_scan.angle_max - laser_scan.angle_min) * target_center
    / (double) dataCount + laser_scan.angle_min; 
  //printf("Target center=%d angle=%f\n",target_center, target_angle);
  //cout << "1:" << 
}

int main(int argc, char** argv)
{
  // laser
  ros::init(argc, argv, "hLaserReader");
  ros::NodeHandle node;
  //ros::Subscriber hokuyoSubscriber = node.subscribe("/scan", 1, scanValues);
  ros::Subscriber laserSubscriber = node.subscribe("/scan", 100, laserCallback);

  //ros::NodeHandle nh("~");
  //if(!nh.getParam("max_margin",max_margin)) max_margin = 10.0;
  //if(!nh.getParam("min_margin",min_margin)) min_margin =  0.5;
  //ros::spin();


  //init the ROS node
  //ROS_INFO_STREAM("Hello World");
  //ros::init(argc, argv, "robot_driver");
  //ros::NodeHandle nh;

  //init publisher
  ros::Publisher cmd_vel_pub_;
  cmd_vel_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

  //init direction that turtlebot should go
  geometry_msgs::Twist cmd_speed;

  cmd_speed.linear.x  = 0; // m/s
  cmd_speed.linear.y  = 0; // m/s
  cmd_speed.angular.z = 0; // rad

  ros::Rate loop_rate(30); // 5Hz

  while(ros::ok()) { //have we ctrl + C?  If no... keep going!
    //"publish" sends the command to turtlebot to keep going
    ROS_INFO("Target dist=%f angle=%f\n", target_distance, target_angle);

    if ((target_distance >=  follow_min_distance) &&
	(target_distance <=  follow_max_distance)) {
      double tmp_speed = gain_linear * (follow_min_distance - target_distance);
      if (fabs(tmp_speed) > linear_speed) tmp_speed = linear_speed;
      if (tmp_speed < 0) tmp_speed = 0;
      cmd_speed.linear.x = tmp_speed;
    }   
    else {
      cmd_speed.linear.x = 0;
    }

    if (target_angle > 5.0 * M_PI/ 180.0) {
      cmd_speed.angular.z =  gain_turn * target_angle;
    }
    else {
      cmd_speed.angular.z = 0;
    }

    cmd_vel_pub_.publish(cmd_speed);
    loop_rate.sleep();
    ros::spinOnce();


  }


  ROS_INFO("Finished\n");

  return 0;
}

