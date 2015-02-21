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

using namespace std;

const double follow_max_distance = 3.0; // follow distance
const double follow_min_distance = 0.2; // follow distance 
const double leg_width_max = 0.3;   // 脚幅の最大値
const double leg_width_min = 0.08;  // 脚幅の最大値
const double follow_angle = 45;     // 探す範囲は正面のこの角度[deg]  
const double gain_linear = 0.5;            // 比例ゲイン
const double gain_turn  = 5.0;
const double turn_max_speed     = 1.0;
const double linear_max_speed   = 0.5;

double laser_data[1080]; // hokuyo lidar UTM-30LX
double target_angle;     // target angle [rad] center is 0 [rad]
double target_distance;  // minimum distance from a robot

class Object {
public:
  bool leg;   // leg:true  or not:false
  int begin; // right side laser
  int end;   // left side laser
  int diff;
  double distance;
  double angle; // rad
  double width;
};

Object obj[100], human;

void findHuman(int dataCount, double laser_angle_min, 
	       double laser_angle_max, double laser_angle_increment)
{
  const double threshold = 0.3; // 物体を識別するしきい値
  int center = dataCount/2;  // レーザーの中央                                   
  double init_search_dist  = 2.0; // 追跡する人の初期距離 [m]                        
  int search_lines = 1080 *  (follow_angle/270.0); // 走査線数
  int left=0, right=dataCount; // 走査線の左端、右端の番号
  int sum_no = 0, no=0; 
  target_distance = 999;
  double last_distance = 999;  // 1本前の走査線の距離
  int flag_begin = 999, flag_end = 999;
  int object_num = 0;

  for (int j = center - search_lines/2; j <= center + search_lines/2; j++) {
    if (laser_data[j] == 999) {
      continue;
    }
    double diff = last_distance - laser_data[j];
    last_distance = laser_data[j];

    //cout << "diff=" << diff << endl;

    if (diff >  threshold) {
      flag_begin = j;
      obj[object_num].begin = j;
    }
    if (diff < -threshold) {
      flag_end   = j;
      obj[object_num].end = j;
    }

    if ((flag_begin != 999) && (flag_end != 999)) {
      obj[object_num++].diff = flag_end - flag_begin;
      flag_begin = 999;
      flag_end   = 999;
    }
  }

  cout << "object_num=" << object_num << endl;

  for (int i=0; i < object_num; i++) {
    double sum_dist = 0, sum_num = 0;
    int count = 0;
    for (int j = obj[i].begin; j < obj[i].end; j++) {
      sum_dist += laser_data[j];
      sum_num  += j;
      count++;
    }
    obj[i].distance = sum_dist/(obj[i].diff + 1);
    int obj_center;
    if (count == 0) obj[i].angle = 999;
    else {
      obj_center  = sum_num/count;
      obj[i].angle
	= (laser_angle_max - laser_angle_min) * obj_center
	/ (double) dataCount + laser_angle_min;
    }
    obj[i].width = obj[i].distance * obj[i].diff * laser_angle_increment;
    if ((obj[i].distance <= follow_max_distance) && (obj[i].width <= leg_width_max) && (obj[i].width >= leg_width_min)) {
      obj[i].leg = true;
    }
    else obj[i].leg = false;

    //printf("sum_dist=%f count=%d\n",sum_dist,count);
    printf("obj[%d]=%f\n",obj[i]);
    printf("obj:%d distance=%f angle=%f width=%f diff=%d center=%d\n",
	   i, obj[i].distance,obj[i].angle*180/M_PI, obj[i].width,obj[i].diff,
	   obj_center);
  }
  
  int leg_count = 0;
  double angle_sum = 0.0, dist_sum = 0.0;
  for (int i=0; i < object_num; i++) {
    if (obj[i].leg == true) {
      leg_count++;
      angle_sum += obj[i].angle;
      dist_sum  += obj[i].distance;
    }
  }

  if (leg_count > 0) {
    human.angle   = angle_sum/leg_count;
    human.distance = dist_sum/leg_count;
  }
  else {
    human.angle   = 999;
    human.distance = 999;
  }

  printf("Human angle=%f distance=%f\n",human.angle*180/M_PI,human.distance);

  if (leg_count == 2) {
    cout << "Detect a human with two legs" << endl;
  }
  else if (leg_count < 0) {
    cout << "Error: findHuman" << endl;
  } 
  else if (leg_count == 0) {
    cout << "No detect" << endl;
  }
  else if (leg_count == 1) {
    cout << "Detect one leg" << endl;
  }
  else {
    cout << "Detect " << leg_count << endl;
  }


 
  /* if (fabs(last_distance - laser_data[j]) > threshold)
    else if (laser_data[j] < init_search_dist) {
      sum_no += j; // レーザー光線の番号を足す                                   
      if (target_distance > laser_data[j])  {
        target_distance = laser_data[j];
      }
    }
    no++;
    last_distance = laser_data[j];
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
  */

}



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
      laser_data[i] = 999; // invalid data
    }
  }

  findHuman(dataCount, laser_scan.angle_min, 
	    laser_scan.angle_max,laser_scan.angle_increment);

  //for(int i = 0; i < dataCount; i++) {
  //  ROS_INFO("myvalue[%d]:%f\n", i,laser_data[i]);                            
  //}       
  //printf("Target center=%d angle=%f\n",target_center, target_angle);
  //cout << "1:" << 
}

int main(int argc, char** argv)
{
  // laser
  ros::init(argc, argv, "dfollow");
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
  geometry_msgs::Twist cmd_speed, tmp_speed;

  cmd_speed.linear.x  = 0; // m/s
  cmd_speed.linear.y  = 0; // m/s
  cmd_speed.angular.z = 0; // rad

  ros::Rate loop_rate(33); // 5Hz

  double last_linear_speed  = 0;
  double last_angular_speed = 0;
  while(ros::ok()) { //have we ctrl + C?  If no... keep going!
    //"publish" sends the command to turtlebot to keep going
    // ROS_INFO("Target dist=%f angle=%f\n", target_distance, target_angle);

    if (human.distance == 999) cmd_speed.linear.x = 0;
    else if (((human.distance >=  follow_min_distance) &&
	      (human.distance <=  follow_max_distance))) {
      
      /* double tmp_speed = gain_linear * (follow_min_distance - human.distance);
      if (fabs(tmp_speed) > linear_max_speed) tmp_speed = linear_max_speed;
      if (tmp_speed < 0) tmp_speed = 0;
      cmd_speed.linear.x = tmp_speed; */

      if (follow_min_distance - human.distance > 0) 
	cmd_speed.linear.x -= 0.05;
      else
	cmd_speed.linear.x += 0.05;

      if (cmd_speed.linear.x > linear_max_speed) 
	cmd_speed.linear.x = linear_max_speed;
      if (cmd_speed.linear.x < - linear_max_speed)
	cmd_speed.linear.x = - linear_max_speed;
    }   
    else {
      cmd_speed.linear.x = 0;
    }

    // 5度以内のときは回転しない
    if (human.distance == 999) cmd_speed.angular.z = 0;
    else if (fabs(human.angle) > 10.0 * M_PI/ 180.0) {
      double tmp_speed = gain_turn * human.angle;
      if (tmp_speed > turn_max_speed) tmp_speed = turn_max_speed;
      else if (tmp_speed < -turn_max_speed) tmp_speed = -turn_max_speed;

      cmd_speed.angular.z =  tmp_speed;
    }
    else {
      cmd_speed.angular.z = 0;
    }

    // 滑らかに加減速。動きが遅くないうまくいかないので止める
    /* double tmp_lv, tmp_av, divide=2;
    tmp_speed.linear.x  = 0;
    tmp_speed.linear.y  = 0;
    tmp_speed.angular.z = 0;
    if (fabs(cmd_speed.linear.x - last_linear_speed) > 0.2) {
      for (int i=1; i <= divide; i++) {
	tmp_speed.linear.x = (cmd_speed.linear.x - last_linear_speed) * i/divide
	  +  last_linear_speed;
	tmp_speed.angular.z 
	  = (cmd_speed.angular.z - last_angular_speed)* i/divide
	  +  last_angular_speed;
	cmd_vel_pub_.publish(tmp_speed);
	usleep(5*1000*1000);
	ros::spinOnce();
      }
    } */

    cmd_vel_pub_.publish(cmd_speed);

    //last_linear_speed  = cmd_speed.linear.x;
    //last_angular_speed = cmd_speed.angular.z;
    //loop_rate.sleep();
    ros::spinOnce();
  }


  ROS_INFO("Finished\n");

  return 0;
}

