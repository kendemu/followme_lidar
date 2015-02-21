//http://marksilliman.com/2014/10/06/turtlebots-missing-hello-world-program/
/*
 * Hello World and Crash for Turtlebot
 * Moves Turtlebot forward until you ctrl + c
 * Tested using TurtleBot 2, ROS Indigo, Ubuntu 14.04
 */
/// Read home/kensei4/hydro/src/amcl/src/amcl_node.cpp
// 141210: LIDARのデータを画像化してOpenCVで処理する
// 141210: 切り出しがうまくいかず、凸壁を脚と間違える
// 141203: 何とか追跡できるようになったが、壁を追跡者と間違えたる
// To Do: 人と壁の判別、比例航法の実装

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "/usr/local/include/kobuki-1.0/libkobuki.h"
//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include "libkobuki.h"
using namespace std;
// アールティ用ロボットネームスペース
using namespace rt_net;

const double follow_max_distance = 1.0; // follow distance
const double follow_min_distance = 0.3; // follow distance 
const double leg_width_max = 0.3;   // 脚幅の最大値
const double leg_width_min = 0.1;  // 脚幅の最大値
const double follow_angle = 45;     // 探す範囲は正面のこの角度[deg]  
const double gain_linear = 0.5;            // 比例ゲイン
const double gain_turn  = 5.0;
const double linear_max_speed   = 0.5;
const double turn_max_speed     = 1.0;

double laser_distance[1080]; // hokuyo lidar UTM-30LX
double laser_intensities[1080];
double target_angle;     // target angle [rad] center is 0 [rad]
double target_distance;  // minimum distance from a robot

const int IMAGE_WIDTH=500, IMAGE_HEIGHT=500;
const double mToPixel = 50; // mをpixelへ変換 1m == 20 pixel

cv::RNG rng(12345); // 乱数発生器　12345は初期化の種


// 初期化時に塗りつぶす                                                    
cv::Mat lidar_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 
		  CV_8UC3, cv::Scalar::all(255));


//cv::Mat lidar_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
//		    CV_8UC3, 255);

//cv::Mat lidar_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
//		    IPL_DEPTH_8U, 3);
//cv::Mat lidar_image = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 
//			     CV_8UC3);


class Object {
public:
  bool leg;   // leg:true  or not:false
  int begin; // right side laser
  int end;   // left side laser
  int diff;
  double distance;
  double angle; // rad
  double width;
  double radius; // radius of an enclose circle
  cv::Point2f pos; // position of an image
};

Object obj[100], human;

void findHuman(int dataCount, double laser_angle_min, 
	       double laser_angle_max, double laser_angle_increment)
{
  const double threshold = 0.05; // 物体を識別するしきい値(距離の変化) [m]
  int center = dataCount/2;  // レーザーの中央の番号                                   
  double init_search_dist  = 2.0; // 追跡する人の初期距離 [m]                        
  int search_lines = 1080 *  (follow_angle/270.0); // 走査線数
  int left=0, right=dataCount; // 走査線の左端、右端の番号
  int sum_no = 0, no=0; 
  double last_distance = 999;  // 1本前の走査線の距離
  int flag_begin = 999, flag_end = 999; // 走査線の番号がわかる
  int object_num = 0;

  for (int j = center - search_lines/2; j <= center + search_lines/2; j++) {
    if (laser_distance[j] == 999) {
      continue;
    }
    double diff = last_distance - laser_distance[j];
    last_distance = laser_distance[j];

    //cout << "diff=" << diff << endl;
    // 距離の差がthresholdより大きいと脚でないとみなす
    if (diff >  threshold) { // 凸形状始まり
      flag_begin = j;
      obj[object_num].begin = j;
    }
    if (diff < -threshold) { // 凸形状終わり
      flag_end   = j;
      obj[object_num].end = j;
    }

    if (flag_begin != 999){
      if (flag_end != 999) {
	obj[object_num++].diff = flag_end - flag_begin;
	flag_begin = 999; // カウンターをリセット
	flag_end   = 999;
      }
      else {
	// 終端処理。途中(flag_begin==999)から終端まで物体がある場合
       	if(j == center + search_lines/2) {
	  obj[object_num++].diff = j - flag_begin;
	}
      }
    }
  }

  cout << "object_num=" << object_num << endl;

  for (int i=0; i < object_num; i++) {
    double sum_dist = 0, sum_num = 0;
    int count = 0;
    for (int j = obj[i].begin; j < obj[i].end; j++) {
      sum_dist += laser_distance[j];
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
    //printf("obj[%d]=%f\n",i,obj[i]);
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
}

// LIDARデータを画像に変換する
void changeToPicture(int dataCount, double laser_angle_min,
               double laser_angle_max, double laser_angle_increment)
{

  for (int j=0; j < lidar_image.rows; j++) {
    for (int i = 0; i < lidar_image.cols; i++) {
      lidar_image.data[j*lidar_image.step+i*lidar_image.elemSize()  ] = 255;
      lidar_image.data[j*lidar_image.step+i*lidar_image.elemSize()+1] = 255;
      lidar_image.data[j*lidar_image.step+i*lidar_image.elemSize()+2] = 255;
    }
  } 

  int center = dataCount/2;  // レーザーの中央の番号                                 
  double init_search_dist  = 2.0; // 追跡する人の初期距離 [m]                        
  int search_lines = 1080 *  (follow_angle/270.0); // 走査線数


  for (int j = center - search_lines/2; j <= center + search_lines/2; j++) {
    //for (int j = 0; j <= dataCount; j++) {
    int x=0, y=0, tmp=0;
    if (laser_distance[j] <= 0.5*IMAGE_WIDTH*sqrt(2)/mToPixel) {
      double angle = (laser_angle_max - laser_angle_min) * j
	/ (double) dataCount + laser_angle_min;
      x = mToPixel * laser_distance[j]*cos(angle) + (int) (0.5 * IMAGE_WIDTH);
      y = mToPixel * laser_distance[j]*sin(angle) + (int) (0.5 * IMAGE_HEIGHT);
      tmp = x;
      x   = lidar_image.cols- y;   // x軸も左右反転 画面は左隅が0,0
       
      y   = lidar_image.rows - tmp; // y軸は上下反転
    }

    //cout << "intensity=" << laser_intensities[j] << endl;

    //std::cout << " x="<< x << " y=" << y << std::endl;
    if ((0 <= x) && (x < lidar_image.cols) && (0 <= y) && (y < lidar_image.rows)) {
      // fast version
      int value = (int) (laser_intensities[j] * 255.0/6000.0);
      if (value > 255)  value = 255;
      if (value <   0)  value =   0;
      lidar_image.data[y*lidar_image.step+x*lidar_image.elemSize()  ] = 0;
      lidar_image.data[y*lidar_image.step+x*lidar_image.elemSize()+1] = 0;
      //lidar_image.data[y*lidar_image.step+x*lidar_image.elemSize()+2] = 255;
      lidar_image.data[y*lidar_image.step+x*lidar_image.elemSize()+2] 
	= value;
      //cout << "intensity[" << j << "]=" << value << endl;
      // slow version
      //lidar_image.at<cv::Vec3b>(y,x) = cv::Vec3b(255,0,0);
    }
  }
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
      laser_distance[i] = value;
      laser_intensities[i] = laser_scan.intensities[i];
    }
    else {
      laser_distance[i] = 999; // invalid data
      laser_intensities[i] = -999;
    }
  }

  //findHuman(dataCount, laser_scan.angle_min, 
  //	    laser_scan.angle_max,laser_scan.angle_increment);

  changeToPicture(dataCount, laser_scan.angle_min,
            laser_scan.angle_max,laser_scan.angle_increment);

  //cv::imshow("lidar image", lidar_image);
  //cv::waitKey();
  //for(int i = 0; i < dataCount; i++) {
  //  ROS_INFO("myvalue[%d]:%f\n", i,laser_distance[i]);                            
  //}       
  //printf("Target center=%d angle=%f\n",target_center, target_angle);
  //cout << "1:" << 
}

int findLegs(float thresh_min, float thresh_max, cv::Mat input_image, Object *object, cv::Mat display_image,
	      const string& winname, cv::Scalar color)
{
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  // http://stackoverflow.com/questions/22930605/remove-circles-using-opencv                                                

  findContours(input_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0 , 0) );            
  //findContours(e2_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0 , 0) );

  //cout << "countour number=" << contours.size() << endl;
  int object_num = 0;

  for(unsigned int cn=0; cn<contours.size(); cn++)
    {
      cv::Point2f center;
      float radius;
      // find minimum circle enclosing the contour                                                                          
      cv::minEnclosingCircle(contours[cn],center,radius);
      
      if ((radius < thresh_min) || (radius >  thresh_max)) continue;
      
      cv::Rect circleROI(center.x-radius, center.y-radius, center.x+radius, center.y+radius);
      
      // count pixel inside the circle                                                                                      
      float sumCirclePixel = 0;
      float sumCirclePixelMarked = 0;
      
      for(int j=circleROI.y; j<circleROI.y+circleROI.height; ++j)
	{
	  for(int i=circleROI.x; i<circleROI.x+circleROI.width; ++i)
	    {
	      cv::Point2f current(i,j);
	      // test if pixel really inside the circle:                                                                    
	      if(cv::norm(current-center) < radius)
		{
		  sumCirclePixel++;
		  if((int) input_image.at<unsigned char>(j,i) == 0)
		    sumCirclePixelMarked += 1.0;
		}
	    }
	}
      
      const float ratioThreshold = 0.1625f;
      float ratio;
      if (sumCirclePixel == 0) ratio = 0.0f;
      else ratio =  sumCirclePixelMarked/sumCirclePixel;
      //float ratio = sumCirclePixelMarked/(2*3.14*radius);                                                                 
      if( ratio >= ratioThreshold) {
	cv::circle(display_image,center,radius,color,-1);
	//cv::cvCircle(display_image,center,radius,color,1);
	//cout << "no="<< cn << " x=" << center.x << " y=" << center.y << " radius=" << radius 
	//<< " count=" << sumCirclePixel	 << " ratio="<< ratio << endl;
	object[object_num].radius = radius;
	object[object_num].pos = center;
	object_num++;
      }
    }

  cout << "Object num=" << object_num << endl;
  cv::imshow(winname, display_image);
  return object_num;

}

// main関数
// rosのmobile_baseは遅いので、rtのライブラリを使用した
int main(int argc, char* argv[])
{
  std::cout << "Starting Followme Demo" << std::endl;
  //std::string arg = "/def/ttyUSB0";
  std::string arg="/dev/kobuki";
  if(argc > 1) {
    arg = argv[1];
  }
  Kobuki *kobuki = createKobuki(KobukiStringArgument(arg));
  if (kobuki == NULL) std::cout << "Open Error:/dev/kobuki \n";
  std::cout << "Starting Followme Demo" << std::endl;
  std::cout << "follow_max_distance =" << follow_max_distance << std::endl;
  std::cout << "follow_min_distance =" << follow_min_distance << std::endl;
  std::cout << "follow_angle        =" << follow_angle << std::endl;
  std::cout << "leg_width_max       =" << leg_width_max << std::endl;
  std::cout << "leg_width_min       =" << leg_width_min << std::endl;

  // laser                                                                                
  ros::init(argc, argv, "dfollow");
  ros::NodeHandle node;
  //ros::Subscriber hokuyoSubscriber = node.subscribe("/scan", 1, scanValues);            
  ros::Subscriber laserSubscriber = node.subscribe("/scan", 100, laserCallback);

  //init direction that turtlebot should go
  geometry_msgs::Twist cmd_speed, tmp_speed;
  cmd_speed.linear.x  = 0; // m/s
  cmd_speed.linear.y  = 0; // m/s
  cmd_speed.angular.z = 0; // rad                                                         


  // OpenCV
  //char windowName[]="Lidar";
  //IplImage *image = cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
  //cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);

  //while (1) {
  //  cvShowImage(windowName, image);
  //}

  // 初期化後に塗りつぶす
  cv::namedWindow("lidar image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  //cv::waitKey(0);

  // 背景差分計算用オブジェクトの生成
  cv::BackgroundSubtractorGMG backGroundSubtractor;
  //cv::BackgroundSubtractorMOG backGroundSubtractor;
  //cv::BackgroundSubtractorMOG2 backGroundSubtractor;

  ros::Rate loop_rate(33); // 33Hz     

  int loop=0;
  while(ros::ok()) {
    cv::Mat white_image1(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
			 CV_8UC3, cv::Scalar::all(255));
    cv::Mat white_image2(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
			 CV_8UC3, cv::Scalar::all(255));
    cv::Mat white_image3(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
                         CV_8UC3, cv::Scalar::all(255));

    cv::Mat color_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
                         CV_8UC3, cv::Scalar::all(255));



    cv::Mat frame, foreGroundMask, output;

    //cv::imshow("lidar image", lidar_image);

    cv::Mat d1_img, d3_img, e1_img, e2_img, e3_img, tmp_img,final_img;
    cv::Mat e1_bin_img, e2_bin_img, e3_bin_img;
    cv::Mat e2d1_img,e2_color_img;
    cv::Mat lidar_gray_image,lidar_bin_image, rec_img;
    cv::Mat bin_img1, bin_img2, bin_img3;

    // グレースケールに変換する
    cvtColor(lidar_image, lidar_gray_image, CV_RGB2GRAY);

    // gray scale -> binary                                                           
    cv::threshold(lidar_gray_image, lidar_bin_image, 0, 255,
                  cv::THRESH_BINARY| cv::THRESH_OTSU);

    cv::dilate(lidar_bin_image, d1_img, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(lidar_bin_image, d3_img, cv::Mat(), cv::Point(-1,-1), 3);
    cv::erode(lidar_bin_image, e1_img, cv::Mat(), cv::Point(-1,-1), 1);
    cv::erode(lidar_bin_image, e2_img, cv::Mat(), cv::Point(-1,-1), 2);
    cv::erode(lidar_bin_image, e3_img, cv::Mat(), cv::Point(-1,-1), 3);

    cv::threshold(e2_img, e2_bin_img, 0, 255,
                  cv::THRESH_BINARY| cv::THRESH_OTSU);

    cv::dilate(e2_img, e2d1_img, cv::Mat(), cv::Point(-1,-1), 1);


    // カラーに変換する
    cvtColor(e2_img, e2_color_img, CV_GRAY2BGR);


    cv::imshow("lidar original image", lidar_bin_image);
    //cv::imshow("e2->d1 image", e2d1_img);
    cv::imshow("e2 image", e3_img);


    // http://stackoverflow.com/questions/22930605/remove-circles-using-opencv
    //cv::Scalar colorMarked(255,0,0);

    /// Find contours                           
    // std::vector<std::vector<cv::Point> > contours;
    // std::vector<cv::Vec4i> hierarchy;
    //findContours(lidar_bin_image, contours, hierarchy, 
    //   CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );

    // To find legs
    cv::Scalar red(0,0,255), blue(255,0,0);

    //findLegs(lidar_bin_image, lidar_image, "Cirlce 1", red);
    //findLegs(e2_img, e2_color_image, "Circle 2", blue);
    //if ((radius < 5) || (radius > 30)) continue;                                                                          
    float leg_radius = 0.05 * mToPixel;

    //findLegs(leg_radius, 3 * leg_radius, lidar_bin_image, white_image1, "Cirlce 1", red);
    //findLegs(leg_radius, 3 * leg_radius, e1_img, white_image1, "Cirlce 1", red);
    //findLegs(5.0 * leg_radius, 1000, e3_img, white_image2, "Circle 2", blue);

    Object obj1[100], obj2[100];
    int obj1_num = 0, obj2_num = 0, obj_num = 0;
    obj1_num = findLegs(leg_radius, 3 * leg_radius, lidar_bin_image, obj1, white_image1, "Cirlce 1", red);
    obj2_num = findLegs(5.0 * leg_radius, 1000, e2_img, obj2, white_image2, "Circle 2", blue);
    //findLegs(5.0 * leg_radius, 1000, e3_img, color_image, "Circle 2", blue);
    cout << "obj1 num=" << obj1_num << " obj2_num=" << obj2_num << endl;
    
    for (int i=0; i < obj1_num; i++) {
      bool inside = false;
      for (int j=0; j < obj2_num; j++) {
	double dist = cv::norm(obj1[i].pos - obj2[j].pos);
	if (dist <= obj2[j].radius) {
	  inside = true;
	} 
      }
      if (inside == false) {
	obj[obj_num].pos = obj1[i].pos;
	obj[obj_num].radius = obj1[i].radius;
	cv::circle(white_image3,obj[obj_num].pos,obj[obj_num].radius,red,1);
	cout <<"pos=(" << obj[obj_num].pos.x << "," << obj[obj_num].pos.y << ")" << 
	  " radius=" << obj[obj_num].radius << endl;
	obj_num++;
      }
    }
    cout << "obj1 num="<< obj1_num << " obj2_num=" << obj2_num<< " obj_num=" << obj_num << endl;


    //cv::threshold(white_image1, bin_img1, 0, 255, cv::THRESH_BINARY| cv::THRESH_OTSU);
    //cv::threshold(white_image2, bin_img2, 0, 255, cv::THRESH_BINARY| cv::THRESH_OTSU);
    //cv::bitwise_and(bin_img1,bin_img2,bin_img3);
    cv::imshow("Detected Legs",white_image3);




  //cv::imshow("erode",e2_img);

    // gray scale -> binary
    //cv::threshold(lidar_gray_image, lidar_bin_image, 0, 255, 
    //		  cv::THRESH_BINARY| cv::THRESH_OTSU);
    //cv::threshold(e2_img, lidar_bin_image, 0, 255,
    //              cv::THRESH_BINARY| cv::THRESH_OTSU);

    // マスク画像の取得
    //backGroundSubtractor(e1_img, foreGroundMask);

    // 差分：入力画像にマスク処理を行う
    //cv::bitwise_and(e1_img, e1_img, output, foreGroundMask);
    //cv::erode(foreGroundMask, final_img, cv::Mat(), cv::Point(-1,-1), 1);
    //cv::imshow("lidar image", lidar_image);

    /// Find contours
    //std::vector<std::vector<cv::Point> > contours;
    //std::vector<cv::Vec4i> hierarchy;
    //cv::dilate(lidar_image, d1_img, cv::Mat(), cv::Point(-1,-1), 1);
    //cv::erode(lidar_img, tmp_img, cv::Mat(), cv::Point(-1,-1), 3);

    // findCountoursは２値画像のみ
    //findContours(lidar_bin_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    ////findContours(e3_img, contours, hierarchy, 
    ////		 CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    // CV_CHAIN_CODE 出力はフリーマンチェーンコード． 他手法は，ポリゴン（頂点のシーケンス）を出力します．
    //CV_CHAIN_APPROX_NONE チェーンコードの全ての点を通常の点群に変換
    //CV_CHAIN_APPROX_SIMPLE 水平・垂直・斜めの線分を圧縮し，それらの端点のみを残します．
    //CV_CHAIN_APPROX_TC89_L1,CV_CHAIN_APPROX_TC89_KCOS Teh-Chin チェーン近似アルゴリズムの1つを適用します．
    //CV_LINK_RUNS 値が1のセグメントを水平方向に接続する，全く異なる輪郭抽出アルゴリズム．この手法を用いる場合は，抽出モードが CV_RETR_LIST でなければいけません．
    /// Draw contours
    /* 
    cv::Mat drawing = cv::Mat::zeros(lidar_image.size(), CV_8UC3 );

    vector<cv::Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    cv::Point2d point, min_point;
    double dist, min_dist = 100000, min_contour = 0;
    cout << "contour size:" << contours.size() << endl;
    for( int i = 0; i< contours.size(); i++ )   {
      double a=contourArea( contours[i],false);
      cv::Scalar color = cv::Scalar( rng.uniform(0, 255), 
		   rng.uniform(0,255), rng.uniform(0,255) );

      minEnclosingCircle( (cv::Mat)contours[i], center[i], radius[i]);
      cout << "radius" << radius[i] << endl;
      if (radius[i] < mToPixel * 0.5 )  // 100pixel
      {
	  circle(drawing, center[i], (int)radius[i], color, 2, 8, 0 );
	  drawContours(drawing, contours, i, color, 2, 8, hierarchy, 
		       0, cv::Point() );
      }

      cv::Moments mom = cv::moments(contours[i]);
      point.x = mom.m10/mom.m00;
      point.y = mom.m01/mom.m00;
      //cout << "countour["<<i<<"] area=" << a ;
      //cout << " x=" << point.x << " y=" << point.y << endl;
      dist = (point.x - IMAGE_WIDTH/2) * (point.x -IMAGE_WIDTH/2)
	   + (point.y - IMAGE_HEIGHT/2) * (point.y -IMAGE_HEIGHT/2);
      if (sqrt(dist) < min_dist) {
	min_dist = sqrt(dist);
	min_contour = i;
	min_point.x = point.x;
	min_point.y = point.y;
      }
    } 

    min_dist = min_dist / mToPixel;
    double min_angle = atan2(point.x - IMAGE_WIDTH/2, IMAGE_HEIGHT/2 - point.y);
    cout << "Min contour=" << min_contour ;
    cout << " dist=" << min_dist << " angle=" << min_angle * 180/M_PI << endl;
    human.distance  = min_dist;
    human.angle     = min_angle; 
    
    cv::Vec3b p = lidar_image.at<cv::Vec3b>(min_point.y,min_point.x);
    // 赤成分にintensityが格納されている　0~6000 -> 0~255;
    cout << "intensity=" << p(2) << endl;
    */     


    //cv::drawContours(drawing, contours, -1, cv::Scalar(0,0,255), 2);
    // iterate through each contour.
    /* int largest_area=0;
    int largest_contour_index=0;
    cv::Rect bounding_rect;
    cv::Mat drawing = cv::Mat::zeros( lidar_image.size(), CV_8UC3 ); 
    cv::Scalar color( 255,255,255);  // color of the contour in the     
    for( int i = 1; i< contours.size(); i++ )
      {
        //  Find the area of contour
        double a=contourArea( contours[i],false); 
	cout << i << "area="  << a << endl;
	if (a < 200) {
	  if(a>largest_area){
	    largest_area=a;cout<<i<<" area  "<<a<<endl;
	    // Store the index of largest contour
	    largest_contour_index=i;               
	    // Find the bounding rectangle for biggest contour
	    bounding_rect=boundingRect(contours[i]);
	  }
	  drawContours(drawing, contours,i, 
		       color, CV_FILLED,8,hierarchy);
	}
	} */
    //cv::Scalar color( 255,255,255);  // color of the contour in the
    //Draw the contour and rectangle
    //drawContours(drawing, contours,largest_contour_index, color, CV_FILLED,8,hierarchy);
    //cv::rectangle(drawing, bounding_rect, cv::Scalar(0,255,0),2, 8,0);
    //cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
    //imshow( "Display window", drawing);    


    //const int drawAllContours = -1;
    //cv::drawContours(drawing, contours,
    //		     drawAllContours,cv::Scalar(0));


    //cv::imshow("lidar image", lidar_image);
    //cv::imshow("output", final_img);

    //cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    //cv::imshow( "drawing", drawing);
    cv::waitKey(1);

    //std::cout << "loop:"<< loop++ <<std::endl;
    // 999 はエラーなので速度を0にセット
    if (human.distance == 999) cmd_speed.linear.x = 0;
    // 人が追跡距離内にいる場合
    else if (((human.distance >=  follow_min_distance) &&
              (human.distance <=  follow_max_distance))) {

      if (follow_min_distance - human.distance > 0)
	cmd_speed.linear.x -= 0.01;  // 近づきすぎ
      else
        cmd_speed.linear.x += 0.01;  // 離れすぎ

      // 速度の上限と下限を設定
      if (cmd_speed.linear.x >   linear_max_speed)
        cmd_speed.linear.x = linear_max_speed;
      if (cmd_speed.linear.x < - linear_max_speed)
        cmd_speed.linear.x = - linear_max_speed;
    }
    // 人が追跡距離外に出た場合は停止する
    else {
      cmd_speed.linear.x = 0;
    }

    // 5度以内のときは回転しない 
    if (human.distance == 999) cmd_speed.angular.z = 0;
    else if (fabs(human.angle) > 5.0 * M_PI/ 180.0) {
      if (human.angle < 0) 
	cmd_speed.angular.z -= 0.015;  // 近づきすぎ
      else
        cmd_speed.angular.z += 0.015;  // 離れすぎ

      if (cmd_speed.angular.z >  turn_max_speed) 
	cmd_speed.angular.z =  turn_max_speed;
      if (cmd_speed.angular.z < -turn_max_speed) 
	cmd_speed.angular.z = -turn_max_speed;
    }
    else {
      cmd_speed.angular.z = 0;
    }

    cmd_speed.linear.x = 0;
    cmd_speed.angular.z = 0;
    kobuki->setTargetVelocity(cmd_speed.linear.x,cmd_speed.angular.z);

    //loop_rate.sleep(); 
    usleep(1*1000); 
    ros::spinOnce();
  }

  ROS_INFO("Finished\n");
 
  delete kobuki;
  return 0;
}
