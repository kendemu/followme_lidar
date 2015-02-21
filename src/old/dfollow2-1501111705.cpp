// Read home/kensei4/hydro/src/amcl/src/amcl_node.cpp
// 150104: 輪郭抽出により物体を取り出し、その重心、面積、モーメントにより
//         脚と思われる物体を推定している。パラメータの調整が不十分。
//         
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
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#define MOMENT_EXEL // 慣性モーメントデータ取得用
//#define DEBUG       // デバッグ用
//#define RECORD  // 録画用
//#define NO_MOVE  // no move

using namespace std;
// アールティ用ロボットネームスペース
using namespace rt_net;

using namespace cv;

const double follow_max_distance = 4.0; // follow distance
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
  static int64 epochs = 0; 

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
    //if (laser_distance[j] <= 0.5*IMAGE_WIDTH*sqrt(2)/mToPixel) {
    double angle = (laser_angle_max - laser_angle_min) * j
      / (double) dataCount + laser_angle_min;
    x = mToPixel * laser_distance[j]*cos(angle) + (int) (0.5 * IMAGE_WIDTH);
    y = mToPixel * laser_distance[j]*sin(angle) + (int) (0.5 * IMAGE_HEIGHT);
    tmp = x;
    x   = lidar_image.cols- y;   // x軸も左右反転 画面は左隅が0,0
    y   = lidar_image.rows - tmp; // y軸は上下反転
    //}

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

}

int findLegs(float thresh_min, float thresh_max, cv::Mat input_image, Object *object, cv::Mat display_image,
	      const string& winname, cv::Scalar color)
{
  static int epochs = 0;

  // Measure time begin
  int64 time = cv::getTickCount();

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  findContours(input_image, contours, hierarchy, 
	       CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0 , 0) );            
  //findContours(e2_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0 , 0) );

  //cout << "countour number=" << contours.size() << endl;
  int object_num = 0;

  for(unsigned int cn=0; cn<contours.size(); cn++)
    {
      cv::Point2f center;
      float radius;

      // find minimum circle enclosing the contour
      cv::minEnclosingCircle(contours[cn],center,radius);
      
      // 半径による除外
      //if ((radius < thresh_min) || (radius >  thresh_max)) continue;
      
      // 面積による除外(m00)
      int area_min = 35, area_max = 90; // pixel
      cv::Moments mom = cv::moments(contours[cn]);
      if (!((mom.m00 > area_min) && (mom.m00 < area_max))) continue;

      // m01
      int m01_min = 0, m01_max = 20000;
      if (!((mom.m01 > m01_min) && (mom.m01 < m01_max))) continue;

      // m10
      int m10_min = 9000, m10_max = 22000;
      if (!((mom.m10 > m10_min) && (mom.m10 < m10_max))) continue;

      // m11 
      int m11_min = 0, m11_max = 5000000;
      if (!((mom.m11 > m11_min) && (mom.m11 < m11_max))) continue;

      // m02                                                                                                   
      int m02_min = 0, m02_max = 4000000;
      if (!((mom.m02 > m02_min) && (mom.m02 < m02_max))) continue;

      // m20                                                                                                   
      int m20_min = 2000000, m20_max = 5500000;
      if (!((mom.m20 > m20_min) && (mom.m20 < m20_max))) continue;

      // m30                                                                                                   
      float m30_min = 5E+8, m30_max = 1.4E+9;
      if (!((mom.m30 > m30_min) && (mom.m30 < m30_max))) continue;


      // 密度による除外
      //float density_thresh = 0.25f;
      //float density = mom.m00/(M_PI * radius * radius);
      //if (density < density_thresh) continue;

      // 重心による判定
      // 脚（円柱）の断面はU字型なので重心のy座標が円より下になる
      // x座標は中心近辺。中心からずれている脚は追わない 
      //float y_thresh = 0.2;
      Point2f point;
      point.x = mom.m10/mom.m00;
      point.y = mom.m01/mom.m00;

      float diff_x = 1.0;
      if (fabs(center.x-point.x) > diff_x) continue;

      float diff_y = - 0.2;
      if(center.y - point.y > diff_y)  continue;
      
      //if ((210 <center.y ) && (center.y < 220)) {
      // if (160 < center.y) {
      #ifdef MOMENT_EXEL
      printf("Epochs=%d,", epochs++);
      printf("Contour[%d],(, %.0f, %.0f,), ",cn, center.x, center.y);
      printf("  m00=,%.1f, m01=,%.1f, m10=,%.1f, m11=,%.1f, m02=,%.1f, m20=,%.1f,",mom.m00, mom.m01, mom.m10, mom.m11, mom.m02, mom.m20);
      printf("  m12=,%.1f, m21=,%.1f, m03=,%.1f, m30=,%.1f,", mom.m12, mom.m21, mom.m03, mom.m30);
      printf("  center.x-point.x=,%f, center.y-point.y=,%f\n", center.x-point.x, center.y-point.y);
      #endif

      //} 
      //printf("  density=%f\n", density);
      //printf("  point.y-center.y=%f\n", point.y - center.y);
      //printf("  fabs(point.x-center.x)=%f", fabs(point.x-center.x));
      //}

      // 慣性モーメントによる判定
      // m11の情報を使う
      
      //cout << "m00(area)=" << mom.m00 << "m11=" << mom.mu11 << endl; 
      //float ratio = sumCirclePixelMarked/(2*3.14*radius); 
      //if((point.y > center.y + 0.2) && (fabs(point.x - center.x) < 0.2 * radius))  {
      //if((point.y > center.y + 0.2 ) && (fabs(point.x - center.x) < 0.2 * radius) && (ratio >= ratioThreshold))  {
      // m00 is area
      //if((point.y > center.y + 0.2 ) && (fabs(point.x - center.x) < 0.5 * radius)
      //   && (mom.m00 > 50) && (mom.m00 < 120))  {

	//cout << "m00(area)=" << mom.m00 << "m11=" << mom.mu11 << endl;

	cv::circle(display_image,center,radius,color,1);
        //cv::cvCircle(display_image,center,radius,color,1);                          
        //cout << "no="<< cn << " cx=" << center.x << " cy=" << center.y << " my=" << point.y <<
	//  " diffy=" << point.y - center.y << " radius=" << radius  <<  endl;
	//cout << " diffx=" << point.x - center.x << endl;
        object[object_num].radius = radius;
        object[object_num].pos    = center;
        object_num++;

     // 密度による判定
      /* const float ratioThreshold = 0.25f; //0.4f, 0.1625f;
      float ratio;
      if (sumCirclePixel == 0) ratio = 0.0f;
      else ratio =  sumCirclePixelMarked/sumCirclePixel;
      //float ratio = sumCirclePixelMarked/(2*3.14*radius);                                                                 
      if( ratio >= ratioThreshold) {
	cv::circle(display_image,center,radius,color,1);
	//cv::cvCircle(display_image,center,radius,color,1);
	cout << "no="<< cn << " x=" << center.x << " y=" << center.y << " radius=" << radius 
	<< " count=" << sumCirclePixel	 << " ratio="<< ratio << endl;
	object[object_num].radius = radius;
	object[object_num].pos = center;
	object_num++;
	} */
    }

  printf("FindLeg Time=%f[ms]\n", (cv::getTickCount()-time)*1000/cv::getTickFrequency());

  //cout << "Object num=" << object_num << endl;
  //cv::imshow(winname, display_image);
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


  cv::Mat img(cv::Size(500, 500), CV_8UC3, cv::Scalar(0, 0, 255));
  #ifdef RECORD
  // （1）動画ファイルを書き出すの準備を行う
  cv::VideoWriter writer("videofile.avi", CV_FOURCC_MACRO('X', 'V', 'I', 'D'), 30.0, cv::Size(500, 500), true);

  // （2）動画ファイル書き出しの準備に成功したかチェックする（失敗していればエラー終了する）
  if(!writer.isOpened())
    return -1;
  #endif

  // 初期化後に塗りつぶす
  cv::namedWindow("lidar image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);


  ros::Rate loop_rate(33); // 33Hz     

  int loop=0;

  while(ros::ok()) {
    // Measure time begin 
    int64 time = cv::getTickCount();

    cv::Mat white_image1(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
			 CV_8UC3, cv::Scalar::all(255));
    cv::Mat white_image2(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
			 CV_8UC3, cv::Scalar::all(255));
    cv::Mat white_image3(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
                         CV_8UC3, cv::Scalar::all(255));
    cv::Mat color_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
                         CV_8UC3, cv::Scalar::all(255));


    cv::Mat frame, foreGroundMask, output;

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
    // 脚を見つけるために原画像で連結領域を探す
    // 調整するパラメータ１番目と２番目。脚断面の半径のピクセル数(1: 最小、２：最大)
    //obj1_num = findLegs(0.1*mToPixel, 0.2*mToPixel, lidar_bin_image, obj1, lidar_image, "Cirlce 1", red);

    //  obj1_num = findLegs(0.1*mToPixel, 0.3*mToPixel, e1_img, obj1, lidar_image, "Cirlce 1", red);
    obj1_num = findLegs(leg_radius, 6 * leg_radius, e1_img, obj1, lidar_image, "Cirlce 1", red);
    //obj1_num = findLegs(leg_radius, 3 * leg_radius, e1_img, obj1, white_image1, "Cirlce 1", red);
    //obj1_num = findLegs(leg_radius, 3 * leg_radius, lidar_bin_image, obj1, white_image1, "Cirlce 1", red);
    

    // 大きな物体を除外するために２回膨張させた画像から連結領域を探す
    //obj2_num = findLegs(5.0 * leg_radius, 1000, e2_img, obj2, white_image2, "Circle 2", blue);
    //findLegs(5.0 * leg_radius, 1000, e3_img, color_image, "Circle 2", blue);
    /* cout << "obj1 num=" << obj1_num << " obj2_num=" << obj2_num << endl;
    for (int i=0; i < obj1_num; i++) {
      bool inside = false;
      // 大きな円を除外する
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
    cout << "obj1 num="<< obj1_num << " obj2_num=" << obj2_num<< " obj_num=" << obj_num << endl;  */

    //cv::threshold(white_image1, bin_img1, 0, 255, cv::THRESH_BINARY| cv::THRESH_OTSU);
    //cv::threshold(white_image2, bin_img2, 0, 255, cv::THRESH_BINARY| cv::THRESH_OTSU);
    //cv::bitwise_and(bin_img1,bin_img2,bin_img3);
    //cv::imshow("Blur image", blur_image);
    cv::namedWindow( "Detected Legs", CV_WINDOW_AUTOSIZE );
    cv::imshow("Detected Legs",lidar_image);

    // 脚と推定される最も近い物体を追跡する
    double min_dist = 999999999, min_angle = 999999999, tmp_dist;
    Point2f min_point;

    #ifdef DEBUG
    cout << "Object num=" << obj1_num << endl;
    #endif

    for (int i=0; i < obj1_num; i++) {
      tmp_dist = (obj1[i].pos.x - IMAGE_WIDTH/2) * (obj1[i].pos.x -IMAGE_WIDTH/2)
		      + (obj1[i].pos.y - IMAGE_HEIGHT/2) * (obj1[i].pos.y -IMAGE_HEIGHT/2);
      if (tmp_dist < min_dist) {
	min_dist = tmp_dist;
	min_point.x = obj1[i].pos.x;
	min_point.y = obj1[i].pos.y;
      }
    }

   if (min_dist == 999999999) {
      human.distance = 999;
      human.angle    = 999;
    }
    else {
      min_dist = sqrt(min_dist) / mToPixel;
      double min_angle = atan2(min_point.x - IMAGE_WIDTH/2, IMAGE_HEIGHT/2 - min_point.y);
      #ifdef DEBUG
      cout << " dist=" << min_dist << " angle=" << min_angle * 180/M_PI << endl; 
      #endif
      human.distance  = min_dist; 
      human.angle     = min_angle;    
    }

   #ifdef DEBUG
    cout << "human dist=" << human.distance << " angle=" << human.angle << endl;
   #endif 

    cv::waitKey(1);

    //std::cout << "loop:"<< loop++ <<std::endl;
    // 999 はエラーなので速度を0にセット
    if (human.distance == 999) cmd_speed.linear.x = 0;
    // 人が追跡距離内にいる場合
    else if (((human.distance >=  follow_min_distance) &&
              (human.distance <=  follow_max_distance))) {

      if (follow_min_distance - human.distance > 0)
	cmd_speed.linear.x -= 0.02;  // 近づきすぎ
      else
        cmd_speed.linear.x += 0.02;  // 離れすぎ

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

    #ifdef NO_MOVE
    cmd_speed.linear.x = 0;
    cmd_speed.angular.z = 0;
    kobuki->setTargetVelocity(cmd_speed.linear.x,cmd_speed.angular.z);
    #endif

    // 動画を取るときは以下をコメントアウトする
    // writer << 取り込みたイメージ名
    #ifdef RECORD
    writer << lidar_image;
    #endif

    loop_rate.sleep(); 
    //usleep(1*1000); 
    ros::spinOnce();
    printf("Main Loop=%f[ms]\n", (double) (cv::getTickCount()-time)*1000/(cv::getTickFrequency()));
  }

  ROS_INFO("Finished\n");
 
  delete kobuki;
  return 0;
}
