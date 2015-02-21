// Read home/kensei4/hydro/src/amcl/src/amcl_node.cpp
// 150114: LIDAR反射強度の導入
//         比例航法導入
// 150112: LIDAR画像を安定させるために１時刻前の画像と現在の画像のANDを取るようにした
//         識別のパラメータ調整
// 150111: 高速化のためlidarのデータをgray scaleに格納。輝度はintensity情報
//         追跡時にふらつく原因は、一番近い物体を追跡しているため。
//         2個物体を見つけた時はその重心を物体の位置とするように変更。
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
#define RECORD      // 録画用
//#define MOVE       //  動かさないときはコメントアウト
#define PROPORTIONAL_NAVI /// 比例航法でないときはコメントアウト

using namespace std;
// アールティ用ロボットネームスペース
using namespace rt_net;

using namespace cv;

const double follow_max_distance = 4.0; // follow distance
const double follow_distance = 2.5;     // follow distance
const double follow_min_distance = 0.5; // follow distance 
const double follow_speed = 0.7;        // 追跡するときの基準速度
const double leg_width_max = 0.3;   // 脚幅の最大値
const double leg_width_min = 0.1;  // 脚幅の最大値
const double follow_angle = 180;     // 探す範囲は正面のこの角度[deg]  
const double gain_linear = 2.0;            // 比例ゲイン
const double gain_turn  = 15;
const double gain_proportion  = 100;  // 比例航法のゲイン
const double linear_max_speed   = 0.7;
const double turn_max_speed     = 3.14;

double laser_distance[1080]; // hokuyo lidar UTM-30LX
double laser_intensities[1080];
double target_angle;     // target angle [rad] center is 0 [rad]
double target_distance;  // minimum distance from a robot

const int IMAGE_WIDTH=500, IMAGE_HEIGHT=500;
const double mToPixel = 50; // mをpixelへ変換 1m == 20 pixel, 1 pixel = 5cm
//const int IMAGE_WIDTH=500, IMAGE_HEIGHT=500;
//const double mToPixel = 50; // mをpixelへ変換 1m == 20 pixel, 1 pixel = 5cm

cv::RNG rng(12345); // 乱数発生器　12345は初期化の種


// 初期化時に塗りつぶす                                                    
cv::Mat lidar_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 
		  CV_8UC3, cv::Scalar::all(255));
cv::Mat lidar_gray_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
		    CV_8U, cv::Scalar::all(255));
// 1時刻前の画像
cv::Mat lidar_gray_old_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 
			 CV_8U, cv::Scalar::all(255));


class Object {
public:
  bool leg;   // leg:true  or not:false
  int begin; // right side laser
  int end;   // left side laser
  int diff;
  double distance,last_distance; // present, last detected distance
  double angle,last_angle; // rad
  double width;
  double radius; // radius of an enclose circle
  cv::Point2f pos; // position of an image
};

Object human; // obj[100];
cv::Scalar red(0,0,255), blue(255,0,0);



// LIDARデータを画像に変換する
void changeToPicture(int dataCount, double laser_angle_min,
               double laser_angle_max, double laser_angle_increment)
{
  static int64 epochs = 0; 

  lidar_gray_image = cv::Scalar::all(255); // 画面を白くする

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
      lidar_gray_image.data[y*lidar_gray_image.step+x*lidar_gray_image.elemSize()] 
	= value; 
      
      //cout << "intensity[" << j << "]=" << value << endl;
      // slow version
      //lidar_image.at<cv::Vec3b>(y,x) = cv::Vec3b(255,0,0);
    }
  }
  // データを安定化させるために１時刻前の画像と現在の画像のANDを取るようにした
  //cv::Mat tmp_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),CV_8U, cv::Scalar::all(255));
  //cv::bitwise_and(lidar_gray_image, lidar_gray_old_image, tmp_image);
  //lidar_gray_old_image = lidar_gray_image.clone();
  //lidar_gray_image = tmp_image.clone();

  //cv::namedWindow( "Lidar gray image", CV_WINDOW_AUTOSIZE );
  //cv::imshow("Lidar gray image",lidar_gray_image);
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
  //cout << "laser_scan.range_min=" << laser_scan.range_min <<  " max=" << laser_scan.range_max << endl;
  //cout << "data count=" << laser_scan.ranges.size() << endl;
  //exit(1);

  changeToPicture(dataCount, laser_scan.angle_min,
            laser_scan.angle_max,laser_scan.angle_increment);

}

int findLegs(float thresh_min, float thresh_max, cv::Mat input_image, Object *object, cv::Mat display_image,
	      const string& winname, cv::Scalar color)
{
  static int epochs = 0;

  // 画像の初期化
  //display_image = cv::Scalar::all(255);
  //display_image = lidar_gray_image;

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

      // ロボットより後ろは除外                  
      if (center.y -IMAGE_HEIGHT/2 > 0) continue;

      // 輪郭の長さにより除外
      int contour_min = 10, contour_max = 30;
      if (!((contours[cn].size() >= contour_min) && (contours[cn].size() <= contour_max))) continue;

      //  follow_max_distance * mToPixelより前方は検出しない
      //if (follow_max_distance * mToPixel < IMAGE_HEIGHT/2 - center.y) continue; 

      // 外接する長方形を求める
      cv::Rect rect = cv::boundingRect(cv::Mat(contours[cn]));

      // 長方形の底辺による除外
      //double width_min = 10, width_max = 24;
      double width_min = 5, width_max = 21;
      ///double width_min = 5, width_max = 0.6 * mToPixel;
      if (!((rect.width >= width_min) && (rect.width <= width_max))) continue;

      // 縦横比による除外
      double ratio_min = 0.2, ratio_max = 2.1;
      //double ratio_min = 0.2, ratio_max = 1.5;
      double ratio;
      if (rect.width != 0) {
	ratio = (double) rect.height/rect.width;
	if (!((ratio >= ratio_min) && (ratio <= ratio_max))) continue;
      }

      //if ((240 < center.x) && (center.x < 260)) {
      //printf("Epochs=%d,", epochs++);
      //printf("Contour[%d],(, %.0f, %.0f,), ",cn, center.x, center.y);
      //printf("  width=,%d, ratio=,%.2f,\n", rect.width, ratio);
      //}

      // 半径による除外
      //if ((radius < thresh_min) || (radius >  thresh_max)) continue;
      
      // 面積による除外(m00)
      int area_min = 40, area_max = 160; // pixel
      cv::Moments mom = cv::moments(contours[cn]);
      if (!((mom.m00 > area_min) && (mom.m00 < area_max))) continue;

      // m01
      int m01_min = 0, m01_max = 40000;
      //if (!((mom.m01 > m01_min) && (mom.m01 < m01_max))) continue;

      // m10
      int m10_min = 3400, m10_max = 25000;
      if (!((mom.m10 > m10_min) && (mom.m10 < m10_max))) continue;

      // m11 
      int m11_min = 0, m11_max = 5000000;
      //if (!((mom.m11 > m11_min) && (mom.m11 < m11_max))) continue;

      // m02                                                                                                   
      int m02_min = 0, m02_max = 4000000;
      //if (!((mom.m02 > m02_min) && (mom.m02 < m02_max))) continue;

      // m20
      int m20_min = 2000000, m20_max = 5500000;
      //if (!((mom.m20 > m20_min) && (mom.m20 < m20_max))) continue;

      // m30 
      float m30_min = 5E+8, m30_max = 1.4E+9;
      //if (!((mom.m30 > m30_min) && (mom.m30 < m30_max))) continue;

      // 重心による判定
      // 脚（円柱）の断面はU字型なので重心のy座標が円より下になる
      // x座標は中心近辺。中心からずれている脚は追わない 
      //float y_thresh = 0.2;
      Point2f point;
      point.x = mom.m10/mom.m00;
      point.y = mom.m01/mom.m00;

      float diff_x = 1.5;
      if (fabs(center.x-point.x) > diff_x) continue;

      float diff_y =  0.5;
      if(center.y - point.y > diff_y)  continue;
      

      #ifdef MOMENT_EXEL
      if ((center.x > 230) && (center.x < 270)) {
      printf("Epochs=,%d,", epochs++);
      printf("Contour[%d],size=,%d(, %.0f, %.0f,), ",cn, (int) contours[cn].size(),center.x, center.y);
      printf(" rect.width=,%d, ratio=,%.2f,",rect.width, ratio);
      printf("  m00=,%.1f, m01=,%.1f, m10=,%.1f, m11=,%.1f, m02=,%.1f, m20=,%.1f,",mom.m00, mom.m01, mom.m10, mom.m11, mom.m02, mom.m20);
      printf("  m12=,%.1f, m21=,%.1f, m03=,%.1f, m30=,%.1f,", mom.m12, mom.m21, mom.m03, mom.m30);
      printf("  center.x-point.x=,%f, center.y-point.y=,%f\n", center.x-point.x, center.y-point.y);
      }
      #endif

      //cv::circle(display_image,center,radius,color,1);
      cv::rectangle(display_image,rect,color,1);


      //cv::cvCircle(display_image,center,radius,color,1);                          
      //cout << "no="<< cn << " cx=" << center.x << " cy=" << center.y << " my=" << point.y <<
      //  " diffy=" << point.y - center.y << " radius=" << radius  <<  endl;
      //cout << " diffx=" << point.x - center.x << endl;
      object[object_num].radius = radius;
      object[object_num].pos    = center;
      object_num++;
    }

  //printf("FindLeg Time=%f[ms]\n", (cv::getTickCount()-time)*1000/cv::getTickFrequency());

  //cout << "Object num=" << object_num << endl;
  //cv::imshow(winname, display_image);
  return object_num;

}

void calcHumanPos(int object_num,  Object *object, double *dist, double *ang)
{
  // 人の位置推定アルゴリズム                                                                           
  // 物体数０：ロスト                                                                                   
  // 物体数１：重心位置                                                                                 
  // 物体数２以上：１番目と２番目に近い物体の中心。ただし、２つの重心が50cm以上離れていると除外する。   
  double min1_dist = 999999999, min1_angle = 999, min1_num=999, image1_dist;
  double min2_dist = 999999999, min2_angle = 999, min2_num=999, image2_dist;
  Point2f min1_point, min2_point;

  //#ifdef DEBUG                                                                                        
  //cout << "Object num=" << object_num << endl;
  //#endif                                                                                              

  switch (object_num) {
  case 0:
    {
      *dist = 999;
      *ang  = 999;
      return;
    }
  case 1:
    {
      // 1番近い物体を探す                                                                           //min1_dist = (object[0].pos.x - IMAGE_WIDTH/2) * (object[0].pos.x -IMAGE_WIDTH/2)
      //	  + (object[0].pos.y - IMAGE_HEIGHT/2) * (object[0].pos.y -IMAGE_HEIGHT/2);
      //min1_point.x = object[0].pos.x;
      //min1_point.y = object[0].pos.y;

      //*dist = sqrt(min1_dist) / mToPixel;
      //*ang  = atan2(min1_point.x - IMAGE_WIDTH/2, IMAGE_HEIGHT/2 - min1_point.y);
      //cv::circle(lidar_image,min1_point,5,blue,2);  
      *dist = 999;
      *ang  = 999;
      return;
    }
  default: // 2個以上
   {
      // 1番近い物体を探す
      for (int i=0; i < object_num; i++) {
	double diff1 = fabs(object[i].pos.x - IMAGE_WIDTH/2);
	if (diff1 == 0) diff1 = 0.01;
	// 左右６０度以内を脚と考える。それより外は追わない.
	if ((object[i].pos.y - IMAGE_HEIGHT/2 < 0)
	    && (object[i].pos.y - IMAGE_HEIGHT/2) / diff1 >= -0.5) continue; 
	image1_dist = (object[i].pos.x - IMAGE_WIDTH/2) * (object[i].pos.x -IMAGE_WIDTH/2)
	  + (object[i].pos.y - IMAGE_HEIGHT/2) * (object[i].pos.y -IMAGE_HEIGHT/2);
	if (image1_dist < min1_dist) {
	  min1_num  = i;
	  min1_dist = image1_dist;
	  min1_point.x = object[i].pos.x;
	  min1_point.y = object[i].pos.y;
	}
      }
      // ２番目に近い物体を探す 
      for (int i=0; i < object_num; i++) {
	if (i == min1_num) continue;
	double diff2 = fabs(object[i].pos.x - IMAGE_WIDTH/2);
        if (diff2 == 0) diff2 = 0.01;
        // 左右60度以内を脚と考える。それより外は追わない. 
        if ((object[i].pos.y - IMAGE_HEIGHT/2 < 0)
            && (object[i].pos.y - IMAGE_HEIGHT/2) / diff2 >= -0.5) continue;

	image2_dist = (object[i].pos.x - IMAGE_WIDTH/2) * (object[i].pos.x -IMAGE_WIDTH/2)
	  + (object[i].pos.y - IMAGE_HEIGHT/2) * (object[i].pos.y -IMAGE_HEIGHT/2);
	if (image2_dist < min2_dist) {
	  min2_num  = i;
	  min2_dist = image2_dist;
	  min2_point.x = object[i].pos.x;
	  min2_point.y = object[i].pos.y;
	}
      }

      // ２個の距離が0.7m以上離れていたら誤検出     
      double d2 = ((min1_point.x - min2_point.x) *  (min1_point.x - min2_point.x)
		   + (min1_point.y - min2_point.y) *  (min1_point.y - min2_point.y))
	/(mToPixel*mToPixel);
      if (d2 < 0.7 * 0.7) {
         *dist = (sqrt(min1_dist) + sqrt(min2_dist))/(mToPixel * 2);
	 *ang  = ((atan2(min1_point.x - IMAGE_WIDTH/2, IMAGE_HEIGHT/2 - min1_point.y))
               +(atan2(min2_point.x - IMAGE_WIDTH/2, IMAGE_HEIGHT/2 - min2_point.y)))/2;
	 cv::circle(lidar_image,min1_point,5,blue,1);
	 cv::circle(lidar_image,min2_point,5,blue,1);
      }
      else {
	*dist = 999;
	*ang  = 999;
	//*dist = sqrt(min1_dist) / mToPixel;
	//*ang  = atan2(min1_point.x - IMAGE_WIDTH/2, IMAGE_HEIGHT/2 - min1_point.y);
	//cv::circle(lidar_image,min1_point,5,blue,2);
      }
      return;
    }
  }
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

  human.last_distance = 0;
  human.last_angle    = 0;


  cv::Mat img(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3, cv::Scalar(0, 0, 255));
  #ifdef RECORD
  // （1）動画ファイルを書き出すの準備を行う
  cv::VideoWriter writer("videofile.avi", CV_FOURCC_MACRO('X', 'V', 'I', 'D'), 30.0, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), true);

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
    cv::Mat lidar_bin_image, rec_img;
    cv::Mat bin_img1, bin_img2, bin_img3;

    // グレースケールに変換する
    //cvtColor(lidar_image, lidar_gray_image, CV_RGB2GRAY);

    // gray scale -> binary                                                           
    cv::threshold(lidar_gray_image, lidar_bin_image, 0, 255,
                  cv::THRESH_BINARY| cv::THRESH_OTSU);

    cv::namedWindow( "Lidar bin image", CV_WINDOW_AUTOSIZE );
    cv::imshow("Lidar bin image",lidar_bin_image);


    cv::dilate(lidar_bin_image, d1_img, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(lidar_bin_image, d3_img, cv::Mat(), cv::Point(-1,-1), 3);

    cv::erode(lidar_bin_image, e1_img, cv::Mat(), cv::Point(-1,-1), 1);
    cv::erode(lidar_bin_image, e2_img, cv::Mat(), cv::Point(-1,-1), 2);
    cv::erode(lidar_bin_image, e3_img, cv::Mat(), cv::Point(-1,-1), 3);


    cv::threshold(e2_img, e2_bin_img, 0, 255,
                  cv::THRESH_BINARY| cv::THRESH_OTSU);

    cv::dilate(e2_img, e2d1_img, cv::Mat(), cv::Point(-1,-1), 1);

    // To find legs

    //findLegs(lidar_bin_image, lidar_image, "Cirlce 1", red);
    //findLegs(e2_img, e2_color_image, "Circle 2", blue);
    //if ((radius < 5) || (radius > 30)) continue;                                                                          
    float leg_radius = 0.05 * mToPixel;

    //findLegs(leg_radius, 3 * leg_radius, lidar_bin_image, white_image1, "Cirlce 1", red);
    //findLegs(leg_radius, 3 * leg_radius, e1_img, white_image1, "Cirlce 1", red);
    //findLegs(5.0 * leg_radius, 1000, e3_img, white_image2, "Circle 2", blue);

    Object obj[100], obj1[100], obj2[100];
    int obj_num = 0, obj1_num = 0, obj2_num = 0;
    // 脚を見つけるために原画像で連結領域を探す
    // 調整するパラメータ１番目と２番目。脚断面の半径のピクセル数(1: 最小、２：最大)
    //obj1_num = findLegs(0.1*mToPixel, 0.2*mToPixel, lidar_bin_image, obj1, lidar_image, "Cirlce 1", red);

    //  obj1_num = findLegs(0.1*mToPixel, 0.3*mToPixel, e1_img, obj1, lidar_image, "Cirlce 1", red);
    
    cv::Mat lidar_color_image;
    cv::cvtColor(lidar_gray_image, lidar_color_image, CV_GRAY2BGR);
    lidar_image = lidar_color_image;
    obj_num = findLegs(leg_radius, 6 * leg_radius, e1_img, obj, lidar_image, "Cirlce 1", red);
    //obj_num = findLegs(leg_radius, 6 * leg_radius, lidar_bin_image, obj, lidar_image, "Cirlce 1", red);


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

    cv::namedWindow( "Lidar gray image", CV_WINDOW_AUTOSIZE );
    cv::imshow("Lidar gray image",lidar_gray_image);

    cv::namedWindow( "Detected Legs", CV_WINDOW_AUTOSIZE );
    cv::imshow("Detected Legs",lidar_image);

    double tmp_distance, tmp_angle;
    calcHumanPos(obj_num,obj, &tmp_distance, &tmp_angle);
    human.distance = tmp_distance;
    human.angle    = tmp_angle;
    //printf("human.distance=%.1f angle=%.1f \n",human.distance,human.angle*180/M_PI);


   #ifdef DEBUG
    cout << "human dist=" << human.distance << " angle=" << human.angle << endl;
   #endif 

    cv::waitKey(1);

    // 安定して検出できないための工夫
    static bool LOST = false;
    static int lost_count = 0;
    const int lost_count_max = 2; // この回数だけ連続して失敗するとロスト
    if (human.distance == 999) lost_count++;
    else {
      LOST = false;
      lost_count = 0;
      //human.last_distance = human.distance;
      //human.last_angle    = human.angle;
    }
    // 連続lost_count_max回発見できなかったらロスト
    if (lost_count >= lost_count_max) LOST = true;

    if (LOST) { // ロストしたときは１時刻前の値を使う
      human.distance = human.last_distance;
      human.angle    = human.last_angle;
      } 



    //std::cout << "loop:"<< loop++ <<std::endl;
    // 999 はエラーなので速度を0にセット
    if (human.distance == 999) cmd_speed.linear.x = 0;
    // 人が追跡距離内にいる場合
    else if (((human.distance >=  follow_min_distance) &&
              (human.distance <=  follow_max_distance))) 
      {
	double diff = follow_distance - human.distance;

	if (fabs(diff) > 0.1)  	{
	  cmd_speed.linear.x += gain_linear * diff;  // 近づきすぎ
	}
	
	if (human.distance < follow_min_distance)
	cmd_speed.linear.x = 0;
      //double diff = follow_distance - human.distance;
      //cmd_speed.linear.x = gain * diff;
      

      // 速度の上限と下限を設定
      if (cmd_speed.linear.x >   linear_max_speed)
        cmd_speed.linear.x = linear_max_speed;
      if (cmd_speed.linear.x < 0)
        cmd_speed.linear.x = 0;
    }
    // 人が追跡距離外に出た場合は停止する
    else {
      cmd_speed.linear.x = 0;
    }

    // 5度以内のときは回転しない 
    if (human.angle == 999) cmd_speed.angular.z = 0;
    else if (fabs(human.angle) > 0.0 * M_PI/ 180.0) {
	cmd_speed.angular.z -= gain_turn * human.angle;  // 近づきすぎ
    }

    if (cmd_speed.angular.z >  turn_max_speed) 
      cmd_speed.angular.z =  turn_max_speed;
    if (cmd_speed.angular.z < -turn_max_speed) 
      cmd_speed.angular.z = -turn_max_speed;

    // 比例航法
    #ifdef PROPROTIONAL_NAVI
    double angle_change;
    if ((human.angle != 999) && (human.last_angle != 999)) 
      angle_change = human.angle - human.last_angle;
    else
      angle_change = 0;

    cmd_speed.angular.z =- gain_proportion * angle_change;
    cout << "angle_change" << angle_change << "last_angle=" << human.last_angle << " angle=" << human.angle << endl;
    cout << "angular.z=" << cmd_speed.angular.z << endl;
    #endif
    //printf("angle=%.1f speed.z=%.1f distance=%.2f speed.x=%.1f\n",
    //	   human.angle*180/M_PI, cmd_speed.angular.z, human.distance, cmd_speed.linear.x);

    #ifdef MOVE
    kobuki->setTargetVelocity(cmd_speed.linear.x,cmd_speed.angular.z);
    #endif

    if (human.angle != 999)  // 人を見つけている時に現在の値を次の変数に保存
     {
      human.last_distance = human.distance;
      human.last_angle    = human.angle;
    }

    // 動画を取るときは以下をコメントアウトする
    // writer << 取り込みたイメージ名
    #ifdef RECORD
    writer << lidar_image;
    #endif

    loop_rate.sleep(); 
    //usleep(1*1000); 
    ros::spinOnce();
    //xprintf("Main Loop=%f[ms]\n", (double) (cv::getTickCount()-time)*1000/(cv::getTickFrequency()));
  }

  ROS_INFO("Finished\n");
 
  delete kobuki;
  return 0;
}
