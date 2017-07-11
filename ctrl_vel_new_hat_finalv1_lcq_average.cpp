#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>

//#include </usr/local/include/opencv2/opencv.hpp>
#include<opencv.hpp>
#include <ctype.h>
#include <iostream>
#include <fstream>
#include "mavros_msgs/State.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"

using namespace std;
using namespace cv;

double theta;
double count=0.0;
double wn;
float p0=0;
float p1=0;
float p2=0;
float p3=0;



sensor_msgs::Imu current_state;

void cacSIFTFeatureAndCompare1(cv::Mat srcImage1, cv::Mat srcImage2, float paraHessian, vector<Point2f> &keyPoints_min_xiaotu, vector<Point2f> &keyPoints_min_datu);

void state_cb(const sensor_msgs::Imu::ConstPtr& msg){
    current_state = *msg;
    p0=current_state.orientation.w;
    p1=current_state.orientation.x;
    p2=current_state.orientation.y;
    p3=current_state.orientation.z;
}



Mat Q = (Mat_<double>(4,4)<<1, 0, 0, -361.4730985462666,
  0, 1, 0, -272.1588581576943,
  0, 0, 0, 548.2305693935355,
  0, 0, 0.006341732724551271, -0);              					//校正旋转矩阵R，投影矩阵P 重投影矩阵Q



Mat cameraMatrixL = (Mat_<double>(3, 3) << 448.5378049000174, 0, 324.8809738711533,
  0, 452.4866266972798, 226.7763297648286,
  0, 0, 1);
Mat distCoeffL = (Mat_<double>(5, 1) << 0.04758753671284408, -0.0596091045075235, -0.001952052204146848, 0.008862981611723015, -0.199401111654961);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 458.3425519191315, 0, 339.1660066594821,
  0, 461.1052941269068, 226.9853524498002,
  0, 0, 1);
Mat distCoeffR = (Mat_<double>(5, 1) << -0.05682803887053835, 0.8890880815233615, -0.005391408915285348, -0.001557825533545376, -2.829656918388968);

/*下面两个矩阵表达的是两个相机之间的位置关系*/
Mat T = (Mat_<double>(3, 1) << -157.6633688382337,
  -2.328569883804867,
  1.26167812736466);//T平移向量

Mat R = (Mat_<double>(3, 3) << 0.9996368587249664, -0.007295074935325424, -0.02594094370237057,
  0.007168792956369447, 0.9999620139470223, -0.004957728367294566,
  0.0259761253082572, 0.004769962756998862, 0.999651183348105);//R 旋转矩阵


VideoCapture cap0(0),cap1(1);

Mat frame0,frame1,hsvFrame0,hsvFrame1,imgThresholded0,imgThresholded1;
bool stop=true;
int height,width;
Scalar redMin=Scalar(150,140,0);
Scalar redMax=Scalar(179,255,255);
Mat element=getStructuringElement(MORPH_RECT,Size(5,5));

float normal_frame_z;//正常帧的记录
int IndexOfNormalFrame=0;

/*测距部分全局变量结束*/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "ctrl_vel");
   ros::NodeHandle nh;

   ros::Subscriber state_sub = nh.subscribe("mavros/imu", 10, state_cb);

   ros::Publisher position_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",10);
   ros::Rate loop_rate(10);
   ros::spinOnce();

   geometry_msgs::TwistStamped vel;
   int count = 1;

        //PositionReciever qp;:
        //Body some_object;
        //qp.connect_to_server();

	cap0.set(CV_CAP_PROP_FRAME_WIDTH,320);
	cap0.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	cap1.set(CV_CAP_PROP_FRAME_WIDTH,320);
	cap1.set(CV_CAP_PROP_FRAME_HEIGHT,240);

	// added by crx
	// load the cascade
    const char *pstrCascadeFileName = "F:\\tools3\\cascade2xml\\output stage15.xml";
    CvHaarClassifierCascade *pHaarCascade = NULL;
    pHaarCascade = (CvHaarClassifierCascade*)cvLoad(pstrCascadeFileName);
    if(!pHaarCascade) {printf("分类器加载失败\n");return -1;}


   while(ros::ok())
   {
       //some_object = qp.getStatus();
        // some_object.print();
        //printf("%f\n",some_object.position_x);

	/*测距部分代码开始*/

		if(!cap0.isOpened())
		{
			cout<<"print cap0 not open"<<endl;
			break;
		}

		if(!cap1.isOpened())
		{
			cout<<"print cap0 not open"<<endl;
			break;
		}

			cap0>>frame0;
			cap1>>frame1;
			if(frame0.empty()||frame1.empty())
				{
					stop=true;
					break; //如果当前帧为空，退出循环
				}
			imshow("test0",frame0);
			imshow("test1",frame1);

			vector<Point2f> keyPoints_min_0, keyPoints_min_1;
			cacSIFTFeatureAndCompare1(srcImage1, srcImage2,2500,keyPoints_min_0,  keyPoints_min_1);

			Mat frame0_gray, frame1_gray;

			cvtColor(frame0, frame0_gray, CV_RGB2GRAY);
			cvtColor(frame1, frame1_gray, CV_RGB2GRAY);

			IplImage* Iplimage_0_GRAY;
			*Iplimage_0_GRAY = IplImage(frame1_gray);
			IplImage* Iplimage_1_GRAY;
			*Iplimage_1_GRAY = IplImage(frame1_gray);

			// 设置缓存区
			CvMemStorage *pcvMStorage0 = cvCreateMemStorage(0);
			cvClearMemStorage(pcvMStorage0);
			// 【3】识别
			CvSeq *pcvSeqFaces0 = cvHaarDetectObjects(Iplimage_0_GRAY, pHaarCascade, pcvMStorage0);
			printf("左边图人脸个数: %d\n", pcvSeqFaces0->total);

			// 设置缓存区
			CvMemStorage *pcvMStorage1 = cvCreateMemStorage(0);
			cvClearMemStorage(pcvMStorage1);
			// 【3】识别
			CvSeq *pcvSeqFaces1 = cvHaarDetectObjects(Iplimage_1_GRAY, pHaarCascade, pcvMStorage1);
			printf("右边图人脸个数: %d\n", pcvSeqFaces1->total);

			if (pcvSeqFaces1->total == 0) continue;
			if (pcvSeqFaces0->total == 0) continue;



			vector<vector<int>> rect_area_0;
			int index0 = 0;
			vector<vector<int>> rect_area_1;
			int index1 = 0;

			cvtColor(frame0, hsvFrame0, CV_BGR2HSV);//转换为HSV颜色空间
			cvtColor(frame1, hsvFrame1, CV_BGR2HSV);//转换为HSV颜色空间


			for (int m = 0; m <pcvSeqFaces0->total; m++)
			{
				CvRect* r = (CvRect*)cvGetSeqElem(pcvSeqFaces0, m);
				CvPoint center;
				int radius;
				center.x = cvRound((r->x + r->width * 0.5));
				center.y = cvRound((r->y + r->height * 0.5));
				radius = cvRound((r->width + r->height) * 0.25);

				int xmin, ymin, xmax, ymax;
				if (center.x - radius <= 0) xmin = 1;
				else xmin = center.x - radius;
				if (center.y - radius <= 0) ymin = 1;
				else ymin = center.y - radius;
				if (center.x + radius >= 1920) xmax = 1920;
				else xmax = center.x + radius;
				if (center.y + radius >= 1080) ymax = 1080;
				else ymax = center.y + radius;
				
				Mat tiny0(hsvFrame0, *r);
				imshow("小框框", tiny0);

				inRange(tiny0, redMin, redMax, imgThresholded0); //过滤图像
				morphologyEx(imgThresholded0, imgThresholded0, MORPH_OPEN, element);//形态学操作，开操作

				vector<vector<Point> > contours0;
				vector<Vec4i> hierarcy0;
				findContours(imgThresholded0, contours0, hierarcy0, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//寻找轮廓

				// added by crx

				double max_area = 0.0;
				int max_index = 0;
				int T = 20;

				for (int i = 0; i < contours0.size(); i++)   //size()or size
				{
					if (contourArea(contours0[0], true) > max_area )
					{
						max_area = contourArea(contours0[0], true);
						max_index = i;
					}
				}

				if (max_area > T)
				{
					rect_area_0[index0][0] = r->x;
					rect_area_0[index0][1] = r->y;
					rect_area_0[index0][2] = r->width;
					rect_area_0[index0][3] = r->height;
					rect_area_0[index0][4] = max_area;
					index0 ++;
				}

				cvReleaseMemStorage(&pcvMStorage0); // 释放缓存
			}


			for (int m = 0; m <pcvSeqFaces1->total; m++)
			{
				CvRect* r = (CvRect*)cvGetSeqElem(pcvSeqFaces1, m);
				CvPoint center;
				int radius;
				center.x = cvRound((r->x + r->width * 0.5));
				center.y = cvRound((r->y + r->height * 0.5));
				radius = cvRound((r->width + r->height) * 0.25);


				Mat tiny1(hsvFrame1, *r);
				imshow("小框框", tiny1);

				inRange(tiny1, redMin, redMax, imgThresholded1); //过滤图像
				morphologyEx(imgThresholded1, imgThresholded1, MORPH_OPEN, element);//形态学操作，开操作

				vector<vector<Point> > contours1;
				vector<Vec4i> hierarcy1;
				findContours(imgThresholded1, contours1, hierarcy1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//寻找轮廓

				// added by crx

				double max_area = 0.0;
				int max_index = 0;
				int T = 20;

				for (int i = 0; i < contours1.size(); i++)   //size()or size
				{
					if (contourArea(contours1[0], true) > max_area)
					{
						max_area = contourArea(contours1[0], true);
						max_index = i;
					}
				}

				if (max_area > T)
				{
					rect_area_1[index0][0] = r->x;
					rect_area_1[index0][1] = r->y;
					rect_area_1[index0][2] = r->width;
					rect_area_1[index0][3] = r->height;
					rect_area_1[index0][4] = max_area;
					index0++;
				}

				cvReleaseMemStorage(&pcvMStorage1); // 释放缓存
			}









			double max_area_all_0 = 0.0, max_area_all_1 = 0.0;
			int  index_max_0, index_max_1;
		
			for (int i = 0; i < rect_area_0.size(); i++)
			{
				if (rect_area_0[i][5] > max_area_all_0)
				{
					max_area_all_0 = rect_area_0[i][5];
					index_max_0 = i;
				}
			}
			
			int center_0_x = rect_area_0[index_max_0][0] + int((rect_area_0[index_max_0][3]) / 2);
			int center_0_y = rect_area_0[index_max_0][1] + int((rect_area_0[index_max_0][4]) / 2);



			for (int i = 0; i < rect_area_1.size(); i++)
			{
				if (rect_area_1[i][5] > max_area_all_1)
				{
					max_area_all_1 = rect_area_1[i][5];
					index_max_1 = i;
				}
			}

			int center_1_x = rect_area_1[index_max_1][0] + int((rect_area_1[index_max_1][3]) / 2);
			int center_1_y = rect_area_1[index_max_1][1] + int((rect_area_1[index_max_1][4]) / 2);



			double x_sum_0 =  0.0,y_sum_0 =  0.0;
			double x_sum_1 =  0.0,y_sum_1 =  0.0;
			int n = 0;
			for(int i = 0;i < keyPoints_min_0.size();i++)
			{
				if( (keyPoints_min_0[i].x <= (rect_area_0.x +rect_area_0.width)) && (keyPoints_min_0[i].x >= (rect_area_0.x)) 
					&& keyPoints_min_0[i].y <= (rect_area_0.y +rect_area_0.height)) && (keyPoints_min_0[i].y >= (rect_area_0.y)))
				{
					x_sum_0 += keyPoints_min_0.x;
					y_sum_0 += keyPoints_min_0.y;
					x_sum_1 += keyPoints_min_1.x;
					y_sum_1 += keyPoints_min_1.y;
					n++;
				}
			}
			x_sum_0 / = n;
			y_sum_0 / = n;
			x_sum_1 / = n;
			y_sum_1 / = n;

			double d = x_sum_0 - x_sum_1;

		   //测距

		//用来计算深度信息的Z=f*Tx/d;
		double f,Tx;								//焦距f,和基线长Tx
		double d = center_0_x - center_1_x;									//视差d，为了防止计算出来的不是int，设为double类型
		f=Q.at<double>(2,3);
		Tx=-1/Q.at<double>(3,2);



		//以下定义的几个是为了计算X,Y
		double Cx,Cy,fx,fy;

		Cx=cameraMatrixL.at<double>(0,2);
		Cy=cameraMatrixL.at<double>(1,2);

		fx=cameraMatrixL.at<double>(0,0);
		fy=cameraMatrixL.at<double>(1,1);

		double X_coor,Y_coor,Z_coor,X1,Y1,Z1,X,Y,Z;
		double q0=0.781,q1=0.0129,q2=0.0159,q3=0.624,y0=2.0;



		Z_coor=-f*Tx/d;

		X_coor=(x_sum_0-Cx)*Z_coor/fx;
		Y_coor=(y_sum_0-Cy)*Z_coor/fy;

		//Mat xyz=(Mat_<double>(3,1)<<(X_coor,Y_coor,Z_coor));   //三维坐标

		//cout<<"圆心的坐标是("<<xyz<<")"<<endl;
		X1=Z_coor;
		Y1=-X_coor+8;

		float normal_frame_z=Z1;
		Z1=-Y_coor;
		if(Z1/normal_frame_z>3||Z1>200)
			{
				if(IndexOfNormalFrame<3)
					{
						Z1=normal_frame_z;
					}

				IndexOfNormalFrame++;

				continue;
			}
Z=-X1+800;
X=Z1;
Y=Y1;
double roll1[3] [3]={{1/1.41, 1/1.41, 0}, {-1/1.41, 1/1.41, 0}, {0, 0, 1}};
double rawco[3] [1]={{X},{Y},{Z}};
double co[3][1];
for(int m=0;m<3;m++)
	{
        for(int s=0;s<1;s++)
		{
            co[m][s]=0;//变量使用前记得初始化,否则结果具有不确定性
            for(int n=0;n<3;n++)
			{
                co[m][s]+=roll1[m][n]*rawco[n][s];
            }
        }
	}
		cout<<"x="<<X<<"  y="<<Y<<"  z="<<Z<<endl;
		double r32=2*(q0*q1+q2*q3);
		double r33=pow(q0,y0)-pow(q1,y0)-pow(q2,y0)+pow(q3,y0);
		double r31=2*(q1*q3-q0*q2);
		double r21=2*(q1*q2+q0*q3);
		double r11=pow(q0,y0)+pow(q1,y0)-pow(q2,y0)-pow(q3,y0);
		double roll=atan(r32/r33);
		double pitch=asin(-r31);
		double yaw=atan(r21/r11);
		double matrix_a[3] [3]={ { cos(pitch)*cos(yaw), -cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw),sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw) }, { cos(pitch)*sin(yaw), cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw) },{-sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch)} };//a矩阵2X3
		double matrix_result[3][1];//结果矩阵2X4
    	double vmat[3][1];


    for(int m=0;m<3;m++)
	{
        for(int s=0;s<1;s++)
		{
            matrix_result[m][s]=0;//变量使用前记得初始化,否则结果具有不确定性
            for(int n=0;n<3;n++)
			{
                matrix_result[m][s]+=matrix_a[m][n]*co[n][s];
            }
        }
	}



	double sizevr=0;
	for (int i=0;i<3;++i)
	{
		sizevr +=pow(matrix_result[i][0],2.0);
	}
		double sizev=sqrt(sizevr);

	for (int i = 0; i < 3; ++i)
	{
		vmat[i][0]=matrix_result[i][0]/500;
	}

	//measurement end

       vel.header.stamp = ros::Time::now();
       vel.header.seq=count;
       vel.header.frame_id = 1;
       vel.twist.linear.x=vmat[0][0];
       vel.twist.linear.y=vmat[1][0];
       vel.twist.linear.z=vmat[2][0];
       //vel.twist.angular.x=0;
       //vel.twist.angular.y=0;
       //vel.twist.angular.z=0;

       position_pub.publish(vel);
       ros::spinOnce();
       count++;
       loop_rate.sleep();
	waitKey(30);
   }


   return 0;
}



void cacSIFTFeatureAndCompare1(cv::Mat srcImage1, cv::Mat srcImage2, float paraHessian, vector<Point2f> &keyPoints_min_xiaotu, vector<Point2f> &keyPoints_min_datu)
{
	CV_Assert(srcImage1.data != NULL && srcImage2.data != NULL);
	Mat grayMat1, grayMat2;
	cvtColor(srcImage1, grayMat1, CV_RGB2GRAY);
	cvtColor(srcImage2, grayMat2, CV_RGB2GRAY);
	SurfFeatureDetector surfDetector(paraHessian);
	SurfDescriptorExtractor surfExtractor;
	vector<KeyPoint> keyPoints1, keyPoints2;
	Mat descriptorMat1, descriptorMat2;
	surfDetector.detect(grayMat1, keyPoints1);
	surfDetector.detect(grayMat2, keyPoints2);
	surfExtractor.compute(grayMat1, keyPoints1, descriptorMat1);
	surfExtractor.compute(grayMat2, keyPoints2, descriptorMat2);
	if (keyPoints1.size() > 0 && keyPoints2.size() > 0)
	{
		cv::FlannBasedMatcher matcher;
		vector< cv::DMatch > matches;
		std::vector<cv::DMatch> viewMatches;
		matcher.match(descriptorMat1, descriptorMat2, matches);
		//最优匹配判断  
		double minDist = 100;
		for (int i = 0; i < matches.size(); i++)
		{
			if (matches[i].distance < minDist)
				minDist = matches[i].distance;
		}
		for (int i = 0; i < matches.size(); i++)
		{
			if (matches[i].distance <= 2 * minDist)
			{
				viewMatches.push_back(matches[i]);
				keyPoints_min_xiaotu.push_back(keyPoints1[matches[i].queryIdx].pt);
				keyPoints_min_datu.push_back(keyPoints2[matches[i].trainIdx].pt);
			}
		}

		Mat matchMat;
		drawMatches(srcImage1, keyPoints1, srcImage2, keyPoints2, viewMatches, matchMat);
		imshow("matchMat", matchMat);
		waitKey();
	}

	
}
