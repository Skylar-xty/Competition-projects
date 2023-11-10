#include "ros/ros.h"
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int hmin = 92, smin = 40, vmin = 0;
int hmax = 150, smax = 141, vmax = 255;
int areamax=347, areamin=37, Distance1=81, Distance2=6;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "set_param");
    ros::NodeHandle nh;
    namedWindow("Trackbars", (640, 400));//(640,200)是尺寸
    //运行时，把3个min的都移到最小值，把3个max的都移到最大值，然后移动使其保持为白色
    createTrackbar("Hue Min", "Trackbars", &hmin, 179);//对于hue色相饱和度最大180,对于另外两个色相饱和度最大255
    createTrackbar("Hue Max", "Trackbars", &hmax, 179);
    createTrackbar("Sat Min", "Trackbars", &smin, 255);
    createTrackbar("Sat Max", "Trackbars", &smax, 255);
    createTrackbar("Val Min", "Trackbars", &vmin, 255);
    createTrackbar("Val Max", "Trackbars", &vmax, 255);
    createTrackbar("AreaMax", "Trackbars", &areamax, 1500);
    createTrackbar("AreaMin", "Trackbars", &areamin, 500);
    createTrackbar("DistMax", "Trackbars", &Distance1, 500);
    createTrackbar("DistMin", "Trackbars", &Distance2, 500);
    while (true)
    {
        ros::param::set("hmin",hmin);
        ros::param::set("smin",smin);
        ros::param::set("vmin",vmin);
        ros::param::set("hmax",hmax);
        ros::param::set("smax",smax);
        ros::param::set("vmax",vmax);
        ros::param::set("AreaMax",areamax);
        ros::param::set("AreaMin",areamin);
        ros::param::set("DistMax",Distance1);
        ros::param::set("DistMin",Distance2);
    
        char c = waitKey(20);//增加延时
        if(c == 27)
        {
            system("rosparam dump /home/t718/qingzhou_ws/src/s_road/001.yaml");
            break;
        }
    } 
    return 0;
}