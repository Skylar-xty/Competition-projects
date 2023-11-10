#include "ros/ros.h"
#include<opencv2/opencv.hpp>
#include<string.h>
#include "std_msgs/String.h" //普通文本类型的消息
#include <sstream>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <queue>
#include <ctime>
#include <unistd.h>

using namespace cv;
using namespace std;

int hmin,hmax,smin,smax,vmin,vmax;
int areamax,areamin,Distance1,Distance2;
float c1, c2, c3, offset;
int tolerate = 0;
geometry_msgs::Twist lastcmd;
clock_t begin_t, end_t;
bool ctrl = false;
bool Sstart;
bool is_print=false;

string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + ", height=(int)" +
        to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + to_string(framerate) +
        "/1 ! nvvidconv flip-method=" + to_string(flip_method) + " ! video/x-raw, width=(int)" + to_string(display_width) + ", height=(int)" +
        to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

double dist(Point p1,Point p2)
{
    return(sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y)));
}
float Detection(Mat imgDil,Mat img)
{
    float cmd_z;
    // cout<<c1<<"     "<<c2<<endl;
    //imgDil是传入的扩张边缘的图像用来查找轮廓，img是要在其上绘制轮廓的图像
    vector<vector<Point>> contours;//轮廓检测到的轮廓。每个轮廓线存储为一个向量
    vector<Vec4i> hierarchy;//包含关于映像拓扑的信息  typedef Vec<int, 4> Vec4i;具有4个整数值
    //在二值图像中查找轮廓
    findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    //drawContours(img, contours, -1, Scalar(255, 0, 255), 2);//img：要绘制轮廓在什么图片上，contours：要绘制的轮廓，-1定义要绘制的轮廓号（-1表示所有轮廓），Saclar表示轮廓颜色，2表示厚度
    vector<vector<Point>> conPoly(contours.size());//conploy的数量应小于contours
    vector<Rect> boundRect(contours.size());
    vector<Point> points,usfulpoints;
    //遍历检测到的轮廓
    for (int i = 0; i < contours.size(); i++)
    {
        int area = contourArea(contours[i]);
        //cout << area << endl;
        if (area < areamax && area > areamin)
        {
            //cout << area << endl;
            // drawContours(img, contours, i, Scalar(0, 255, 255), 2);
            float peri = arcLength(contours[i], true);//计算封闭轮廓周长
            approxPolyDP(contours[i], conPoly[i],0.03*peri,true);//以指定的精度近似多边形曲线。第二个参数conPloy[i]存储近似的结果，是输出。
            int objCor = (int)conPoly[i].size();
            if (objCor >=4  && objCor <= 8)
            {
                //cout << objCor << endl;
                boundRect[i]=boundingRect(conPoly[i]);//计算边界矩形
                float aspRatio = (float)boundRect[i].width / (float)boundRect[i].height;//宽高比
                if (aspRatio > 0 && aspRatio < 4 )
                {
                        //*** rectangle(img,boundRect[i],Scalar(0, 255, 255),2);
                        //drawContours(img, conPoly, i, Scalar(0, 0, 255), -1);
                        Mat tmp(contours.at(i));
                        Moments moment=moments(tmp, false);
                        if (moment.m00 != 0)//除数不能为0
                        {
                            //*** drawContours(img, conPoly, i, Scalar(0, 0, 255), -1);
                            int x = cvRound(moment.m10 / moment.m00);//计算重心横坐标
                            int y = cvRound(moment.m01 / moment.m00);//计算重心纵坐标
                            points.push_back({x,y});
                        }
                }
            }
        }
    }
    // cout<<"points:"<<points.size()<<endl;
    //增大找到车道起点的鲁棒性
    if(points.size()>0)
    {
        if(dist({100,200},points[0])<30)
            usfulpoints.push_back(points[0]);
        else if (points.size()>1 && dist({100,180},points[1])<30)
            usfulpoints.push_back(points[1]);
        else if (points.size()>2 && dist({100,160},points[2])<30)
            usfulpoints.push_back(points[2]);
        else if (points.size()>3 && dist({100,140},points[3])<30)
            usfulpoints.push_back(points[3]);
        else
        {
            cmd_z = -100;//点太偏
            return cmd_z;
        }
    }
    if(usfulpoints.size()>0)
    {
        //  circle (img, usfulpoints[0], 3, Scalar(255, 255, 255), -1);
        //剔除二维数据离群点
        for(int i=0; i<points.size(); i++)
        {
            //cout<<usfulpoints[usfulpoints.size()-1].x<<"   "<<usfulpoints[usfulpoints.size(s)-1].y<<"   "<<points[i].x<<"   "<<points[i].y<<"   "<<dist(usfulpoints[usfulpoints.size()-1].x,usfulpoints[usfulpoints.size()-1].y,points[i].x,points[i].y)<<endl;
            double Dist = dist(usfulpoints[usfulpoints.size()-1],points[i]);
            if(Dist < Distance1 && Dist > Distance2)
            {
                //cout<<usfulpoints[usfulpoints.size()-1].x<<"   "<<usfulpoints[usfulpoints.size()-1].y<<"   "<<points[i].x<<"   "<<points[i].y<<"   "<<dist(usfulpoints[usfulpoints.size()-1].x,usfulpoints[usfulpoints.size()-1].y,points[i].x,points[i].y)<<endl;
                usfulpoints.push_back(points[i]);
                // circle (img, points[i], 3, Scalar(255, 255, 255), -1);
            }
        }
        cout<<"usfulpoints:"<<usfulpoints.size()<<endl;
    }
    else
    {
        cmd_z = -110;//没有点
        return cmd_z;
    }
    //cmd_z  向左为正，向右为负
    if (usfulpoints.size()>0)
    {
        if(usfulpoints[0].x<135||usfulpoints[0].x>165)
        {
            // circle (img, usfulpoints[0], 4, Scalar(0, 0, 255), -1);
            string core="("+to_string(usfulpoints[0].x)+","+to_string(usfulpoints[0].y)+")";
            cout<<core<<endl;;
            ROS_INFO("case:1");
            cmd_z = float(c1*float(150 - usfulpoints[0].x))/*+lastcmd.angular.z*/;
            // ROS_INFO("%.4f  %.4f",c1,cmd_z);
        }
        else
        {
            ROS_INFO("case:2");
            Vec4f lines;  //存放拟合后的直线
            vector<Point2f> point;  //待检测是否存在直线的所有点
            point.push_back({150,300});
            for(int i=0; i<3;i++)
                point.push_back(usfulpoints[i]);
            fitLine(point, lines, DIST_L1, 0, 0.01, 0.01);
            double k = lines[1] / lines[0];  //直线斜率
            k = float(180*atan(k)/3.14);
            // ROS_INFO("angle=%.4f",k);
            if(k>0)
            {
                k = float(90 - k - offset);  
                cmd_z = float(c2*k);
            }
            else
            {
                k = float(-90 - k + offset);
                cmd_z = float(c3*k);
            }
            // ROS_INFO("%.4f  %.4f  %.4f",c2,k,cmd_z);
        }

    }
    else
    {
        cmd_z = -110;//没有点
    }
    
    // //输出中点坐标
    // for(int i=0;i<usfulpoints.size();i++)
    // {
    //     string core;
    //     core="("+to_string(usfulpoints[i].x)+","+to_string(usfulpoints[i].y)+")";
    //     putText(img, core, Point(usfulpoints[i].x-150, usfulpoints[i].y), FONT_HERSHEY_PLAIN, 1,Scalar(255, 255, 255));
    // }
    //绘制折线
    //polylines(img, usfulpoints, false, Scalar(0, 255, 0), 28, 8, 0);
    return cmd_z;
    // cmds.push(cmd_z);
}
void s_road(ros::NodeHandle nhh)
{
    //实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题 参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher vel_pub = nhh.advertise<geometry_msgs::Twist>("cmd_vel", 3);
    //组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    geometry_msgs::Twist cmd_vel;
    // 编辑msg.data;
    cmd_vel.linear.x = 0.4;
    cmd_vel.angular.z = 0.20;
    vel_pub.publish(cmd_vel);
    // begin_t = clock();
    // 已经确定参数后
    ros::param::get("hmin",hmin);
    ros::param::get("smin",smin);
    ros::param::get("vmin",vmin);
    ros::param::get("hmax",hmax);
    ros::param::get("smax",smax);
    ros::param::get("vmax",vmax);
    ros::param::get("AreaMax",areamax);
    ros::param::get("AreaMin",areamin);
    ros::param::get("DistMax",Distance1);
    ros::param::get("DistMin",Distance2);
    std::cout<<hmin<<" "<<hmax<<" "<<smin<<" "<<smax<<" "<<vmin<<" "<<vmax
    <<" "<<areamax<<" "<<areamin<<" "<<Distance1<<" "<<Distance2<<endl;
    ros::param::get("c1",c1);
    ros::param::get("c2",c2);
    ros::param::get("c3",c3);
    ros::param::get("offset",offset);
    //ros::param::get("delay",delay);
        // int capture_width = 1280;
    // int capture_height = 720;
    // int display_width = 1280;
    // int display_height = 720;
    // int framerate =60;
    // int flip_method = 0;
    int capture_width = 640;
    int capture_height = 480;
    int display_width = 640;
    int display_height = 480;
    int framerate = 30;
    int flip_method = 0;
    ////创建管道
    string pipeline = gstreamer_pipeline(capture_width, capture_height, display_width, display_height, framerate, flip_method);
   // std::cout << "使用gstreamer管道: \n\t" << pipeline << "\n";
    //管道与视频流绑定
    VideoCapture capture(pipeline, CAP_GSTREAMER);
    if(!capture.isOpened())
    {
        std::cout<<"打开摄像头失败."<<std::endl;
        return;
    }
    //获取视频基本信息
    int height = capture.get(CAP_PROP_FRAME_HEIGHT);
    int width = capture.get(CAP_PROP_FRAME_WIDTH);
    int count = capture.get(CAP_PROP_FRAME_COUNT);
    int fps = capture.get(CAP_PROP_FPS);
    cout << height<<"       "<< width<< "       " <<count<< "       " <<fps << endl;
    Mat frame,frame1,matrix, imgWarp, HSV, mask, mask1;
    Mat Dist = (Mat_<double>(5, 1) << -0.3466, 0.1347, 0, 0, 0);
    Mat K = (Mat_<double>(3, 3) << 396.362327865683, 0, 335.498095543887, 0, 528.487957995562, 238.650590420761, 0, 0, 1);
    Point2f src[4] = {{191, 281},
                    {479, 281},
                    {0,   480},
                    {640, 480}};//Point2f表示浮点数
    Point2f dst[4] = {{0, 0},
                    {200, 0},
                    {77, 200},
                    {123, 200}};//Point2f表示浮点数
    matrix = getPerspectiveTransform(src, dst);
    double FPS;
    char fstring[10];  // 用于存放帧率的字符串
    double t = 0;
    //////////////////////////////////////////////
    Scalar lower1(hmin, smin, vmin);
    Scalar upper1(hmax, smax, vmax);
    Scalar lower2(0, smin, vmin);
    Scalar upper2(21, smax, vmax);
    /////////////////////////////////////////////
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));//创建一个核，增加Size（只能是奇数）会扩张/侵蚀更多
    //循环读取视频
    while (Sstart)
    {
        bool change;
        ros::param::get("pathpointchange",change);
        if(change)
        {
            Sstart=false;
            ros::param::set("Sstart",Sstart);
            ctrl=false;
            ros::param::set("Sctrl",ctrl);
            ros::param::set("SroadFlag",1);
            break;
        }
        t = (double)cv::getTickCount();
        if (waitKey(1) == 27){ break; }
        capture.read(frame);
        int ret = capture.read(frame);
        // cout<<ret<<endl;
        if (!ret) {
        break;
        }
        int count=0,count1=0;
        undistort(frame,frame1,K,Dist);
        warpPerspective(frame1, imgWarp, matrix, Point(300,300));
        cvtColor(imgWarp, HSV, COLOR_BGR2HSV);//转换图像到HSV空间，在其中查找颜色更加容易
        medianBlur(HSV, HSV, 7);
        inRange(HSV, lower1, upper1, mask);//定义颜色下限和上限，因为由于照明和不同的阴影，颜色的值将不完全相同，会是一个值的范围
        if(!ctrl)
        {
            inRange(HSV, lower2, upper2, mask1);
            dilate(mask, mask, kernel);//扩张边缘（增加边缘厚度）
            dilate(mask1, mask1, kernel);//扩张边缘（增加边缘厚度）
            vector<vector<Point>> contours,contours1;//轮廓检测到的轮廓。每个轮廓线存储为一个向量
            vector<Vec4i> hierarchy,hierarchy1;//包含关于映像拓扑的信息  typedef Vec<int, 4> Vec4i;具有4个整数值
            //在二值图像中查找轮廓
            findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            findContours(mask1, contours1, hierarchy1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            drawContours(imgWarp, contours, -1, Scalar(255, 0, 255), 2);
            drawContours(imgWarp, contours1, -1, Scalar(255, 0, 0), 2);
            vector<vector<Point>> conPoly(contours.size());//conploy的数量应小于contours
            vector<Rect> boundRect(contours.size());
            vector<Point> points,usfulpoints;
            //遍历检测到的轮廓
            for (int i = 0; i < contours1.size(); i++)
            {
                int area = contourArea(contours1[i]);
                //cout << area << endl;
                if (area > 100)
                {
                    count1+=1;
                    break;
                }
            }
            if(count1>0)
            {
                for (int i = 0; i < contours.size(); i++)
                {
                    int area = contourArea(contours[i]);
                    //cout << area << endl;
                    if (area > 100)
                    {
                        count+=1;
                        break;
                    }
                }
                if(count>3)
                {
                    ctrl=true;
                    ROS_INFO("Now Sroad Start!");
                    ros::param::set("SroadFlag",-1);
                }
                
            }
            else
            {
                for (int i = 0; i < contours.size(); i++)
                {
                    int area = contourArea(contours[i]);
                    //cout << area << endl;
                    if (area > 1100)
                    {
                        ctrl=true;
                        ROS_INFO("Now Sroad Start!");
                        ros::param::set("SroadFlag",-1);
                        break;
                    }
                }
            }
        }
        else
        {
            dilate(mask, mask, kernel);//扩张边缘（增加边缘厚度）
            erode(mask, mask, kernel);//侵蚀边缘（减小边缘厚度）
            dilate(mask, mask, kernel);//扩张边缘（增加边缘厚度）
            erode(mask, mask, kernel);//侵蚀边缘（减小边缘厚度）
            cmd_vel.angular.z = Detection(mask, imgWarp);
            cmd_vel.linear.x=0.4;
            int S_end;
            if(cmd_vel.angular.z==-110)//没有点
            {
                cmd_vel.linear.x=0.4;
                cmd_vel.angular.z=-0.2;
            }
            if(cmd_vel.angular.z==-100)//点太偏
            {
                tolerate++;
                if(tolerate>=10)//容忍度超限，进行异常判断
                {
                    tolerate=0;//清空容忍度
                    ros::param::get("S_end",S_end);
                    cmd_vel.linear.x=-0.4;
                    cmd_vel.angular.z=0;
                }
                else //容忍度未超限，上次命令增加3%
                {
                    cout<<"read last cmd"<<endl;
                    cmd_vel = lastcmd;
                    cmd_vel.angular.z *= 1.03;
                }
            }
            else
                tolerate=0;//清空容忍度
            if(lastcmd.angular.z!=0 && abs(cmd_vel.angular.z)>3*abs(lastcmd.angular.z))//滤波
            {
                cmd_vel.angular.z=lastcmd.angular.z;
            }
            //发布消息
            vel_pub.publish(cmd_vel);
            if(S_end==1)
            {
                cout<<"右转"<<endl;
                sleep(0.5);
                cmd_vel.linear.x=0;
                cmd_vel.angular.z=0;
                cout<<"交接"<<endl;
                ctrl=false;
                ros::param::set("Sctrl",ctrl);
                ros::param::set("SroadFlag",1);
                ros::param::set("S_begin",0);
            }
            //加入调试，打印发送的消息
            ROS_INFO("发送的消息:%lf %lf",cmd_vel.linear.x,cmd_vel.angular.z);
            lastcmd = cmd_vel;
            //t该处代码执行所耗的时间,单位为秒
            t = ((double)getTickCount() - t) / getTickFrequency();
            //cout<<t<<endl;
            FPS = 1.0 / t;
            sprintf(fstring, "%.2f", FPS);// 帧率保留两位小数
            string sstring="FPS:";
            sstring += fstring;
            ROS_INFO("%s",sstring.c_str());
            // putText(imgWarp, sstring, Point(10, 50), FONT_HERSHEY_PLAIN, 3,Scalar(255, 255, 255));
            // imshow("Image", imgWarp);
            // imshow("aaa", mask);
        }
        ros::param::get("Sstart",Sstart);
    }
    capture.release();
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"S_Road");
    ros::NodeHandle nh;
    bool exit;
    exit = false;
    ros::param::set("exit",exit);
    while(!exit)
    {
        ros::param::get("Sstart",Sstart);
        if(Sstart)
        {
            is_print=false;
            s_road(nh);
        }
        else
        {
            sleep(0.3);
            ctrl=false;
            ros::param::set("Sctrl",ctrl);
            if(!is_print)
            {
                ROS_INFO("waiting for start");
                is_print=true;
            }  
        }
        ros::param::get("exit",exit);
    }
    ros::param::set("exit",false);
    return 0;
}




































































// #include "ros/ros.h"
// #include<opencv2/opencv.hpp>
// #include<string.h>
// #include "std_msgs/String.h" //普通文本类型的消息
// #include <sstream>
// #include <cmath>
// #include <boost/algorithm/string.hpp>
// #include <boost/thread.hpp>
// #include <geometry_msgs/Twist.h>
// #include <math.h>
// #include <queue>
// #include <ctime>
// #include <unistd.h>

// using namespace cv;
// using namespace std;

// // int hmin=78;
// // int hmax=147;
// // int smin=112;
// // int smax=255;
// // int vmin=0;
// // int vmax=255;

// // int areamax=3000;
// // int areamin=100;
// // int Distance1=350;
// // int Distance2=50;
// int hmin,hmax,smin,smax,vmin,vmax;
// int areamax,areamin,Distance1,Distance2;
// float c1, c2, c3, offset;
// int tolerate = 0;
// geometry_msgs::Twist lastcmd;
// clock_t begin_t, end_t;
// bool ctrl = false;
// bool Sstart;
// bool is_print=false;

// string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
// {
//     return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + ", height=(int)" +
//         to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + to_string(framerate) +
//         "/1 ! nvvidconv flip-method=" + to_string(flip_method) + " ! video/x-raw, width=(int)" + to_string(display_width) + ", height=(int)" +
//         to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
// }

// double dist(Point p1,Point p2)
// {
//     return(sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y)));
// }
// float Detection(Mat imgDil,Mat img)
// {
//     float cmd_z;
//     // cout<<c1<<"     "<<c2<<endl;
//     //imgDil是传入的扩张边缘的图像用来查找轮廓，img是要在其上绘制轮廓的图像
//     vector<vector<Point>> contours;//轮廓检测到的轮廓。每个轮廓线存储为一个向量
//     vector<Vec4i> hierarchy;//包含关于映像拓扑的信息  typedef Vec<int, 4> Vec4i;具有4个整数值
//     //在二值图像中查找轮廓
//     findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//     //drawContours(img, contours, -1, Scalar(255, 0, 255), 2);//img：要绘制轮廓在什么图片上，contours：要绘制的轮廓，-1定义要绘制的轮廓号（-1表示所有轮廓），Saclar表示轮廓颜色，2表示厚度
//     vector<vector<Point>> conPoly(contours.size());//conploy的数量应小于contours
//     vector<Rect> boundRect(contours.size());
//     vector<Point> points,usfulpoints;
//     //遍历检测到的轮廓
//     for (int i = 0; i < contours.size(); i++)
//     {
//         int area = contourArea(contours[i]);
//         //cout << area << endl;
//         if (area < areamax && area > areamin)
//         {
//             //cout << area << endl;
//             // drawContours(img, contours, i, Scalar(0, 255, 255), 2);
//             float peri = arcLength(contours[i], true);//计算封闭轮廓周长
//             approxPolyDP(contours[i], conPoly[i],0.03*peri,true);//以指定的精度近似多边形曲线。第二个参数conPloy[i]存储近似的结果，是输出。
//             int objCor = (int)conPoly[i].size();
//             if (objCor >=4  && objCor <= 8)
//             {
//                 //cout << objCor << endl;
//                boundRect[i]=boundingRect(conPoly[i]);//计算边界矩形
//                float aspRatio = (float)boundRect[i].width / (float)boundRect[i].height;//宽高比
//                if (aspRatio > 0 && aspRatio < 4 )
//                {
//                     //*** rectangle(img,boundRect[i],Scalar(0, 255, 255),2);
//                     //drawContours(img, conPoly, i, Scalar(0, 0, 255), -1);
//                     Mat tmp(contours.at(i));
//                     Moments moment=moments(tmp, false);
//                     if (moment.m00 != 0)//除数不能为0
//                     {
//                         //*** drawContours(img, conPoly, i, Scalar(0, 0, 255), -1);
//                         int x = cvRound(moment.m10 / moment.m00);//计算重心横坐标
//                         int y = cvRound(moment.m01 / moment.m00);//计算重心纵坐标
//                         points.push_back({x,y});
//                     }
//                }
//             }
//         }
//     }
//     // cout<<"points:"<<points.size()<<endl;
//     //增大找到车道起点的鲁棒性
//     if(points.size()>0)
//     {
//         if(dist({150,300},points[0])<60)
//             usfulpoints.push_back(points[0]);
//         else if (points.size()>1 && dist({150,300},points[1])<80)
//             usfulpoints.push_back(points[1]);
//         else if (points.size()>2 && dist({150,300},points[2])<80)
//         usfulpoints.push_back(points[2]);
//         else if (points.size()>3 && dist({150,300},points[3])<80)
//             usfulpoints.push_back(points[3]);
//         else if (points.size()>4 && dist({150,300},points[4])<80)
//             usfulpoints.push_back(points[4]);
//         else
//         {
//             cmd_z = -100;
//             return cmd_z;
//         }
//     }
//     if(usfulpoints.size()>0)
//     {
//          circle (img, usfulpoints[0], 3, Scalar(255, 255, 255), -1);
//         //剔除二维数据离群点
//         for(int i=0; i<points.size(); i++)
//         {
//             //cout<<usfulpoints[usfulpoints.size()-1].x<<"   "<<usfulpoints[usfulpoints.size(s)-1].y<<"   "<<points[i].x<<"   "<<points[i].y<<"   "<<dist(usfulpoints[usfulpoints.size()-1].x,usfulpoints[usfulpoints.size()-1].y,points[i].x,points[i].y)<<endl;
//             double Dist = dist(usfulpoints[usfulpoints.size()-1],points[i]);
//             if(Dist < Distance1 && Dist > Distance2)
//             {
//                 //cout<<usfulpoints[usfulpoints.size()-1].x<<"   "<<usfulpoints[usfulpoints.size()-1].y<<"   "<<points[i].x<<"   "<<points[i].y<<"   "<<dist(usfulpoints[usfulpoints.size()-1].x,usfulpoints[usfulpoints.size()-1].y,points[i].x,points[i].y)<<endl;
//                 usfulpoints.push_back(points[i]);
//                 circle (img, points[i], 3, Scalar(255, 255, 255), -1);
//             }
//         }
//         cout<<"usfulpoints:"<<usfulpoints.size()<<endl;
//     }
//     else
//     {
//         cmd_z = -100;
//         return cmd_z;
//     }
//     //cmd_z  向左为正，向右为负
//     if (usfulpoints.size()>0)
//     {
//         if(usfulpoints[0].x<135||usfulpoints[0].x>165)
//         {
//             circle (img, usfulpoints[0], 4, Scalar(0, 0, 255), -1);
//             string core="("+to_string(usfulpoints[0].x)+","+to_string(usfulpoints[0].y)+")";
//             cout<<core<<endl;;
//             ROS_INFO("case:1");
//             cmd_z = float(c1*float(150 - usfulpoints[0].x))/*+lastcmd.angular.z*/;
//             // ROS_INFO("%.4f  %.4f",c1,cmd_z);
//         }
//         else
//         {
//             ROS_INFO("case:2");
//             Vec4f lines;  //存放拟合后的直线
//             vector<Point2f> point;  //待检测是否存在直线的所有点
//             point.push_back({150,300});
//             for(int i=0; i<3;i++)
//                 point.push_back(usfulpoints[i]);
//             fitLine(point, lines, DIST_L1, 0, 0.01, 0.01);
//             double k = lines[1] / lines[0];  //直线斜率
//             k = float(180*atan(k)/3.14);
//             // ROS_INFO("angle=%.4f",k);
//             if(k>0)
//             {
//                 k = float(90 - k - offset);  
//                 cmd_z = float(c2*k);
//             }
//             else
//             {
//                 k = float(-90 - k + offset);
//                 cmd_z = float(c3*k);
//             }
//             // ROS_INFO("%.4f  %.4f  %.4f",c2,k,cmd_z);
//         }

//     }
//     else
//     {
//         cmd_z = -100;
//     }
    
//     // //输出中点坐标
//     // for(int i=0;i<usfulpoints.size();i++)
//     // {
//     //     string core;
//     //     core="("+to_string(usfulpoints[i].x)+","+to_string(usfulpoints[i].y)+")";
//     //     putText(img, core, Point(usfulpoints[i].x-150, usfulpoints[i].y), FONT_HERSHEY_PLAIN, 1,Scalar(255, 255, 255));
//     // }
//     //绘制折线
//     //polylines(img, usfulpoints, false, Scalar(0, 255, 0), 28, 8, 0);
//     return cmd_z;
//     // cmds.push(cmd_z);
// }
// void s_road(ros::NodeHandle nhh)
// {
//     //实例化 发布者 对象
//     //泛型: 发布的消息类型
//     //参数1: 要发布到的话题 参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
//     ros::Publisher vel_pub = nhh.advertise<geometry_msgs::Twist>("cmd_vel", 3);
//     //组织被发布的数据，并编写逻辑发布数据
//     //数据(动态组织)
//     geometry_msgs::Twist cmd_vel;
//     // 编辑msg.data;
//     cmd_vel.linear.x = 0.4;
//     cmd_vel.angular.z = 0.20;
//     vel_pub.publish(cmd_vel);
//     // begin_t = clock();
//     // 已经确定参数后
//     ros::param::get("hmin",hmin);
//     ros::param::get("smin",smin);
//     ros::param::get("vmin",vmin);
//     ros::param::get("hmax",hmax);
//     ros::param::get("smax",smax);
//     ros::param::get("vmax",vmax);
//     ros::param::get("AreaMax",areamax);
//     ros::param::get("AreaMin",areamin);
//     ros::param::get("DistMax",Distance1);
//     ros::param::get("DistMin",Distance2);
//     std::cout<<hmin<<" "<<hmax<<" "<<smin<<" "<<smax<<" "<<vmin<<" "<<vmax
//     <<" "<<areamax<<" "<<areamin<<" "<<Distance1<<" "<<Distance2<<endl;
//     ros::param::get("c1",c1);
//     ros::param::get("c2",c2);
//     ros::param::get("c3",c3);
//     ros::param::get("offset",offset);
//     //ros::param::get("delay",delay);
//         // int capture_width = 1280;
//     // int capture_height = 720;
//     // int display_width = 1280;
//     // int display_height = 720;
//     // int framerate =60;
//     // int flip_method = 0;
//     int capture_width = 640;
//     int capture_height = 480;
//     int display_width = 640;
//     int display_height = 480;
//     int framerate = 30;
//     int flip_method = 0;
//     ////创建管道
//     string pipeline = gstreamer_pipeline(capture_width, capture_height, display_width, display_height, framerate, flip_method);
//    // std::cout << "使用gstreamer管道: \n\t" << pipeline << "\n";
//     //管道与视频流绑定
//     VideoCapture capture(pipeline, CAP_GSTREAMER);
//     if(!capture.isOpened())
//     {
//         std::cout<<"打开摄像头失败."<<std::endl;
//         return;
//     }
//     //获取视频基本信息
//     int height = capture.get(CAP_PROP_FRAME_HEIGHT);
//     int width = capture.get(CAP_PROP_FRAME_WIDTH);
//     int count = capture.get(CAP_PROP_FRAME_COUNT);
//     int fps = capture.get(CAP_PROP_FPS);
//     cout << height<<"       "<< width<< "       " <<count<< "       " <<fps << endl;
//     Mat frame,frame1,matrix, imgWarp, HSV, mask;
//     Mat Dist = (Mat_<double>(5, 1) << -0.3466, 0.1347, 0, 0, 0);
//     Mat K = (Mat_<double>(3, 3) << 396.362327865683, 0, 335.498095543887, 0, 528.487957995562, 238.650590420761, 0, 0, 1);
//     Point2f src[4] = {{149, 294},
// 					  {491, 294},
// 				      {0,   480},
// 					  {640, 480}};//Point2f表示浮点数
//     Point2f dst[4] = {{0, 0},
// 					  {300, 0},
// 					  {112, 300},
// 					  {188, 300}};//Point2f表示浮点数
//     matrix = getPerspectiveTransform(src, dst);
//     double FPS;
//     char fstring[10];  // 用于存放帧率的字符串
//     double t = 0;
//     //////////////////////////////////////////////
//     Scalar lower0(hmin, smin, vmin);
//     Scalar upper0(hmax, smax, vmax);
//     /////////////////////////////////////////////
//     Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7));//创建一个核，增加Size（只能是奇数）会扩张/侵蚀更多
//     //循环读取视频
//     while (Sstart)
//     {
//         bool change;
//         ros::param::get("pathpointchange",change);
//         if(change)
//         {
//             Sstart=false;
//             ros::param::set("Sstart",Sstart);
//             ctrl=false;
//             ros::param::set("Sctrl",ctrl);
//             ros::param::set("SroadFlag",1);
//             ros::param::set("S_begin",0);
//             break;
//         }
//         t = (double)cv::getTickCount();
//         if (waitKey(1) == 27){ break; }
//         capture.read(frame);
//         int ret = capture.read(frame);
//         // cout<<ret<<endl;
//         if (!ret) {
//             break;
//         }
//         undistort(frame,frame1,K,Dist);
//         warpPerspective(frame1, imgWarp, matrix, Point(300,300));
//         cvtColor(imgWarp, HSV, COLOR_BGR2HSV);//转换图像到HSV空间，在其中查找颜色更加容易
//         medianBlur(HSV, HSV, 7);
//         if(!ctrl)
//         {
//             int S_begin;
//             ros::param::get("S_begin",S_begin);
//             if(S_begin==1)
//             {
//                 ctrl=true;
//                 ros::param::set("Sctrl",ctrl);
//                 ROS_INFO("Now Sroad Start!");
//                 ros::param::set("SroadFlag",-1);
//                 ros::param::set("S_begin",0);
//             }
//             else
//             {
//                 sleep(0.1);
//             }
//         }
//         else
//         {
//             inRange(HSV, lower0, upper0, mask);//定义颜色下限和上限，因为由于照明和不同的阴影，颜色的值将不完全相同，会是一个值的范围
//             dilate(mask, mask, kernel);//扩张边缘（增加边缘厚度）
//             erode(mask, mask, kernel);//侵蚀边缘（减小边缘厚度）
//             dilate(mask, mask, kernel);//扩张边缘（增加边缘厚度）
//             erode(mask, mask, kernel);//侵蚀边缘（减小边缘厚度）
//             cmd_vel.angular.z = Detection(mask, imgWarp);
//             cmd_vel.linear.x=0.4;
//             int S_end;
//             if(cmd_vel.angular.z==-100)
//             {
//                 tolerate++;
//                 if(tolerate>=10)//容忍度超限，进行异常判断
//                 {
//                     tolerate=0;//清空容忍度
//                     ros::param::get("S_end",S_end);
//                     if(S_end==1/*在终点*/)
//                     {
//                         cmd_vel.linear.x=0.4;
//                         cmd_vel.angular.z=-0.14;
//                     }
//                     else
//                     {
//                         cmd_vel.linear.x=-0.4;
//                         cmd_vel.angular.z=0;
//                     }
//                 }
//                 else //容忍度未超限，延续上次命令
//                 {
//                     cout<<"read last cmd"<<endl;
//                     cmd_vel = lastcmd;
//                     cmd_vel.angular.z *= 1.03;
//                 }
                
//             }
//             else
//                 tolerate=0;//清空容忍度
//             if(lastcmd.angular.z!=0 && abs(cmd_vel.angular.z)>3*abs(lastcmd.angular.z))
//             {
//                 cmd_vel.angular.z=lastcmd.angular.z;
//             }
//             //发布消息
//             vel_pub.publish(cmd_vel);
//             if(S_end==1)
//             {
//                 cout<<"右转"<<endl;
//                 sleep(0.5);
//                 cmd_vel.linear.x=0;
//                 cmd_vel.angular.z=0;
//                 cout<<"交接"<<endl;
//                 ctrl=false;
//                 ros::param::set("Sctrl",ctrl);
//                 ros::param::set("SroadFlag",1);
//                 ros::param::set("S_begin",0);
//             }
//             //加入调试，打印发送的消息
//             ROS_INFO("发送的消息:%lf %lf",cmd_vel.linear.x,cmd_vel.angular.z);
//             lastcmd = cmd_vel;
//             //t该处代码执行所耗的时间,单位为秒
//             t = ((double)getTickCount() - t) / getTickFrequency();
//             //cout<<t<<endl;
//             FPS = 1.0 / t;
//             sprintf(fstring, "%.2f", FPS);// 帧率保留两位小数
//             string sstring="FPS:";
//             sstring += fstring;
//             ROS_INFO("%s",sstring.c_str());
//             // putText(imgWarp, sstring, Point(10, 50), FONT_HERSHEY_PLAIN, 3,Scalar(255, 255, 255));
//             // imshow("Image", imgWarp);
//             // imshow("aaa", mask);
//         }
//         ros::param::get("Sstart",Sstart);
//     }
//     capture.release();
// }

// int main(int argc, char *argv[])
// {
//     setlocale(LC_ALL,"");
//     ros::init(argc,argv,"S_Road");
//     ros::NodeHandle nh;
//     bool exit;
//     exit = false;
//     ros::param::set("exit",exit);
//     while(!exit)
//     {
//         ros::param::get("Sstart",Sstart);
//         if(Sstart)
//         {
//             is_print=false;
//             s_road(nh);
//         }
//         else
//         {
//             sleep(0.3);
//             ctrl=false;
//             ros::param::set("Sctrl",ctrl);
//             if(!is_print)
//             {
//                 ROS_INFO("waiting for start");
//                 is_print=true;
//             }  
//         }
//         ros::param::get("exit",exit);
//     }
//     ros::param::set("exit",false);
//     return 0;
// }