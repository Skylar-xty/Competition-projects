#include "ros/ros.h"
#include<opencv2/opencv.hpp>
#include<string.h>

using namespace cv;
using namespace std;

// int hmin=78;
// int hmax=147;
// int smin=112;
// int smax=255;
// int vmin=0;
// int vmax=255;

// int areamax=3000;
// int areamin=100;
// int Distance1=350;
// int Distance2=50;
int hmin,hmax,smin,smax,vmin,vmax;
int areamax,areamin,Distance1,Distance2;

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
void Detection(Mat imgDil,Mat img)
{//imgDil是传入的扩张边缘的图像用来查找轮廓，img是要在其上绘制轮廓的图像
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
        if (area > areamax)
        {
            //cout << area << endl;
            drawContours(img, contours, i, Scalar(0, 255, 255), 2);
        }
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"get_param");
    ros::NodeHandle nh;
    int capture_width = 640;
    int capture_height = 480;
    int display_width = 640;
    int display_height = 480;
    int framerate = 60;
    int flip_method = 0;

    ////创建管道
    string pipeline = gstreamer_pipeline(capture_width, capture_height, display_width, display_height, framerate, flip_method);
   // std::cout << "使用gstreamer管道: \n\t" << pipeline << "\n";

    //管道与视频流绑定
    VideoCapture capture(pipeline, CAP_GSTREAMER);
    if(!capture.isOpened())
    {
        std::cout<<"打开摄像头失败."<<std::endl;
        return (-1);
    }
    //获取视频基本信息
    int height = capture.get(CAP_PROP_FRAME_HEIGHT);
    int width = capture.get(CAP_PROP_FRAME_WIDTH);
    int count = capture.get(CAP_PROP_FRAME_COUNT);
    int fps = capture.get(CAP_PROP_FPS);
    cout << height<<"       "<< width<< "       " <<count<< "       " <<fps << endl;
    Mat frame,frame1,matrix, imgWarp, HSV, mask;
    Mat Dist = (Mat_<double>(5, 1) << -0.3466, 0.1347, 0, 0, 0);
    Mat K = (Mat_<double>(3, 3) << 396.362327865683, 0, 335.498095543887, 0, 528.487957995562, 238.650590420761, 0, 0, 1);
    Point2f src[4] = {{149, 294},
					  {491, 294},
				      {0,   480},
					  {640, 480}};//Point2f表示浮点数
    Point2f dst[4] = {{0, 0},
					  {300, 0},
					  {112, 300},
					  {188, 300}};//Point2f表示浮点数
    matrix = getPerspectiveTransform(src, dst);
    double FPS;
    char fstring[10];  // 用于存放帧率的字符串
    double t = 0;
    //循环读取视频
    while (true)
    {
        t = (double)cv::getTickCount();
        if (waitKey(1) == 27){ break; }
        int ret = capture.read(frame);
        if (!ret) {
            break;
        }
        undistort(frame,frame1,K,Dist);
        warpPerspective(frame1, imgWarp, matrix, Point(300,300));
        cvtColor(imgWarp, HSV, COLOR_BGR2HSV);//转换图像到HSV空间，在其中查找颜色更加容易
        medianBlur(HSV, HSV, 7);
        ros::param::get("hmin",hmin);
        ros::param::get("smin",smin);
        ros::param::get("vmin",vmin);
        ros::param::get("hmax",hmax);
        ros::param::get("smax",smax);
        ros::param::get("vmax",vmax);
        ros::param::get("AreaMax",areamax);
        // std::cout<<hmin<<" "<<hmax<<" "<<smin<<" "<<smax<<" "<<vmin<<" "<<vmax
        // <<" "<<areamax<<" "<<areamin<<" "<<Distance1<<" "<<Distance2<<endl;
        //////////////////////////////////////////////
        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        /////////////////////////////////////////////
        inRange(HSV, lower, upper, mask);//定义颜色下限和上限，因为由于照明和不同的阴影，颜色的值将不完全相同，会是一个值的范围
        Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7));//创建一个核，增加Size（只能是奇数）会扩张/侵蚀更多
        dilate(mask, mask, kernel);//扩张边缘（增加边缘厚度）
        dilate(mask, mask, kernel);//扩张边缘（增加边缘厚度）
        Detection(mask, imgWarp);
        //t该处代码执行所耗的时间,单位为秒
        t = ((double)getTickCount() - t) / getTickFrequency();
        //cout<<t<<endl;
        FPS = 1.0 / t;
        sprintf(fstring, "%.2f", FPS);// 帧率保留两位小数
        string sstring="FPS:";
        sstring += fstring;
        putText(imgWarp, sstring, Point(10, 50), FONT_HERSHEY_PLAIN, 3,Scalar(255, 255, 255));
        imshow("Image", imgWarp);
        // imshow("source",frame1);
        imshow("aaa", mask);
    }
}