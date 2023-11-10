//
// 从视频寻找起点
//

//使用方法：调整滑动条直到起点处存在大面积蓝色或红色
//        使用areamax估计其面积

#include<opencv2/opencv.hpp>

using namespace cv;
using namespace std;

//红
//int hmin=0;
//int hmax=21;
//蓝
int hmin=100;
int hmax=124;

int smin=25;
int smax=141;
int vmin=0;
int vmax=255;
int areamax=100;
//int areamax=800;

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
    namedWindow("Trackbars", (640, 400));//(640,200)是尺寸
    //运行时，把3个min的都移到最小值，把3个max的都移到最大值，然后移动使其保持为白色
    createTrackbar("Hue Min", "Trackbars", &hmin, 179);//对于hue色相饱和度最大180,对于另外两个色相饱和度最大255
    createTrackbar("Hue Max", "Trackbars", &hmax, 179);
    createTrackbar("Sat Min", "Trackbars", &smin, 255);
    createTrackbar("Sat Max", "Trackbars", &smax, 255);
    createTrackbar("Val Min", "Trackbars", &vmin, 255);
    createTrackbar("Val Max", "Trackbars", &vmax, 255);
    createTrackbar("AreaMax", "Trackbars", &areamax, 1500);
    VideoCapture capture("../resources/out.mp4");
    //获取视频基本信息
    int height = capture.get(CAP_PROP_FRAME_HEIGHT);
    int width = capture.get(CAP_PROP_FRAME_WIDTH);
    int count = capture.get(CAP_PROP_FRAME_COUNT);
    int fps = capture.get(CAP_PROP_FPS);
    cout << height<<"       "<< width<< "       " <<count<< "       " <<fps << endl;
    Mat frame,frame1,matrix, imgWarp, HSV, mask;
    Mat Dist = (Mat_<double>(5, 1) << -0.3466, 0.1347, 0, 0, 0);
    Mat K = (Mat_<double>(3, 3) << 396.362327865683, 0, 335.498095543887, 0, 528.487957995562, 238.650590420761, 0, 0, 1);
    capture.read(frame);
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
        warpPerspective(frame1, imgWarp, matrix, Point(200,200));
        cvtColor(imgWarp, HSV, COLOR_BGR2HSV);//转换图像到HSV空间，在其中查找颜色更加容易
        medianBlur(HSV, HSV, 7);
        //////////////////////////////////////////////
        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        /////////////////////////////////////////////
        inRange(HSV, lower, upper, mask);//定义颜色下限和上限，因为由于照明和不同的阴影，颜色的值将不完全相同，会是一个值的范围
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));//创建一个核，增加Size（只能是奇数）会扩张/侵蚀更多
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
        putText(imgWarp, sstring, Point(10, 50), FONT_HERSHEY_PLAIN, 2,Scalar(255, 255, 255));
        imshow("Image", imgWarp);
        // imshow("source",frame1);
        imshow("aaa", mask);
    }
}