//
// 从视频中透视变换
//

//使用方法：直接使用视频验证透视变换参数是否合理

#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

int main()
{

    VideoCapture capture("../resources/out.mp4");
    Mat frame,frame1,matrix,imgWarp;
    Mat Dist = (Mat_<double>(5, 1) << -0.3466, 0.1347, 0, 0, 0);
    Mat K = (Mat_<double>(3, 3) << 396.362327865683, 0, 335.498095543887, 0, 528.487957995562, 238.650590420761, 0, 0, 1);
    for (int i = 0; i <capture.get(CAP_PROP_FRAME_COUNT);i++)
    {
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
        undistort(frame,frame1,K,Dist);
        warpPerspective(frame1, imgWarp, matrix, Point(200,200));

        imshow("Image", frame1);
        imshow("Image Warp", imgWarp);
        waitKey(1);//增加延时
    }
}