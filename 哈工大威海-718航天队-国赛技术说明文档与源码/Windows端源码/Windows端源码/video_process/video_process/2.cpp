//
// 确定透视变换参数
//

//使用方法：先使用视野里含标准线条或方格的图片进行标定
//        随后使用抽帧图片进行验证，确定参数

#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int h1 = 281, h2 = 281, w1 = 191, w2 = 479;
int h3 = 200, h4 = 200, w3 = 75, w4 = 125;

int main()
{
    string path = "../resources/waican0.jpg";
    Mat img = imread(path);
    Mat img1;
    namedWindow("Trackbars", (640, 200));//(640,200)是尺寸

    createTrackbar("W1", "Trackbars", &w1, 640);
    createTrackbar("H1", "Trackbars", &h1, 480);

    createTrackbar("W2", "Trackbars", &w2, 640);
    createTrackbar("H2", "Trackbars", &h2, 480);

    createTrackbar("W3", "Trackbars", &w3, 200);
    createTrackbar("H3", "Trackbars", &h3, 200);

    createTrackbar("W4", "Trackbars", &w4, 200);
    createTrackbar("H4", "Trackbars", &h4, 200);
    Mat matrix, imgWarp;
    Mat Dist = (Mat_<double>(5, 1) << -0.3466, 0.1347, 0, 0, 0);
    Mat K = (Mat_<double>(3, 3) << 396.362327865683, 0, 335.498095543887, 0, 528.487957995562, 238.650590420761, 0, 0, 1);
    while (true)
    {
        Point2f src[4] = { {(float)w1,(float)h1},{(float)w2,(float)h2},{0,480},{640,480} };//Point2f表示浮点数
        Point2f dst[4] = { {0.0f,0.0f},{200,0.0f},{(float)w3,(float)h3},{(float)w4,(float)h4} };//Point2f表示浮点数
        matrix = getPerspectiveTransform(src, dst);
        undistort(img,img1,K,Dist);
        warpPerspective(img1, imgWarp, matrix, Point(200,200));

        imshow("Image", img);
        imshow("Image Warp", imgWarp);
        waitKey(1);//增加延时
    }
}