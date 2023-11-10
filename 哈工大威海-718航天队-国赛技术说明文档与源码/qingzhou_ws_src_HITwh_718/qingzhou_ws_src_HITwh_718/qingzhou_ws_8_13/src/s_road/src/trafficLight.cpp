#define _CRT_SECURE_NO_WARNINGS 1

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <ros/ros.h>
#include "std_msgs/Int32.h"


using namespace std;
using namespace cv;


cv::Mat intrinsics = (Mat_<double>(3, 3) << 407.92783, 0, 337.47034, 0, 536.57488, 251.33952, 0, 0, 1);
cv::Mat distCoeffs = (Mat_<double>(5, 1) << -0.31536, 0.08824, 0.00162, -0.00021, 0.0);
bool Tstart;

string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + ", height=(int)" +
        to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + to_string(framerate) +
        "/1 ! nvvidconv flip-method=" + to_string(flip_method) + " ! video/x-raw, width=(int)" + to_string(display_width) + ", height=(int)" +
        to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int hminj[3] = { 57,0,167 }, sminj[3] = { 150,130,92 }, vminj[3] = { 55,108,72 };
int hmaxj[3] = { 85,23,179 }, smaxj[3] = { 255,255,255 }, vmaxj[3] = { 255,255,255 };
int hminy[3] = { 58,0,167 }, sminy[3] = { 100,89,92 }, vminy[3] = { 55,108,71 };
int hmaxy[3] = { 85,20,179 }, smaxy[3] = { 255,159,255 }, vmaxy[3] = { 255,255,255 };

//int hminj[3] = { 58,0,169 }, sminj[3] = { 126,130,120 }, vminj[3] = { 103,108,116 };
//int hmaxj[3] = { 82,23,179 }, smaxj[3] = { 255,255,255 }, vmaxj[3] = { 255,255,255 };
//int hminy[3] = { 58,0,167 }, sminy[3] = { 54,89,101 }, vminy[3] = { 72,108,116 };
//int hmaxy[3] = { 82,20,179 }, smaxy[3] = { 202,159,212 }, vmaxy[3] = { 255,255,255 };

//int hminj[3] = { 53,0,113 }, sminj[3] = { 231,255,156 }, vminj[3] = { 41,60,71 };
//int hmaxj[3] = { 80,30,179 }, smaxj[3] = { 255,255,255 }, vmaxj[3] = { 255,255,255 };
//int hminy[3] = { 48,0,113 }, sminy[3] = { 0,255,156 }, vminy[3] = { 18,60,71 };
//int hmaxy[3] = { 93,30,179 }, smaxy[3] = { 255,255,255 }, vmaxy[3] = { 200,255,255 };

struct candidatemarker
{
    int condidateId;
    int right;
};

bool state;
ros::Publisher light_pub;
//判断颜色
int getcolor(Mat imgHSV, Mat img, Mat imgnew, int z, Point t)
{
    int maxarea = 0;
    int targetcolor = -1;
    Rect newRect;
    Mat imgDil, imgBlur, mask;
    for (int flag = 0; flag <= 2; flag+=2)
    {
        Scalar lower, upper;
        if (z < 190)
        {
            lower = Scalar(hminj[flag], sminj[flag], vminj[flag]);
            upper = Scalar(hmaxj[flag], smaxj[flag], vmaxj[flag]);
        }
        else
        {
            lower = Scalar(hminy[flag], sminy[flag], vminy[flag]);
            upper = Scalar(hmaxy[flag], smaxy[flag], vmaxy[flag]);
        }
        //二值化
        inRange(imgHSV, lower, upper, mask);
        //中值滤波
        medianBlur(mask, imgBlur, 3);
        //膨胀
        Mat kernel0 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        dilate(imgBlur, imgDil, kernel0);
        //腐蚀
        Mat kernel1 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
        erode(imgDil, imgDil, kernel1);
        
        medianBlur(imgDil, imgDil, 3);
        dilate(imgDil, imgDil, kernel0);
        dilate(imgDil, imgDil, kernel0);
		//    if (color == 2)    imshow("imgDil", imgDil);
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        findContours(imgDil, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
        vector<vector<Point>> conPoly(contours.size());
        vector<Rect> boundRect(contours.size());
        int height = img.rows;
        for (int i = 0; i < contours.size(); i++)
        {
            int area = contourArea(contours[i]);
            //灯的面积大于100
            if (area > 100)
            {
                //图形周长
                float peri = arcLength(contours[i], true);
                approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
                boundRect[i] = boundingRect(conPoly[i]);

                int corner = (int)conPoly[i].size();
                //高度比宽度
                float frac = (float)boundRect[i].height / (float)boundRect[i].width;

                if (corner > 4 && frac < 1.7 && frac>0.588)
                {
                    double H, S, V;
                    int cnt = 0;
                    //分别代指左上点以及右下点，求出中心点
                    Point c = (boundRect[i].tl() + boundRect[i].br()) / 2;

                    int croprows = imgHSV.rows;
                    int cropcols = imgHSV.cols;

                    if ((flag == 0 && c.y > height / 2) || (flag == 2 && c.y < height * 3 / 5) || (flag == 1 && c.y > height / 3))
                    {
                        //在中心点的左上以及右下个各找出一个点，并将他们构成矩形
                        Point lightzsp(c.x - 11, c.y - 11), lightyxp(c.x + 11, c.y + 11);
                        if (lightzsp.x < 0)    lightzsp.x = 0;
                        if (lightzsp.y < 0)    lightzsp.y = 0;
                        if (lightyxp.x > cropcols) lightyxp.x = cropcols;
                        if (lightyxp.y > croprows) lightyxp.y = croprows;
                        Rect lightroi(lightzsp, lightyxp);

                        Mat lightcrop = img(lightroi);
                        resize(lightcrop, lightcrop, Size(41, 41));
                        Mat lightcropHSV;
                        cvtColor(lightcrop, lightcropHSV, COLOR_BGR2HSV);
                        //行
                        int lightcroprows = lightcrop.rows;
                        //列
                        int lightcropcols = lightcrop.cols;
                        for (int dx = -11; dx <= 11; dx += 2)
                        {
                            int tempx, tempy;
                            tempx = 21 + dx;
                            tempy = 21 + dx;
                            if (tempy < lightcroprows && tempy>0 && tempx < lightcropcols && tempx>0)
                            {
                                
                                V = lightcropHSV.at<Vec3b>(tempy, tempx)[2];
                                if (V > 195)
                                {
                                    cnt++;
                                    if (cnt > 1) break;
                                }
                            }
                            tempy = 21 - dx;
                            if (tempy < lightcroprows && tempy>0 && tempx < lightcropcols && tempx>0)
                            {
                                V = lightcropHSV.at<Vec3b>(tempy, tempx)[2];
                                if (V > 195)
                                {
                                    cnt++;
                                    if (cnt > 1) break;
                                }
                            }
                        }
                    }
                    if (cnt > 1)
                    {

                        if (area > maxarea)
                        {
                            maxarea = area;
                            targetcolor = flag;
                            newRect = boundRect[i];

                        }
                    }
                }
            }
        }
    }
    if (targetcolor != -1)
    {
        string colortype;
        Scalar color;
        switch (targetcolor)
        {
        case 0:colortype = "Green"; color = Scalar(0, 255, 0); break;
        case 1:colortype = "Yellow"; color = Scalar(32, 165, 218); break;
        case 2:colortype = "Red"; color = Scalar(0, 69, 255); break;
        default:break;
        }
        newRect.x = newRect.x * 0.5 + t.x;
        newRect.y = newRect.y * 0.5 + t.y;
        newRect.height = newRect.height / 2;
        newRect.width = newRect.width / 2;


        //    rectangle(img, boundRect[i], Scalar(0, 255, 0), 3);
        rectangle(imgnew, newRect, Scalar(0, 255, 0), 3);
        //    putText(img, colortype, boundRect[i].tl(), FONT_HERSHEY_DUPLEX, 0.75, Scalar(0, 69, 255), 2);
        putText(imgnew, colortype, Point(10, 80), FONT_HERSHEY_DUPLEX, 1, color, 2);
		// imshow("imgnew",imgnew);
	return targetcolor;
    }
    return -1;
}

int dic0[6][6] = { 0, 0, 0, 0, 0, 0,
                   0, 1, 0, 1, 1, 0,
                   0, 0, 1, 0, 1, 0,
                   0, 0, 0, 1, 1, 0,
                   0, 0, 0, 1, 0, 0,
                   0, 0, 0, 0, 0, 0 };
int dic1[6][6] = { 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 1, 1, 1, 1, 0,
                   0, 1, 0, 0, 1, 0,
                   0, 1, 0, 1, 0, 0,
                   0, 0, 0, 0, 0, 0 };
/*
近处对码图进行处理
输入：透射变换后的二值化码图
返回值：码图对应编号以及符合的像素点数
*/
candidatemarker decodenear(Mat img)
{
    int right0 = 0, right1 = 0;
    int blackcnt = 0;
    candidatemarker result;
    for (int i = 5; i <= 54; i += 9)
    {
        for (int j = 5; j <= 54; j += 9)
        {
            int imgpix = (int)img.at<uchar>(i, j);
            if (imgpix == 255) imgpix = 1;
            else    blackcnt++;
            if (imgpix == dic0[i / 9][j / 9])
            {
                right0++;
            }
            if (imgpix == dic1[i / 9][j / 9])
            {
                right1++;
            }
        }
    }
    if (blackcnt > 33) result.condidateId = -1;
    if (right0 > right1 && right0 > 24)
    {
        result.condidateId = 0;
        result.right = right0;
    }
    else if (right1 > right0 && right1 > 24)
    {
        result.condidateId = 1;
        result.right = right1;
    }
    else result.condidateId = -1;
    return result;
}
/*远处对码图的处理*/ 
candidatemarker decodefar(Mat img)
{
    int right0 = 0, right1 = 0;
    int blackcnt = 0;
    int imgpix, sum, tempimgpix;
    candidatemarker result;
    for (int i = 5; i <= 54; i += 9)
    {
        for (int j = 5; j <= 54; j += 9)
        {
            sum = 0;
            for (int tx = i - 1; tx <= i + 1; tx++)
            {
                for (int ty = j - 1; ty <= j + 1; ty++)
                {
                    tempimgpix = (int)img.at<uchar>(tx, ty);
                    if (tempimgpix == 255)
                    {
                        sum++;
                    }
                }
            }
            if (sum > 4) imgpix = 1;
            else
            {
                blackcnt++;
                imgpix = 0;
            }
            if (imgpix == dic0[i / 9][j / 9])
            {
                right0++;
            }
            if (imgpix == dic1[i / 9][j / 9])
            {
                right1++;
            }
        }
    }
    if (blackcnt > 33) result.condidateId = -1;
    if (right0 > right1 && right0 > 24)
    {
        result.condidateId = 0;
        result.right = right0;
    }
    else if (right1 > right0 && right1 > 24)
    {
        result.condidateId = 1;
        result.right = right1;
    }
    else result.condidateId = -1;
    return result;
}
/*求解两点距离*/
double PointDist(const cv::Point2f& a, const cv::Point2f& b)
{
    double res = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    return res;
}

vector<int> lastmarkerIds;
vector< vector<Point2f> > lastmarkerCorners;
Point d[2], t, zs, yx;
double x, y, z, lx, ly, lz, llx, lly, llz;
int length;
/*对图像进行总处理*/
void detectAruco(Mat imgDil, Mat img) //imgDil是形态学处理后的图像，img是未处理的输出图像
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(imgDil, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    vector<vector<Point>> conPoly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector< vector<Point2f> > markerCorners;
    vector<int> markerIds;
    string shapetype;
    std::vector<cv::Vec3d> rvecs, tvecs;
    int cnt = 0;
    int maxright0 = -1, maxright1 = -1;
    int getflag0 = 0, getflag1 = 0;
    vector<Point2f> tempCorners0, tempCorners1;
    /*读取图片中的码图，并判断码图类型*/ 
    for (int i = 0; i < contours.size(); i++)
    {
        int area = contourArea(contours[i]);

        if (area > 100)
        {
            float peri = arcLength(contours[i], true);
            /*拐点*/
            approxPolyDP(contours[i], conPoly[i], 0.042 * peri, true);
            /*多边形拟合后的点集*/
            boundRect[i] = boundingRect(conPoly[i]);
            int corner = conPoly[i].size();
            /*勾勒出各个拐点框*/
            // drawContours(img, conPoly, i, Scalar(255, 0, 255), 2);
            if (corner == 4) {
                /*高度和宽度的比值*/
                float frac = (float)boundRect[i].height / (float)boundRect[i].width;
                double lenth_1 = PointDist(conPoly[i][0], conPoly[i][1]);
                double lenth_2 = PointDist(conPoly[i][1], conPoly[i][2]);
                /*使用长宽比以及拐点长度之比限制*/
                if (frac < 2 && frac > 0.5 && lenth_1 / lenth_2 < 2 && lenth_1 / lenth_2 > 0.5)
                {
                    int maxyx = -10000, minzs = 10000, maxzx = -10000, maxys = -10000, zsp, zxp, ysp, yxp;
                    for (int k = 0; k < 4; k++)
                    {
                        /*求解最大坐标和x+y，代表右下方的坐标*/
                        if (conPoly[i][k].x + conPoly[i][k].y > maxyx)
                        {
                            maxyx = conPoly[i][k].x + conPoly[i][k].y;
                            yxp = k;
                        }
                        /*求解最小坐标和x+y，代表左上方的坐标*/
                        if (conPoly[i][k].x + conPoly[i][k].y < minzs)
                        {
                            minzs = conPoly[i][k].x + conPoly[i][k].y;
                            zsp = k;
                        }
                        /*求解最大坐标差x-y，代表右上方坐标*/
                        if (conPoly[i][k].x - conPoly[i][k].y > maxys)
                        {
                            maxys = conPoly[i][k].x - conPoly[i][k].y;
                            ysp = k;
                        }
                        /*求解最大坐标差y-x，代表左下方坐标*/
                        if (conPoly[i][k].y - conPoly[i][k].x > maxzx)
                        {
                            maxzx = conPoly[i][k].y - conPoly[i][k].x;
                            zxp = k;
                        }
                    }
                    /*将四个点的数据存储起来,按照自左上开始顺时针方向*/
                    vector<Point2f> tempCorners;
                    tempCorners.push_back((Point2f)conPoly[i][zsp]);
                    tempCorners.push_back((Point2f)conPoly[i][ysp]);
                    tempCorners.push_back((Point2f)conPoly[i][yxp]);
                    tempCorners.push_back((Point2f)conPoly[i][zxp]);
                    
                    float w = 54, h = 54;
                    Mat markerWarp, prematrix;
                    Point2f src[4] = { tempCorners[0],tempCorners[1],tempCorners[2],tempCorners[3] };
                    Point2f dst[4] = { {0.0f,0.0f},{w - 1,0.0f},{w - 1,h - 1},{0.0f,h - 1} };
                    /*将图像透视变换成54x54的大小*/
                    prematrix = getPerspectiveTransform(src, dst);                  //变换矩阵
                    cv::warpPerspective(imgDil, markerWarp, prematrix, Size(w, h));

                    candidatemarker testmarker;
                    if (z < 170)
                    {
                        testmarker = decodenear(markerWarp);
                    }
                    else
                    {
                        testmarker = decodefar(markerWarp);
                    }
              
                    if (testmarker.condidateId == 0)
                    {
                        if (testmarker.right > maxright0)
                        {
                            tempCorners0 = tempCorners;
                            maxright0 = testmarker.right;
                            getflag0 = 1;
                        }
                    }
                    if (testmarker.condidateId == 1)
                    {
                        if (testmarker.right > maxright1)
                        {
                            tempCorners1 = tempCorners;
                            maxright1 = testmarker.right;
                            getflag1 = 1;
                        }
                    }
                    // imshow("markerWarp",markerWarp);
                }
            }
        }
    }
    if (getflag0 == 1)
    {
        markerIds.push_back(0);
        markerCorners.push_back(vector<Point2f>());
        //输出符合要求的点的坐标
        ROS_INFO("markerCorners:   ");
        for (int k = 0; k < 4; k++)
        {
            markerCorners[cnt].push_back(tempCorners0[k]);//一个二维的容器存储四个点
            cout << "    " << markerCorners[cnt][k];
        }
        cout << endl;
        cnt++;

    }
    if (getflag1 == 1)
    {
        markerIds.push_back(1);
        markerCorners.push_back(vector<Point2f>());
        for (int k = 0; k < 4; k++)
        {
            markerCorners[cnt].push_back(tempCorners1[k]);
        }
        cnt++;
    }
    cout << "cnt :" << cnt << endl;
    if (cnt == 2)
    {   
        /*判断前后两次左上的点和右上的点的距离是否小于判断要求*/
        if (PointDist(markerCorners[1][0], markerCorners[0][1]) < 20)
        {
            lastmarkerCorners = markerCorners;
            lastmarkerIds = markerIds;
        }
        else
        {
            if (lastmarkerCorners.size() > 1)
            {
                markerCorners = lastmarkerCorners;
                markerIds = lastmarkerIds;
            }
            else
            {
                cnt = 0;
            }
        }
    }
    else
    {
        //    return;
        if (lastmarkerCorners.size() > 1 && cnt > 0)
        {
            markerCorners = lastmarkerCorners;
            markerIds = lastmarkerIds;
        }
    }
    cout << "cnt :" << cnt <<"markerCorners.size(): "<<markerCorners.size()<< endl;
    if (markerCorners.size() > 1 && cnt > 0)
    {
        cout << "enter" << endl;
        /*在原图上框出目标矩形*/
        aruco::drawDetectedMarkers(img, markerCorners, markerIds);
        /*亚像素角点检测*/
        for (int i = 0; i < 2; i++)
        {
            cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 0.9);
            Mat imgmarkerGray;
            cvtColor(img, imgmarkerGray, COLOR_BGR2GRAY);
            cv::cornerSubPix(imgmarkerGray, markerCorners[i], cv::Size(3, 3), cv::Size(-1, -1), criteria);
        }

        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.07, intrinsics, distCoeffs, rvecs, tvecs);//求解旋转矩阵rvecs和平移矩阵tvecs
        //画出坐标轴
    //    for (int i = 0; i < 2; i++)
            // cv::aruco::drawAxis(img, intrinsics, distCoeffs, rvecs[i], tvecs[i], 0.05);

        x = 0; y = 0; z = 0;
        for (unsigned int i = 0; i < 2; i++)
        {
            cv::Mat rmat;
            Rodrigues(rvecs[i], rmat);

            length = markerCorners[i][1].x - markerCorners[i][0].x;

            d[markerIds[i]] = (markerCorners[i][0] + markerCorners[i][1]) / 2;

            x = x + tvecs[i][0];
            y = y + tvecs[i][1];
            z = z + tvecs[i][2];
        }
        x = x * 100 / 2;
        y = y * 100 / 2;
        z = z * 100 / 2;

        if (llx == 0 && lly == 0 && llz == 0)
        {
            llx = x;
            lly = y;
            llz = z;
            return;
        }
        if (lx == 0 && ly == 0 && lz == 0)
        {
            lx = x;
            ly = y;
            lz = z;
            return;
        }
        x = (x + lx + llx) / 3;
        llx = lx, lx = x;
        y = (y + ly + lly) / 3;
        lly = ly, ly = y;
        z = (z + lz + llz) / 3;
        llz = lz; lz = z;

        char position[41];
        /*输出位置*/
        sprintf(position, "position[%d %d %d]", (int)x, (int)y, (int)z);
        // cv::putText(img, position, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
    	cout<<position<<endl;

	}
    //显示图片（j）
    // imshow("img",img);//原图像
    //imshow("imgDil",imgDil);//灰度图片
}

int mapflag;
Mat map1, map2;
/*图像的坐标映射*/
void undist(Mat img, Mat imgdistort)
{
    if (mapflag == 0)
    {
        cv::Size imageSize(640, 480);
        cv::Mat NewCameraMatrix = getOptimalNewCameraMatrix(intrinsics, distCoeffs, imageSize, 1, imageSize);
        initUndistortRectifyMap(intrinsics, distCoeffs, cv::Mat(), NewCameraMatrix, imageSize, CV_16SC2, map1, map2);
        mapflag = 1;
    }
    remap(img, imgdistort, map1, map2, cv::INTER_LINEAR);
    return;
}

void poseCallback()
{
    // 将接收到的消息打印出来
    ROS_INFO("Now open the camera !\n");
    
    Mat Image, ImgCrop1, imgHSV;
    Mat ImageCopy, imgpreGray, imgpreBlur, imgThre;
    z = 200;
    lx = 0, ly = 0, lz = 0, llx = 0, lly = 0, llz = 0;

    state=true;	
   
    int capture_width = 640;          //摄像头捕获的宽度
    int capture_height = 480;         //摄像头捕获的高度
    int display_width = 640;          //窗口的宽度
    int display_height = 480;         //窗口的高度
    int framerate = 60;               //捕获频率
    int flip_method = 0;              //是否旋转图像

    ////创建管道
    string pipeline = gstreamer_pipeline(capture_width, capture_height, display_width, display_height, framerate, flip_method);
    std::cout << "使用gstreamer管道: \n\t" << pipeline << "\n";

    //管道与视频流绑定
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cout << "打开摄像头失败." << std::endl;
        return ;
    }
    // string path = "/home/t718/biaoding/ArUco0.jpg";
    // Image = imread(path);
    while (Tstart)
    {
        if (!cap.read(Image))
        {
            std::cout << "捕获失败" << std::endl;
            break;
        }

		//图像调试 
        //string path = "lightphoto/nvcamtest_10528_s00_00010.jpg";
        // string path = "biaoding/ArUco0.jpg";
        // Image = imread(path);
        
        //视频调试 
        //  string path = "/home/mmzyyds/testopencv/VideoWrite/Traffic_light.avi";
        //  VideoCapture cap(path);
        
        int frame = 0;
        while (Tstart) {
       //     double start, stop, durationTime;
       //     start = clock();
       //     cout << "frame: " << ++frame << endl;

            cap.read(Image);
            cap.read(Image);
            if(!cap.read(Image)){
             cout<<"读取图片失败"<<endl;
             continue;   
            }
            undist(Image, Image);
            Image.copyTo(ImageCopy);

            cvtColor(ImageCopy, imgpreGray, COLOR_BGR2GRAY);
         //   adaptiveThreshold(imgpreGray, imgThre, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 51, -9);  //自适应阈值 
         // threshold(imgpreGray, imgThre, 0, 255, THRESH_BINARY|CV_THRESH_OTSU);						   //大津法OTSU 
        
            threshold(imgpreGray, imgThre, 95, 255, THRESH_BINARY);
        //    imshow("Imgthreshold", imgThre);

            detectAruco(imgThre, Image);
            //0号码上边中点对应x
            t.x = d[0].x;
            //0号码的上边重点的y数值上移4个边长
            t.y = d[0].y - 4 * length;
            //取到了两个码上边中点的分别左右偏移5个像素
            t.x -= 5; d[1].x += 5;

            if (t.y < 0) t.y = 1;
	        if(t.x<0) t.x=1;
	        if(d[1].x>639) d[1].x=639;
            //以刚才构造的点成为矩形
            Rect roi(t, d[1]);
            //构造出的矩形是需要高度大于宽度的
            if (roi.height > roi.width && roi.height > 0 && roi.width > 0)
            {
                //取到红绿灯的范围
                ImgCrop1 = ImageCopy(roi);
                
                resize(ImgCrop1, ImgCrop1, Size(), 2, 2);
                // imshow("红绿灯框图",ImgCrop1);
                cvtColor(ImgCrop1, imgHSV, COLOR_BGR2HSV);
     		
		        int color=-1;
                color=getcolor(imgHSV, ImgCrop1, Image, z, t);
                if(color!=-1)
                {
                    cout<<"color is "<<color<<endl; 
                    if(color==2&&z<133&&state!=false)
                    {
                        std_msgs::Int32 stopflag;
                        state=false;
                        stopflag.data=false;
                        // light_pub.publish(stopflag);
                        ros::param::set("TrafficFlag",1);
                        for(int i=0;i<500;i++);
                        ros::param::set("TrafficFlag",1);
                        ROS_INFO("The state of car is stop!\n");
                    }
                    if(color==0&&z<133)
                    {
                        std_msgs::Int32 stopflag;
                        state=true;
                        stopflag.data=true;
                        // light_pub.publish(stopflag);
                        ros::param::set("TrafficFlag",2);
                        for(int i=0;i<500;i++);
                        ros::param::set("TrafficFlag",2);
                        // cap.release();
                        ROS_INFO("The state of car is go!\n");
                        cap.release();
                    }
                // if(color==0&&z<120&&state==true)
                // {
                //     // cap.release();
                // 	return ;
                // }
                
                }

                   //      imshow("ImageCrop", ImgCrop1);
            }
            // imshow("ImageCopy", Image);//显示marker  
			// imshow("2",imgThre);
        //    stop = clock();
        //    durationTime = ((double)(stop - start)) / 1000;
        //    cout << "time is " << durationTime << " s" << endl;
            waitKey(1);
            ros::param::get("Tstart",Tstart);
        }
        ros::param::get("Tstart",Tstart);
    }
    
}



int main(int argc,char **argv)
{ 
    int change=1;
    // ROS节点初始化
    ros::init(argc, argv, "Traffic_Light_publisher");

    // 创建节点句柄
    ros::NodeHandle n;
    bool exit;
    exit = false;
    ros::param::set("exit",exit);
    
    // light_pub = n.advertise<std_msgs::Int32>("/stm32_stop", 10);
    
    // ros::Subscriber light_sub = n.subscribe("/traffic_flag", 10, poseCallback);
    cout<<"wait /traffic_flag"<<endl;
    while(!exit)
    {
        ros::param::get("Tstart",Tstart);
        if(!Tstart){
            if(change==1){
                ROS_INFO("Tstart=0\n");
                change=0;
            }
        }
        else{
          change=1;
        }
        if(Tstart==true){
            poseCallback();
        }
        else sleep(0.3);
        ros::param::set("TrafficFlag",0);
        
        ros::param::get("exit",exit);
        // lx=0;ly=0;lz=0; llx=0; lly=0; llz=0;
        
    }
    ros::param::set("exit",false);

    // ros::spin();

    return 0;
}



