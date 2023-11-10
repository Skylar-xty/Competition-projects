//
// 从视频中寻找车道线
//

//使用方法：调整滑动条直到车道线连续且无杂点

#include<opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int hmin=100;
int hmax=124;
int smin=40;
int smax=141;
int vmin=0;
int vmax=255;
int areamax=150;
int areamin=4;
int Distance1=26;
int Distance2=0;

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
        if (area < areamax && area > areamin)
        {
            //cout << area << endl;
            drawContours(img, contours, i, Scalar(0, 255, 255), 2);
            float peri = arcLength(contours[i], true);//计算封闭轮廓周长
            approxPolyDP(contours[i], conPoly[i],0.03*peri,true);//以指定的精度近似多边形曲线。第二个参数conPloy[i]存储近似的结果，是输出。
            int objCor = (int)conPoly[i].size();
            if (objCor >= 4  && objCor <= 8)
            {
                //cout << objCor << endl;
                //    boundRect[i]=boundingRect(conPoly[i]);//计算边界矩形
                //    float aspRatio = (float)boundRect[i].width / (float)boundRect[i].height;//宽高比
                //    if (aspRatio > 0.5  && aspRatio < 4 )
                //    {
                //     rectangle(img,boundRect[i],Scalar(0, 255, 255),2);
                drawContours(img, conPoly, i, Scalar(0, 0, 255), -1);
                Mat tmp(contours.at(i));
                Moments moment=moments(tmp, false);
                if (moment.m00 != 0)//除数不能为0
                {
                    int x = cvRound(moment.m10 / moment.m00);//计算重心横坐标
                    int y = cvRound(moment.m01 / moment.m00);//计算重心纵坐标
                    if(x>=0 && x<=300 && y>=0 && y<=300)
                        points.push_back({x,y});

                }
                //    }
            }
        }
    }
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
    }
    if(usfulpoints.size()>0)
    {
        circle (img, usfulpoints[0], 3, Scalar(255, 255, 255), -1);
        //剔除二维数据离群点
        for(int i=0; i<points.size(); i++)
        {
            //cout<<usfulpoints[usfulpoints.size()-1].x<<"   "<<usfulpoints[usfulpoints.size(s)-1].y<<"   "<<points[i].x<<"   "<<points[i].y<<"   "<<dist(usfulpoints[usfulpoints.size()-1].x,usfulpoints[usfulpoints.size()-1].y,points[i].x,points[i].y)<<endl;
            double Dist = dist(usfulpoints[usfulpoints.size()-1],points[i]);
            if(Dist < Distance1 && Dist > Distance2)
            {
                //cout<<usfulpoints[usfulpoints.size()-1].x<<"   "<<usfulpoints[usfulpoints.size()-1].y<<"   "<<points[i].x<<"   "<<points[i].y<<"   "<<dist(usfulpoints[usfulpoints.size()-1].x,usfulpoints[usfulpoints.size()-1].y,points[i].x,points[i].y)<<endl;
                usfulpoints.push_back(points[i]);
                circle (img, points[i], 3, Scalar(255, 255, 255), -1);
            }
        }
        cout<<"usfulpoints:"<<usfulpoints.size()<<endl;
    }
    //绘制折线
    polylines(img, usfulpoints, false, Scalar(0, 255, 0), 6, 4, 0);
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
    createTrackbar("AreaMax", "Trackbars", &areamax, 1000);
    createTrackbar("AreaMin", "Trackbars", &areamin, 500);
    createTrackbar("DistMax", "Trackbars", &Distance1, 500);
    createTrackbar("DistMin", "Trackbars", &Distance2, 500);
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
        medianBlur(HSV, HSV, 5);
        //////////////////////////////////////////////
        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        /////////////////////////////////////////////
        inRange(HSV, lower, upper, mask);//定义颜色下限和上限，因为由于照明和不同的阴影，颜色的值将不完全相同，会是一个值的范围
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));//创建一个核，增加Size（只能是奇数）会扩张/侵蚀更多
        dilate(mask, mask, kernel);//扩张边缘（增加边缘厚度）
        erode(mask, mask, kernel);//侵蚀边缘（减小边缘厚度）
        dilate(mask, mask, kernel);//扩张边缘（增加边缘厚度）
        erode(mask, mask, kernel);//侵蚀边缘（减小边缘厚度）
        Detection(mask, imgWarp);
        //t该处代码执行所耗的时间,单位为秒
        t = ((double)getTickCount() - t) / getTickFrequency();
        FPS = 1.0 / t;
        sprintf(fstring, "%.2f", FPS);// 帧率保留两位小数
        string sstring="FPS:";
        sstring += fstring;
        putText(imgWarp, sstring, Point(10, 50), FONT_HERSHEY_PLAIN, 2,Scalar(255, 255, 255));
        imshow("Image", imgWarp);
        imshow("aaa", mask);
    }
}