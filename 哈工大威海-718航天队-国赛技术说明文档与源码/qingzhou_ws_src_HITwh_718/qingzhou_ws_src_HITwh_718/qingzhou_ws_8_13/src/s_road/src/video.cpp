#include <iostream>
#include <string>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/objdetect.hpp>
#include <opencv4/opencv2/imgproc/types_c.h>
#include <opencv4/opencv2/videoio.hpp>

using namespace std;
using namespace cv;

string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + ", height=(int)" +
           to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + to_string(flip_method) + " ! video/x-raw, width=(int)" + to_string(display_width) + ", height=(int)" +
           to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main( int argc, char** argv )
{
    int capture_width = 640 ;
    int capture_height = 480 ;
    int display_width = 640 ;
    int display_height = 480 ;
    int framerate = 60 ;
    int flip_method = 0 ;

    //创建管道
    string pipeline = gstreamer_pipeline(capture_width,
    capture_height,
    display_width,
    display_height,
    framerate,
    flip_method);
    std::cout << "使用gstreamer管道: \n\t" << pipeline << "\n";

    //管道与视频流绑定
    VideoCapture cam(pipeline, CAP_GSTREAMER);
    if(!cam.isOpened())
    {
        std::cout<<"打开摄像头失败."<<std::endl;
        return (-1);
    }
	cout << "cam open success!" << endl;
	// namedWindow("cam");
	Mat img;
	VideoWriter vw;
	int fps = cam.get(CAP_PROP_FPS);  //获取摄像机帧率
	if (fps <= 0)fps = 25;
	//创建视频文件
	vw.open("/home/t718/qingzhou_ws/src/s_road/out.mp4", //路径
		VideoWriter::fourcc('m', 'p', '4', 'v'), //编码格式
		fps, //帧率
		Size(cam.get(CAP_PROP_FRAME_WIDTH),
			cam.get(CAP_PROP_FRAME_HEIGHT))  //尺寸
		);
	if (!vw.isOpened())
	{
		cout << "VideoWriter open failed!" << endl;
		getchar();
		return -1;
	}
	cout << "VideoWriter open success!" << endl;
 
	for (int i=0;i<1200;i++)
	{
		cam.read(img);
		if (img.empty())break;
		// imshow("cam", img);
		//写入视频文件
		vw.write(img);
		// if (waitKey(5) == 'q') break;
	}
 
	// waitKey(0);
	return 0;
}