//
// 从视频中抽帧并储存
//

//使用方法：先运行小车s_road功能包下video文件录制1200帧视频(约20s)
//        随后下载视频放入resources文件夹下进行抽帧

#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

int main()
{

    VideoCapture capture("../resources/out.mp4");
    Mat frame;
    for (int i = 0; i <capture.get(CAP_PROP_FRAME_COUNT);i++)
    {
        capture.read(frame);
        stringstream str;
        //设置每50帧获取一次帧
        if (i % 50 == 0)
        {
            str << "f" << i << ".jpg";
            cout << "正在处理第" << i << "帧" << endl;
            imwrite("../resources/" + str.str(), frame); // 将帧转成图片输出
        }
    }
}