#!/usr/bin/env python
#coding:utf-8
import rospy
import socket
import threading
import struct
import time
import cv2
import numpy
from geometry_msgs.msg import PoseStamped

# 此程序在Ubuntu20.04中测试通过，ROS版本为Noetic
# 一般情况下，服务器端父线程处于while(1)循环中，使用listen阻塞进程等待客户端连接，
# 连接后同样有一个while(1)循环，等待客户端消息，若为视频连接请求则开启子线程，同时继续while循环，
# 若收到断开连接请求则退出此循环，关闭socket并回到上层循环。回到上层循环后会再次初始化socket关键字，继续listen客户端连接；
# 而若在连接有客户端的条件下收到结束程序请求，服务端会在关闭socket套接字后结束程序并退出

#########GetIP##########
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8",80))
local_addr=s.getsockname()[0]
s.close()

# 设置gstreamer管道参数
def gstreamer_pipeline(
    capture_width=640, #摄像头预捕获的图像宽度
    capture_height=480, #摄像头预捕获的图像高度
    display_width=640, #窗口显示的图像宽度
    display_height=480, #窗口显示的图像高度
    framerate=60,       #捕获帧率
    flip_method=0,      #是否旋转图像
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )  

class command(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.a = 0.0
        self.w = 0.0
        self.r = 0.0
        self.v = 0.0
class Carame_Accept_Object:
    def __init__(self, S_addr_port=(local_addr, 9999)):  # S_addr_port 是一个元组  ，self初始化对应的实例变量
        self.img_fps=30
        self.resolution=(640,480)       #执行该实例化的分辨率，self相当于int，定义的意思
        self.addr_port=S_addr_port      #地址绑定，addr_port 对应的端口号
        self.Set_Socket(self.addr_port)  #建立连接，调用一个函数
        #设置套接字
    def Set_Socket(self,S_addr_port):
        self.server=socket.socket(socket.AF_INET,socket.SOCK_STREAM)      #选择TCP协议连接
        self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) #端口可复用
        self.server.bind(S_addr_port)            #绑定端口号
        self.server.listen(5)                   #监听端口号
        print("视频端口:%d" % S_addr_port[1])     
            
    def check_option(object,client):                  #按格式解码，确定帧数和分辨率
        info=struct.unpack('ihh',client.recv(8))       #将接受到的传输帧数，分辨率解码
        if info[0]>888:                               #若第一个数值
            object.img_fps=int(info[0])-888            #获取帧数
            object.resolution=list(object.resolution)    # 获取分辨率
            object.resolution[0]=info[1]
            object.resolution[1]=info[2]
            object.resolution = tuple(object.resolution)
            return 1
        else:
            print("数据错误！")
            return 0
                
class MyThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.flag = True
    def run(self):
        camera = Carame_Accept_Object()  # 执行这个类
        f = self.flag
        while (f):
            f = self.flag
            client, D_addr = camera.server.accept()  # 服务器接受到来自客户端的连接，收到客户端的信息为
            video = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)  # 从摄像头中获取视频
            img_param = [int(cv2.IMWRITE_JPEG_QUALITY), camera.img_fps]  # 设置传送图像格式、帧数
            while (f):
                f = self.flag
                time.sleep(0.1)  # 推迟线程运行0.1s
                _, camera.img = video.read()  # 读取视频每一帧
                camera.img = cv2.resize(camera.img, camera.resolution)  # 按要求调整图像大小(resolution必须为元组)
                _, img_encode = cv2.imencode('.jpg', camera.img, img_param)  # 按格式生成图片
                img_code = numpy.array(img_encode)  # 转换成数组
                camera.img_data = img_code.tobytes()  # 生成相应的字符串
                try:
                    client.send(struct.pack("ihh", len(camera.img_data), camera.resolution[0],
                                            camera.resolution[1]) + camera.img_data)  # 按照相应的格式进行打包发送图片
                except:
                    self.flag = False
                    video.release()  # 释放资源
                    return
                camera.server.close()                                 
if __name__ == '__main__':
    rospy.init_node("Server")
    while 1:
        server = socket.socket()
        server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)# 设置端口可复用
        # server.close()
        # server.bind((socket.gethostbyname(socket.gethostname()), 8888)) # 在Ubuntu有时会失效，与本机网络配置有关，推荐选择绑定静态端口
        server.bind((str(local_addr), 8888))
        server.listen(8888)
        print(socket.gethostbyname(local_addr) + ":8888  等待连接...")
        skt, addr = server.accept()
        print("连接来自：" + str(addr))
        # 等待客户端连接
        while 1:
            data = skt.recv(1024)
            message = data[:6].decode()
            # print(data.__len__())
            # print(data[6:])
            if message != "end!!!":
                if message != "___end":
                    if message == "camera":
                        thread1 = MyThread()
                        if not thread1.is_alive():# 防止重复开启线程导致程序崩溃
                            thread1.start()
                            print("视频连接成功！")
                        else:
                            _
                    elif message == "__cmd1":
                        com = command()
                        com.x,com.y,_,_,_,_ = struct.unpack("2i4f", data[6:])# 从缓冲区第6位开始解包
                        print("x:",com.x,"y:",com.y)
                        
                    elif message == "__cmd2":
                        com = command()
                        _,_,com.a,com.v,_,_ = struct.unpack("2i4f", data[6:])# 从缓冲区第6位开始解包
                        print("a:",round(com.a,2),"v:",round(com.v,2))# 保留2位小数
                    elif message == "__cmd3":
                        com = command()
                        _,_,_,_,com.r,com.w = struct.unpack("2i4f", data[6:])# 从缓冲区第6位开始解包
                        print("r:",round(com.r,2),"w:",round(com.w,2))# 保留2位小数
                    elif message == "___cmd":
                        com = command()
                        x1,x2,x3,y1,y2,y3,a1,a2,a3,b1,b2,b3,c1,c2,c3,w1,w2,w3 = struct.unpack("18f", data[6:])# 从缓冲区第6位开始解包
                        rospy.set_param("px1",x1)
                        rospy.set_param("px2",x2)
                        rospy.set_param("px3",x3)
                        rospy.set_param("py1",y1)
                        rospy.set_param("py2",y2)
                        rospy.set_param("py3",y3)
                        rospy.set_param("pa1",a1)
                        rospy.set_param("pa2",a2)
                        rospy.set_param("pa3",a3)
                        rospy.set_param("pb1",b1)
                        rospy.set_param("pb2",b2)
                        rospy.set_param("pb3",b3)
                        rospy.set_param("pc1",c1)
                        rospy.set_param("pc2",c2)
                        rospy.set_param("pc3",c3)
                        rospy.set_param("pw1",w1)
                        rospy.set_param("pw2",w2)
                        rospy.set_param("pw3",w3)
                    elif message == "stop!!":
                        print("stop!!")
                    else:
                        print("\nrecv:" + data[6:] + "\nsend：" + data[6:])# 从缓冲区第6位开始解包
                        skt.send(data[6:])
                        print("发送完毕")
                else:
                    print("对方断开连接")
                    server.shutdown(2)
                    # server.close()
                    server.close()
                    break
            else:
                    import os
                    os._exit(0)