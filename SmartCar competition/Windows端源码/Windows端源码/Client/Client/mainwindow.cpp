#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "config.h"
#include <iostream>
#include<sstream>
#include<cmath>
#include<cstdlib>
#include <synchapi.h>
#include <vector>

#define TEXT_COLOR_1(STRING)         "<b>""<font color=purple>" STRING "</font>"\
    "</b>" "<font color=black> </font>"
#define TEXT_COLOR_2(STRING)         "<b>""<font color=blue>" STRING "</font>""</b>" "<font color=black> </font>"
#define TEXT_COLOR_3(STRING)         "<b>""<font color=green>" STRING "</font>""</b>" "<font color=black> </font>"




QString IP;
qint16 Port;
bool end = false;
MyThread1 *thread1 = new MyThread1();

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("中央任务调度监控管理工具(客户端)");
    this->setFixedSize(WINDOW_WIDTH,WINDOW_HEIGHT);
    this->setWindowIcon(QIcon(WINDOWS_ICO_PATH));
    pic.load(BACK_PATH);
    Initialize();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Initialize()
{
    TCPSocket = new QTcpSocket();
    this->isconnection=false;
    ui->disconnect->setEnabled(false);
    //连接槽信号
    //成功连接服务器的connected()信号连接到slot_connected()
    connect(TCPSocket,SIGNAL(connected()),this,SLOT(slot_connected()));
    //有新数据到达时的readyread()信号连接到slot_recvmessage()
    connect(TCPSocket,SIGNAL(readyRead()),this,SLOT(slot_recvmessage()));
    //与服务器断开连接的disconnected()信号连接到slot_disconnect()
    connect(TCPSocket,SIGNAL(disconnected()),this,SLOT(slot_disconnect()));
    //更新坐标的线程信号连接到主界面update函数
    connect(thread1,&MyThread1::pos_update,this,&MainWindow::update);
    //消息栏回车信号连接到发送信息按钮槽函数
    connect(ui->lineEdit, SIGNAL(returnPressed()), this, SLOT(on_pushButton_send_clicked()));
    ui->EditIP->setText("192.168.0.166");
    ui->EditPort->setText("8888");
    ui->EditPath->setText("E:\\goal.txt");
    thread1->start();
    thread1->pos_connected=false;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.drawPixmap(0,0,width(),height(),pic);
}

//连接按键
void MainWindow::on_link_clicked()
{
    IP = ui->EditIP->text();
    Port = ui->EditPort->text().toInt();
    //终止之前的连接，重置套接字
    TCPSocket->abort();
    thread1->position->abort();
    //给定IP和端口号，连接服务器
    TCPSocket->connectToHost(QHostAddress(IP), Port);
    thread1->position->connectToHost(QHostAddress(IP), 7777);
    this->isconnection = (TCPSocket->waitForConnected())&&(thread1->position->waitForConnected());
    this->ui->link->isFlat();
}

//连接成功处理
void MainWindow::slot_connected()
{
    this->isconnection=true;
    thread1->pos_connected=true;
    ui->label_3->setText("已连接服务器端");
    ui->disconnect->setEnabled(true);
    ui->link->setEnabled(false);
    QMessageBox::information(this, "QT网络通信", "服务器端连接成功！");
}


//发送信息
void MainWindow::on_pushButton_send_clicked()
{
    if(this->isconnection)
    {
        QString sendMessage = ui->lineEdit->text(); //从单行文本框获得要发送消息
        if(!sendMessage.isEmpty())
        {
            //发送消息到服务器
            if(sendMessage=="change"||sendMessage=="exit"||sendMessage=="init")
                this->TCPSocket->write((QString("optcmd")+sendMessage).toUtf8());
            else
                this->TCPSocket->write((QString("mesage")+sendMessage).toUtf8());
//            this->TCPSocket->waitForBytesWritten();
            //本地显示发送的消息
            QString localDispalyMessage = QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss ") +tr("send to server: \n") + sendMessage;                                            ;
            ui->textBrowser->append(localDispalyMessage);
        }
        else
            QMessageBox::warning(this,"错误","消息不能为空!",QMessageBox::Ok);
    }
    else
        QMessageBox::warning(this,"错误","未连接到服务器!",QMessageBox::Ok);

    ui->lineEdit->clear();
}

//接收信息
void MainWindow::slot_recvmessage()
{

    //接收来自服务器的消息
    QByteArray byteArray = this->TCPSocket->readAll();
    QString recvMessage = QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss ") + tr("recv from server: \n") + byteArray ;
    ui->textBrowser->append(recvMessage);
    QMessageBox::information(this, "QT网络通信", "有新消息！");
}
MyThread::MyThread()
{
    stopped = false;
}

MyThread1::MyThread1()
{
    position = new QTcpSocket();

    stopped = false;
}

void MyThread::stop()
{
    stopped = true;
}

void MyThread1::stop()
{
    stopped = true;
}


//视频传输线程
void MyThread::run()
{
    std::string s = "camera.exe "+(IP).toStdString();      //可以调用python程序，也可以调用打包好的exe程序
    //std::string s = "python camera.py "+(IP).toStdString();
    const char* result = s.c_str();     //将字符串str变量变为char型数组
    int tail = 0;
    while (result[tail] != '\0') {
            tail++;
    }
    char* real_result = (char*)malloc(tail+1);
    for (int i = 0; i <= tail; i++) {
        real_result[i] = result[i];
    }
    WinExec(real_result, 0);
    //system(real_result);
}

//坐标更新线程
void MyThread1::run()
{
    while(1)
    {
        if(pos_connected)
        {
            //接收来自服务器的消息
            QString posStr = "";

            QString posx,posy;
            int point=0, lastpoint=0;
            while(posStr.size()<=16)
            {
                QByteArray posArray = position->readAll();
                posStr = posStr.append(QString(posArray));
            }
            for(int i=0;i<posStr.size();i++)
            {
                if(posStr[i]=='x')
                {
                    if(posStr[i+2]=='-')
                    {
                        posx = posStr.mid(i+2,7);
                        if(posStr[i+11]=='-')
                            posy = posStr.mid(i+11,7);
                        else
                            posy = posStr.mid(i+11,6);
                    }
                    else
                    {
                        posx = posStr.mid(i+2,6);
                        if(posStr[i+10]=='-')
                            posy = posStr.mid(i+10,7);
                        else
                            posy = posStr.mid(i+10,6);
                    }
                    for(int j=i;j<posStr.size();j++)
                    {
                        if(posStr[j]=='p')
                        {
                            point=QString(posStr[j+1]).toInt();
                            break;
                        }
                    }
                    if(point==lastpoint)
                        point = 0;
                    else
                        lastpoint=point;
                    emit pos_update(posx,posy,point);
                }
            }
            Sleep(300);
        }
        else
            Sleep(300);
    }
}

//坐标更新
void MainWindow::update(QString x,QString y, int p)
{
    ui->pos_x->setText(x);
    ui->pos_y->setText(y);
    if(p!=0)
        ui->textBrowser->append(QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss"));
    if(p==1)
        ui->textBrowser->append(TEXT_COLOR_1("Point "+QString::number(p)+" has reached!"));
    else if(p==2)
        ui->textBrowser->append(TEXT_COLOR_2("Point "+QString::number(p)+" has reached!"));
    else if (p==3)
        ui->textBrowser->append(TEXT_COLOR_3("Point "+QString::number(p)+" has reached!"));
}

//监控开启
void MainWindow::on_pushButton_clicked()
{
    if(this->isconnection)
    {
            QString conn="camera";
            this->TCPSocket->write(conn.toUtf8());
            thread.start();
            QString camlink = QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss ") + tr("连接到摄像头");
            ui->textBrowser->append(camlink);
    }
    else
    {
        QMessageBox::information(this, "QT网络通信", "请先连接客户端！");
    }
}

command t;
//加载目标点文件
void MainWindow::on_pushButton_7_clicked()
{
   QString path = ui->EditPath->text();//读取输入值
   std::ifstream in(path.toStdString());
   std::string line;
   float goals[20];
   int i=0;
   while (getline(in, line))
   {//获取文件的一行字符串到line中
       std::stringstream ss(line);//初始化 法1
       double x;
       while (ss >> x)
       {
           goals[i]=(float)x;
           QString s0 = "xyabcw";
           QString s1 = s0.mid(i%6, 1) + QString::number(i/6+1, 'i', 0) + ": " + QString::number(goals[i],'f',6);
           //ui->textBrowser->append(s1);
           i++;
       }
       t.x1 = goals[0];
       t.y1 = goals[1];
       t.a1 = goals[2];
       t.b1 = goals[3];
       t.c1 = goals[4];
       t.w1 = goals[5];
       t.x2 = goals[6];
       t.y2 = goals[7];
       t.a2 = goals[8];
       t.b2 = goals[9];
       t.c2 = goals[10];
       t.w2 = goals[11];
       t.x3 = goals[12];
       t.y3 = goals[13];
       t.a3 = goals[14];
       t.b3 = goals[15];
       t.c3 = goals[16];
       t.w3 = goals[17];
   }
   ui->goal1_x->setText(QString::number(t.x1,'f',6));
   ui->goal1_x->setReadOnly(true);
   ui->goal1_y->setText(QString::number(t.y1,'f',6));
   ui->goal1_y->setReadOnly(true);
   ui->goal2_x->setText(QString::number(t.x2,'f',6));
   ui->goal2_x->setReadOnly(true);
   ui->goal2_y->setText(QString::number(t.y2,'f',6));
   ui->goal2_y->setReadOnly(true);
   ui->goal3_x->setText(QString::number(t.x3,'f',6));
   ui->goal3_x->setReadOnly(true);
   ui->goal3_y->setText(QString::number(t.y3,'f',6));
   ui->goal3_y->setReadOnly(true);
   QByteArray dataArray = QByteArray::fromRawData((char*)&t, sizeof(t));//将结构体转为字节流格式
   this->TCPSocket->write(QString("___cmd").toUtf8());//发送数据头
   this->TCPSocket->write(dataArray);//发送字节流
//   QString comshow = QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss ") + tr("x ")+s1+tr("  y: ")+s2;
   QString comshow = QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss \n") + "("+QString::number(t.x1,'f',6)+","
           +QString::number(t.y1,'f',6)+ ") ("+QString::number(t.x2,'f',6)+","+QString::number(t.y2,'f',6) + ") ("
           +QString::number(t.x3,'f',6)+","+QString::number(t.y3,'f',6)+")";
   ui->textBrowser->append(comshow);
   ui->pushButton_8->setFocus();
   ui->pushButton_8->setDefault(1);
}


//目标点1
void MainWindow::on_pushButton_8_clicked()
{
    ui->pushButton_8->setDefault(0);
    this->TCPSocket->write(QString("__cmd1").toUtf8());//发送数据头
    QString comshow = QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss \n") + "("
            +QString::number(t.x1,'f',6)+","
            +QString::number(t.y1,'f',6)+ ")";
    ui->textBrowser->append(comshow);
    ui->pushButton_9->setFocus();
    ui->pushButton_9->setDefault(1);
}

//目标点2
void MainWindow::on_pushButton_9_clicked()
{
    ui->pushButton_9->setDefault(0);
    this->TCPSocket->write(QString("__cmd2").toUtf8());//发送数据头
    QString comshow = QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss \n") + "("
            +QString::number(t.x2,'f',6)+","+QString::number(t.y2,'f',6) + ")";
    ui->textBrowser->append(comshow);
    ui->pushButton_10->setFocus();
    ui->pushButton_10->setDefault(1);
}

//目标点3
void MainWindow::on_pushButton_10_clicked()
{
    ui->pushButton_10->setDefault(0);
    this->TCPSocket->write(QString("__cmd3").toUtf8());//发送数据头
    QString comshow = QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss \n") + "("
            +QString::number(t.x3,'f',6)+","+QString::number(t.y3,'f',6)+")";
    ui->textBrowser->append(comshow);
    ui->pushButton_8->setFocus();
    ui->pushButton_8->setDefault(1);
}


//断开连接处理
void MainWindow::slot_disconnect()
{
    if(TCPSocket != nullptr&&this->isconnection)
    {
        this->isconnection=false;
        QString conend = QDateTime::currentDateTime().toString("yyyy-M-dd hh:mm:ss ") + tr("连接断开!");
        ui->textBrowser->append(conend);
        TCPSocket->disconnectFromHost();
        thread1->deleteLater();
        if(end)
            QMessageBox::information(this, "QT网络通信", "服务端已关闭！");
        else
            QMessageBox::information(this, "QT网络通信", "断开与服务端的连接！");
        ui->disconnect->setEnabled(false);
        ui->link->setEnabled(true);
        ui->label_3->setText("未连接服务器");
     //   TCPSocket->close(); //关闭客户端
     //  TCPSocket->deleteLater();//deletelater的原理是 QObject::deleteLater()并没有将对象立即销毁，而是向主消息循环发送了一个event，下一次主消息循环收到这个event之后才会销毁对象。
    }
}

//清屏按钮
void MainWindow::on_pushButton_2_clicked()
{
    ui->textBrowser->clear();
}

//断开连接按钮
void MainWindow::on_disconnect_clicked()
{
    this->TCPSocket->write(QString("___end").toUtf8());
    slot_disconnect();
//   TCPSocket = new QTcpSocket();
}

//退出按钮
void MainWindow::on_pushButton_3_clicked()
{
    if(this->isconnection)
    {
        this->TCPSocket->write(QString("end!!!").toUtf8());
        end = true;
    }
    QElapsedTimer t;
    t.start();
    while(t.elapsed()<500);
    this->close();
}
