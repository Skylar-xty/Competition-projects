#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QObject>
#include <QMainWindow>
#include <QTcpSocket> //通信套接字
#include <QFile>      //文件操作
#include <QHostAddress> //IP地址
#include <QMessageBox> //提示对话框
#include <QPainter>
#include <QDateTime>
#include <QElapsedTimer>//延时
#include <QtDebug>
#include <QThread>
#include <windows.h>
#include <fstream>
#include <QString>

class MyThread : public QThread
{
    Q_OBJECT

public:
    MyThread();
    void stop();
protected:
    void run();

private:
    volatile bool stopped;
};

class MyThread1 : public QThread
{
    Q_OBJECT

public:
    MyThread1();
    void stop();
    QTcpSocket* position;
    bool pos_connected;
signals:
    void pos_update(QString x,QString y, int p);
protected:
    void run();
private:
    volatile bool stopped;
};

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void Initialize();
    void paintEvent(QPaintEvent *event);


    //数据
    QTcpSocket* TCPSocket;

    QPixmap pic;

    bool isconnection;
public slots:
    void update(QString x,QString y, int p);

private slots:

    void slot_connected(); //处理成功连接到服务器的槽

    void slot_recvmessage(); //接收来自服务器的消息的槽

    void slot_disconnect();//处理断开连接时的槽

    void on_pushButton_send_clicked();//发送消息按钮

    void on_pushButton_2_clicked();//清屏按钮

    void on_link_clicked();//连接按钮

    void on_disconnect_clicked();//取消连接按钮

    void on_pushButton_3_clicked();//退出按钮

    void on_pushButton_clicked();//监控开启

    void on_pushButton_7_clicked();//按目标点模式发送信息

    void on_pushButton_8_clicked();//停车


    void on_pushButton_9_clicked();

    void on_pushButton_10_clicked();

private:
    Ui::MainWindow *ui;
    MyThread thread;//视频传输线程
};
struct command
{
    float x1,x2,x3,y1,y2,y3,a1,a2,a3,b1,b2,b3,c1,c2,c3,w1,w2,w3;
    command()
    {
        this->x1 = 0.0;
        this->x2 = 0.0;
        this->x3 = 0.0;
        this->y1 = 0.0;
        this->y2 = 0.0;
        this->y3 = 0.0;
        this->a1 = 0.0;
        this->a2 = 0.0;
        this->a3 = 0.0;
        this->b1 = 0.0;
        this->b2 = 0.0;
        this->b3 = 0.0;
        this->c1 = 0.0;
        this->c2 = 0.0;
        this->c3 = 0.0;
        this->w1 = 0.0;
        this->w2 = 0.0;
        this->w3 = 0.0;
    }
};

#endif // MAINWINDOW_H
