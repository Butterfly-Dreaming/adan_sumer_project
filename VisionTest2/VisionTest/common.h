#ifndef COMMON_H
#define COMMON_H
#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/time.h>
#include <thread>
#include <termio.h>

//宏定义
#define PI CV_PI

//文件
#define PREDICT_CFG "Predict.yaml"
//DEBUG
#define SHOW_LIGHT_CONTOURS
//#define SHOW_LIGHT_PUT_TEXT
//#define SHOW_BINARY_IMAGE
//#define DRAW_TARGET
//#define SHOW_ARMOR_PARAM
#define SHOW_DRAW_RECT

double now();


struct GimbalPose
{
   float  pitch;
   float  yaw;

   GimbalPose(float pitch = 0.0, float yaw = 0.0, float roll = 0.0)
   {
       this->pitch     = pitch;
       this->yaw       = yaw;
   }

};
typedef enum {
    INFANTRY = 0,
    OLD_HERO,
    NEW_HERO,
    OLD_SENTRY_BELOW,
    OLD_SENTRY_ABOVE,
    NEW_SENTRY_BELOW,
    NEW_SENTRY_ABOVE,
    PLANE
} CarType;
struct Frame
{
    cv::Mat img;
    size_t seq;         //count from 1
    double timeStamp;	//time in ms, from initialization to now
};

/*
 * @Brief:  A simple circular buffer designed only for buffering captured images.
 *          A new frame will cover the oldest one. Every frame has its own mutex
 *          and the mutex is locked when the frame is being updated or being achieved
 *          to ensure thread safety.
 */
class FrameBuffer
{
public:
    FrameBuffer(size_t size);

    ~FrameBuffer() = default;

    bool push(const Frame& frame);

    bool getLatest(Frame& frame);

private:

    std::vector<Frame> _frames;
    std::vector<std::timed_mutex> _mutexs;

    size_t _tailIdx;
    size_t _headIdx;

    double _lastGetTimeStamp;
};



class ImgProdCons :public QObject
{
    Q_OBJECT
public:
    ImgProdCons();
    void produce();//生产图片线程
    void consume();//处理图片线程
    bool flage=0;
    void init();
    void debugser();
    void send_thread();//发送线程

private:
    //controal _control;
    FrameBuffer _buffer;
    std::unique_ptr<Camera> _Camera;
    std::unique_ptr<Solve_Posetion>_Solve_Posetion;
    std::unique_ptr<ArmorDetection>_ArmorDetection;
    std::unique_ptr<controal> _Controal;


//public slots:
    //void onReadyRead();
};
class ourthread :public QThread
{
    Q_OBJECT
public:
    ourthread(ImgProdCons *p,int cmd);
protected:
    void run();
private:
    ImgProdCons *ipc;
    int cmd;

};
#endif // IMGPRODCONS_H
#endif // COMMON_H
