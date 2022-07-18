#include "common.h"
#include "imgprodcons.h"
#include "NUC_communication.h"

#include "QSerialPortInfo"
#include <QApplication>
#include <QDebug>
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include"serialport.h"
double now()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
}
FrameBuffer::FrameBuffer(size_t size):
        _frames(size),
        _mutexs(size),
        _tailIdx(0),
        _headIdx(0),
        _lastGetTimeStamp(0.0)
{

}

bool FrameBuffer::push(const Frame& frame)
{
    const size_t newHeadIdx = (_headIdx + 1) % _frames.size();

    //try for 2ms to lock
    unique_lock<timed_mutex> lock(_mutexs[newHeadIdx],chrono::milliseconds(2));
    if(!lock.owns_lock())
    {
        return false;
    }

    _frames[newHeadIdx] = frame;
    if(newHeadIdx == _tailIdx)
    {
        _tailIdx = (_tailIdx + 1) % _frames.size();
    }
    _headIdx = newHeadIdx;
    return true;
}

bool FrameBuffer::getLatest(Frame& frame)
{
    volatile const size_t headIdx = _headIdx;

    //try for 2ms to lock
    unique_lock<timed_mutex> lock(_mutexs[headIdx],chrono::milliseconds(2));
    if(!lock.owns_lock() ||
       _frames[headIdx].img.empty() ||
       _frames[headIdx].timeStamp == _lastGetTimeStamp)
    {
        return false;
    }

    frame = _frames[headIdx];
    _lastGetTimeStamp = _frames[headIdx].timeStamp;

    return true;
}
ImgProdCons::ImgProdCons():
        _buffer(6)
//_Controal(make_unique<controal>())
{}

void ImgProdCons::init()

{

    _Camera=make_unique<Camera>();
   /*
    * 初始化线程
    */
    _Camera->start();

}
void ImgProdCons::produce(){//如果相机API不变的话，那么这一个一样能用

    auto startTime = chrono::high_resolution_clock::now();

    auto t1 = chrono::high_resolution_clock::now();
    for(;;)
    {

        //if(flage) break;
        Mat newImg;
        uint8_t seq=0;
        double timeStamp = (static_cast<chrono::duration<double,std::milli>>(chrono::high_resolution_clock::now() - startTime)).count();


        if(!_Camera->getImage(newImg))
        {
            //       cout<<"adwdawadw";
            continue;
        }


        _buffer.push(Frame{newImg, seq, timeStamp});


        auto t2 = chrono::high_resolution_clock::now();
//        cout << "Capture period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
//        cout << endl;
        t1 = t2;
    }
}
void ImgProdCons::consume()
{

    QString LogInfo;
    LogInfo.sprintf("%p", QThread::currentThread());
    qDebug() << "OpenSerialPort " <<"threadID : "<<LogInfo;//多线程的使用
    _SerialPort = make_unique<SerialPort>();//那个线程使用串口，在哪个线程定义。
    Frame frame;

      while(1){
        // cout<<"hello word"<<endl;
        if(!_buffer.getLatest(frame)){
            // cout<<"hello word"<<endl;
            continue;
        }
        //frame为当前帧
        /*
         * 在这里写代码
         */
    }
}
void ImgProdCons::send_thread(){
    _Controal = make_unique<controal>();
//    _Controal->shout_Open();
    //  std::
    _Controal->Set_Pitch_And_Yaw(10,10);
}
void ImgProdCons::debugser(){
    _Controal->shout_Open();

/***
    static QSerialPortInfo portInfo;

    auto a = portInfo.availablePorts();
    foreach (auto i, a) {
        cout << i.portName().toStdString() << endl;
    }

    port.setPort(a[0]);
    port.setBaudRate(1000000);
    port.setFlowControl(QSerialPort::NoFlowControl);
    port.setDataBits(QSerialPort::Data8);
    port.setStopBits(QSerialPort::OneStop);

    if(port.open(QIODevice::ReadWrite))
    {
        qDebug() << "open";
        connect(&port,SIGNAL(readyRead()),this,SLOT(onReadyRead()));
    }
    else
    {
        cout << "error";
        cout << port.error();
    }
        //500 -----2000
    toSTM32.shot = toSTM32.set_yaw = toSTM32.set_pitch =toSTM32.get = 0;

    toSTM32.shot = 1;
    char data[64] = {};

    memcpy(data,&toSTM32,sizeof(NUC_data_t));


    if(port.write(data,sizeof (NUC_data_t))==-1){
         qDebug()<<"WriTEfail"<<endl;
         getchar();
    }
    ***/
    // _Controal.shout_Open();

}
ourthread::ourthread(ImgProdCons *p,int c){
    ipc = p;
    cmd = c;
}
void ourthread ::run(){
    if(cmd==1){
        ipc->produce();
    }
    else if(cmd ==2){
        ipc->consume();
    }
    if(cmd ==3){
        ipc->send_thread();
    }

}