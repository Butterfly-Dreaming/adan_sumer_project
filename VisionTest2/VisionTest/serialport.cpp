#include "serialport.h"
#include "cstring"
#include <iostream>
using namespace std;

SerialPort::SerialPort(QObject *parent) : QObject(parent)
{
    mode = 0;

    port = new QSerialPort();
    init_port();

    connect(port,&QSerialPort::readyRead,this,&SerialPort::handle_data);

}

SerialPort::~SerialPort()
{
    port->close();
    port->deleteLater();
    my_thread->quit();
    my_thread->wait();
    my_thread->deleteLater();
}

void SerialPort::init_port()
{
    port->setPortName("/dev/ttyACM0");                   //串口名 windows下写作COM1
    if(port -> isOpen())
    {
        port -> close();
    }
    if (port->open(QIODevice::ReadWrite))
    {
        qDebug() << "Port have been opened";
    }
    else
    {
        qDebug() << "open it failed";
    }
    port->setBaudRate(1000000);                           //波特率
    port->setDataBits(QSerialPort::Data8);             //数据位
    port->setStopBits(QSerialPort::OneStop);           //停止位
    port->setParity(QSerialPort::NoParity);            //奇偶校验
    port->setFlowControl(QSerialPort::NoFlowControl);  //流控制
    port->setReadBufferSize(40960);
}

void SerialPort::handle_data()
{
    //qDebug() << "handle_data";
    QByteArray data = port->readAll();
//    struct MODE{
//        char mode;
//        float pitch;//now_pitch
//        float yaw;//now_~
//    }Mode;-[=p
    memcpy(&fromSTM,data.data(),sizeof(FromSTM));

    if(fromSTM.mode == '0')
        mode = 0;
    if(fromSTM.mode == '1')
    {
        mode = 1;
    }

    if(mode != 0){
        write_data();
      }
}

void SerialPort::write_data()
{

    char data[12];

    if(shot_flag == true)
    {
        NUC2STM.fire = 2;
        memcpy(data,&NUC2STM,sizeof(NUC2STM));
        port -> write(data,sizeof(NUC2STM));
        NUC2STM.fire = 1;
        shot_flag = false;
        return ;
    }
    memcpy(data,&NUC2STM,sizeof(NUC2STM));
    port -> write(data,sizeof(NUC2STM));
}
