#ifndef SERIALPORT_H
#define SERIALPORT_H

#include "common.h"
#include <QObject>
#include <QSerialPort>
#include <QString>
#include <QByteArray>
#include <QObject>
#include <QDebug>
#include <QObject>
#include <QThread>
#include <cstring>

typedef  struct
{
    uint8_t mode;

}nuc_send_data;

class SerialPort : public QObject
{
    Q_OBJECT

public:
    explicit SerialPort(QObject *parent = NULL);
    ~SerialPort();
    void init_port();  //初始化串口
    void write_data();     //发送数据

public slots:
    void handle_data();  //处理接收到的数据

signals:
  //接收数据
  void receive_data(QByteArray tmp);


private:
    QThread *my_thread;
    QSerialPort *port;
    int mode;
};

#endif // SERIALPORT_H
