#ifndef CAMERA_H
#define CAMERA_H
#pragma once
#include <iostream>
#include "common.h"
#include "MvCameraControl.h"
using namespace std;
using namespace cv;

class Camera{
private:
    void* handle;
    int nRet;
    MVCC_INTVALUE stParam;
    Mat new_img;
    unsigned int nBufSize;
    unsigned char* pFrameBuf;

public:
    Camera();
    ~Camera();
    static void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    bool getImage(Mat &img);
    void start();
};

#endif

