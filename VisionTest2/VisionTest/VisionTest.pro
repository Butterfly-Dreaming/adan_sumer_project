TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


SOURCES += \
        common.cpp \
        detector.cpp \
        main.cpp \
        predict.cpp \
        solveangle.cpp

HEADERS += \
    armor.h \
    common.h \
    detector.h \
    predict.h \
    solveangle.h

INCLUDEPATH += \
    /usr/local/include/opencv4 \
    /usr/local/include/opencv4/opencv2

LIBS += /usr/local/lib/*.so

DISTFILES +=
