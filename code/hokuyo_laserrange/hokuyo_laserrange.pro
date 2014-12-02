TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    rs232.c \
    laserrange_functions.cpp \
    arduino_interface.cpp

QMAKE_CXXFLAGS += -std=c++0x -pthread

LIBS += -pthread

HEADERS += \
    rs232.h

