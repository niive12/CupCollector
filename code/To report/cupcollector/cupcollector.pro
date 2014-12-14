TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += main.cpp \
    libraries/Image.cpp \
    libraries/PPMLoader.cpp \
    scanner/scanner.cpp \
    doordetector/doordetector.cpp \
    tekmap/brushfire.cpp \
    tekmap/pixelshade.cpp \
    tekmap/wavefront.cpp \
    robot/robot.cpp \

HEADERS += \
    libraries/Image.hpp \
    libraries/macros.hpp \
    libraries/PPMLoader.hpp \
    assignment.h \
    scanner/scanner.h \
    tekmap/tekmap.hpp \
    doordetector/doordetector.h \
    tekmap/brushfire.h \
    tekmap/pixelshade.h \
    tekmap/wavefront.h \
    robot/robot.h \
