TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += main.cpp \
    libraries/Image.cpp \
    libraries/PPMLoader.cpp \
    tekmap/tekmap.cpp

HEADERS += \
    libraries/Image.hpp \
    libraries/macros.hpp \
    libraries/PPMLoader.hpp \
    assignment.h \
    tekmap/tekmap.h
