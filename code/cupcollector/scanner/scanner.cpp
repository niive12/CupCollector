/** @file */
#include "scanner.h"

scanner::scanner(unsigned int x0, unsigned int y0, int r){
    int size = 100;
    coordinate_array = new tekMap::posType[size];
    //coordinate_array[0] = tekMap::posType(10,10);
    std::cout << "Scanner - Hello World!"<<std::endl;
}

scanner::~scanner(){
    delete [] coordinate_array;
}
