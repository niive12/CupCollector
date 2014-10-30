/** @file */
#include "scanner.h"
#include <cmath>

scanner::scanner(unsigned int x0, unsigned int y0, int r){
    int size = 2*r+1*2*r+1; //Always big enough, because it is big enough for square with sides 2*r+1
    coordinate_array = new pos_t[size];

    std::cout << "Scanner - Hello World!"<<std::endl;
    unsigned int x,y;
    unsigned int x1,y1;
    for(int i = 0; i< 360; i++){//Smaller steps later???
        x = x0 + int(r * cos(i*M_PI/180));
        y = y0 + int(r * sin(i*M_PI/180));

        //Will put in lots of duplicates, and only the circumference and not all the coordinates in the cicle
        coordinate_array[i] = pos_t(x,y);
        //Should not be this way.....

        /*if(x!=x1 || y!=y1){
            cout<<"x: "<<x<<", y: "<<y<<endl;
        )
        x1=x;
        y1=y;
        */
    }
}

scanner::~scanner(){
    delete [] coordinate_array;
}
