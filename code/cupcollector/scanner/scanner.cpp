/** @file */
#include "scanner.h"
#include <cmath>
#include <unordered_set>

scanner::scanner(unsigned int x0, unsigned int y0, int r){
    int size = 2*r+1*2*r+1; //Always big enough, because it is big enough for square with sides 2*r+1
    coordinate_array = new pos_t[size];

    std::cout << "Scanner - Hello World!"<<std::endl;
    unsigned int x,y;
    unsigned int x1,y1;
    for(int i = 0; i< 360; i++){//Smaller steps later???
        x = x0 + int(r * cos(i*3.14/180));
        y = y0 + int(r * sin(i*3.14/180));

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

size_t cupScanner::scan(pos_t center, const pixelshade_map &scanmap, unsigned int radius)
{
    size_t number_of_cups = 0;
    if(r!=radius) {
        circle.clear();
        const array<array<int,2>,8> neighbours =
        {{  {-1,0}, /* W */ {1,0},  /* E */
            {0,-1}, /* N */ {0,1},  /* S */
            {-1,-1},/* NW */{1,-1}, /* NE */
            {1,1},  /* SE */{-1,1}  /* SW */
         }};
        r=radius;
        int rsquared = r*r;
        queue<pos_t> q;
        q.emplace(0,0);
        set<pos_t> visited;
        while(!q.empty()) {
            pos_t cur = q.front();
            q.pop();
            circle.push_front(cur);
            for(auto n:neighbours)
                if( ((cur.cx()+n[0])*(cur.cx()+n[0])
                     +(cur.cy()+n[1])*(cur.cy()+n[1]))<=rsquared )
                    if((visited.emplace(cur.cx()+n[0], cur.cy()+n[1])).second)
                        q.emplace(cur.cx()+n[0], cur.cy()+n[1]);
        }
    }
    for(auto i:circle) {
        try {
            if( WSPACE_IS_CUP(scanmap.const_coordVal(center+i))) {
                ++number_of_cups;
            }
        } catch (const std::out_of_range& oor) {(void)oor;}
    }
    return number_of_cups;
}
