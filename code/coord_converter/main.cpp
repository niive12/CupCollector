#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>



//Smækre lækre c libraries.

#include <cstdlib>
#include <cmath>

#define MIC_PI           3.1415

using namespace std;




int main()
{
    /* Coordinate converter 2000!
     * This program can convert coordinates from (x1,y1),(x2,y2) format
     * to the robot state.
     * This is done using the slope of the line connecting the wheels.
     * ...
     * Michael K. S. 5/12 2014
     * ...
     * Please supply input and output csv's.
     */

    string input_filename = "raw_data.csv";
    string output_filename = "output_data.csv";
    ifstream input_file(input_filename.c_str());
    ofstream output_file(output_filename.c_str());
    stringstream buffer;
    if(input_file.is_open())
    {
        buffer << input_file.rdbuf();
        input_file.close();
        cout << "Input file: \"" << input_filename << "\" loaded." << endl;
    }
    else
    {
        cout << "Problem opening the file" << endl;
        return 0;
    }
    string current_line;
    double x1,y1,x2,y2,count=0;
    double slope=0,angle;
    while(!buffer.eof())
    {
        getline(buffer,current_line);
        x1 = atoi (current_line.substr(0,current_line.find(",")).c_str());
        current_line.erase(0,current_line.find(",")+1);
        y1 = atoi (current_line.substr(0,current_line.find(",")).c_str());
        current_line.erase(0,current_line.find(",")+1);
        x2 = atoi (current_line.substr(0,current_line.find(",")).c_str());
        current_line.erase(0,current_line.find(",")+1);
        y2 = atoi (current_line.c_str());

        cout << x1 << " " << y1 << " " << x2 << " " << y2 << " " << ++count << endl;

        if (x2-x1 != 0) // If possible, update slope
        {
            slope = (y2-y1)/(x2-x1);
            angle = atan(slope) + 0.5*MIC_PI;

            if (x1 > x2)
                angle += MIC_PI;

        }





        if (output_file.is_open())
        {
            output_file << (x1 + x2)/2 << "," << (y1 + y2)/2 << "," << angle << endl;
        }
    }


    cout << "CoordinateConverter2000 started!" << endl;


    return 0;
}

