#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>

//Smækre lækre c libraries.

#include <cstdlib>
#include <cmath>
#define  PI_X2             6.283185307179586476925286766559005768394338798750211641949889

#define swag yolo

#define WHEELSPAN 290 // Taken from R2WD arduino lib.


using namespace std;

int main()
{
    cout << "Progriaaaamm start" << endl;
    string filename = "encoders.csv";


    // Load file
    cout << "Loading file.." << endl;
    std::ifstream file(filename.c_str());
    std::stringstream buffer;
    if(file.is_open())
    {
        buffer << file.rdbuf();
        file.close();
        cout << "File loaded.." << endl;
    }
    else
    {
        cout << "Problem opening the file" << endl;
        return 0;
    }

    struct data_point {
        int timestamp;
        double wheel1, wheel2;
    };
    struct position {

        position()
        {
            position(0,0,0,0);
        }

        position(int a, double b, double c, double d)
        {
            timestamp = a;
            x = b;
            y = c;
            theta = d;
        }


        position(data_point data, position last_position = {0,0,0,0})
        {
            timestamp = data.timestamp;
            double wheel_avg = (data.wheel1 + data.wheel2)/2;
            double d_theta = (data.wheel2 - data.wheel1)/WHEELSPAN;
            theta = d_theta + last_position.theta;

            if (theta > PI_X2) { theta -= PI_X2; }
            else if (theta < (-PI_X2)) { theta += PI_X2; }

            x = wheel_avg*cos(last_position.theta + d_theta/2) + last_position.x;
            y = wheel_avg*sin(last_position.theta + d_theta/2) + last_position.y;
        }


        int timestamp;
        double x, y, theta;
    };

    position start;
    vector<data_point> data_points;
    string current_line;
    double wheel1, wheel2;
    data_point current_data_point;
    position p2;
    while(!buffer.eof())
    {
        getline(buffer, current_line);
        // The following is an ugly blob
        // - it converts a line in the file to timestamp (arduino)
        // and extracts wheel movements.
        current_line.erase(0, current_line.find(",")+1);
        current_data_point.timestamp = atoi( current_line.substr(0, current_line.find(",")).c_str() );
        current_line.erase(0, current_line.find(",")+1);
        current_data_point.wheel1 = atof( current_line.substr(0, current_line.find(",")).c_str() );
        current_line.erase(0, current_line.find(",")+1);
        current_data_point.wheel2 = atof ( current_line.substr(0, current_line.find(",")).c_str() );

        //cout << current_data_point.timestamp << " " << current_data_point.wheel1 << " " << current_data_point.wheel2 << endl;
        position p1(current_data_point, p2);
        p2 = p1;
        cout << p1.timestamp << " " << p1.x << " " << p1.y << " " << p1.theta << endl;

    }




    return 0;
}

