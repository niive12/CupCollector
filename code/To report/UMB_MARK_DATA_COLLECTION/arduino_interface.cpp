#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>


#include <cstring>
#include <cstdlib>
#include <cstdio>


#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"

using namespace std;


void printEncCSV(vector <double> data_points, string filename)
{
    ofstream out_file (filename, std::ios::app);
    if (out_file.is_open())
    {
        //out_file.seekp(0,ios_base::end);
        for (double j : data_points)
        {
            out_file << j << ",";
        }
        out_file << endl;
        out_file.close();
    }
    else
        cout << "Errrooor saving file" << endl;
}

void startEnc(string filename, int COM_port,  atomic<bool> &clear_to_run,  atomic<bool> &laserrange_ctrl, std::chrono::system_clock::time_point system_start)
{
    if (!clear_to_run) { return; }
    int timestamp = 0, scans = 0;

    int n,
            bdrate=9600;      // Should be 115200 - standard for the urg laser range.

    char mode[]={'8','N','1',0},
            str[2][512];
    unsigned char buf[4096];
    std::string curr;
    std::stringstream dataBuffer;
    std::vector<double> data_points;
    auto start = std::chrono::system_clock::now();


    if(RS232_OpenComport(COM_port, bdrate, mode))
    {
        cout << "Can not open COMport for arduino" << endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        n = RS232_PollComport(COM_port, buf, 4095);
        buf[n] = 0;   /* always put a "null" at the end of a string! */

        dataBuffer.write( reinterpret_cast<char *>(buf), n);

        int prev = dataBuffer.tellg();
        if (getline(dataBuffer,curr) && !dataBuffer.eof())
        {
            if ( curr == "UMB_START" )
            {
                cout << "UMB mark started!" << endl;
                laserrange_ctrl = true;
                start = std::chrono::system_clock::now();

            }
            else if ( curr == "UMB_STOP")
            {
                cout << "UMB mark stopped!"  << endl;
                laserrange_ctrl = false;
                clear_to_run = false;
            }
            else
            {
                auto system_timestamp = std::chrono::system_clock::now();
                int system_timestamp_int = chrono::duration<double, std::milli>(system_timestamp - system_start).count();

                data_points.push_back(system_timestamp_int);


                int comma_pos = curr.find(","); // Find next comma
                data_points.push_back(atoi( curr.substr(0,comma_pos).c_str() )); // Extract timestamp
                curr.erase(0,comma_pos+1);                                       // Delete

                comma_pos = curr.find(",");                                      // Find next comma
                data_points.push_back(atof( curr.substr(0,comma_pos).c_str() )); // Extract motor enc1
                curr.erase(0,comma_pos+1);

                data_points.push_back( atof( curr.c_str() ));                    // Extract motor enc2

                printEncCSV(data_points, filename);                              // Add new measurements to csv file
                data_points.clear();
                ++scans;
            }
        }
        else
        {
            dataBuffer.seekg(prev);
        }


        //dataBuffer.str(std::string()); // Empty dataBuffer
        //dataBuffer.seekg(0, dataBuffer.beg); // Return iterator to beginning
        dataBuffer.clear(); // Clear stateflags */


        // Stuff to break the loop
        if (!clear_to_run)
        {
            break;
        }
    }

    auto end = std::chrono::system_clock::now();
    auto diff = end - start;

    cout << "Got " << scans << " encoder readings in " << chrono::duration<double, std::milli>(diff).count() << " ms" << endl;
}

