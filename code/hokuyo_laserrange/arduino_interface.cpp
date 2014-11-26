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


void printEncCSV(vector <int> data_points, string filename)
{
    ofstream out_file (filename, std::ios::app);
    if (out_file.is_open())
    {
        //out_file.seekp(0,ios_base::end);
        for (int j : data_points)
        {
            out_file << j << ",";
        }
        out_file << endl;
        out_file.close();
    }
    else
        cout << "Errrooor saving file" << endl;
}

void startEnc(string filename, int COM_port,  atomic<bool> &clear_to_run)
{
    if (!clear_to_run) { return; }
    int timestamp = 0, scans = 0;

    int n,
            bdrate=9600;      // Should be 115200 - standard for the urg laser range.

    char mode[]={'8','N','1',0},
            str[2][512];
    unsigned char buf[4096];

    auto start = std::chrono::system_clock::now();

    if(RS232_OpenComport(COM_port, bdrate, mode))
    {
        cout << "Can not open COMport" << endl;
    }


    std::string curr;
    std::stringstream dataBuffer;

    int enc_left, enc_right;
    while(1)
    {
#ifdef _WIN32
        Sleep(50);
#else
        usleep(50000);  /* sleep for 1 Second */
#endif
        n = RS232_PollComport(COM_port, buf, 4095);
        buf[n] = 0;   /* always put a "null" at the end of a string! */


        if (n > 7)//  discarded_scans > 2)
        {
            //cout << "Received " << n << " bytes of data." << endl;
            dataBuffer << buf;


            while (!dataBuffer.eof())
            {
                if (getline(dataBuffer,curr))
                {
                    if ( curr == "UMB_START" )
                    {
                        cout << "UMB mark started!" << endl;
                        //Start laser range!
                    }
                    else if ( curr == "UMB_STOP")
                    {
                        cout << "UMB mark stopped!"  << endl;
                    }
                    else
                    {
                        cout << endl;
                        cout << curr << endl;
                        int comma_pos = curr.find(","); // Find next comma
                        timestamp = atoi( curr.substr(0,comma_pos).c_str() );
                        curr.erase(0,comma_pos+1);
                        cout << curr << endl;
                        comma_pos = curr.find(","); // Find next comma
                        enc_left = atoi( curr.substr(0,comma_pos).c_str() );
                        curr.erase(0,comma_pos+1);
                        cout << curr << endl;
                        enc_right = atoi( curr.c_str() );

                    }
                }
                else
                {
                    break;
                }
            }


            ++scans;


            //dataBuffer.str(std::string()); // Empty dataBuffer
            //dataBuffer.seekg(0, dataBuffer.beg); // Return iterator to beginning
            dataBuffer.clear(); // Clear stateflags */

            // Stuff to break the loop
            if (!clear_to_run)
            {
                break;
            }
        }
    }
    auto end = std::chrono::system_clock::now();
    auto diff = end - start;
    cout << "Got " << scans << " scans in " << chrono::duration<double, std::milli>(diff).count() << " ms" << endl;
}

