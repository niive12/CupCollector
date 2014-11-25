#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <chrono>

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



void printCSV(vector <int> data_points, string filename)
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


void extractPoints(string data, int timestamp, string filename)
{
    //string test = dataBuffer1.str();
    stringstream dataBuffer;
    static int pointset_number = 0;
    vector<int> current_points;
    dataBuffer.str(data);
    char current_char;


    //cout << "Length of databuffer: " << data.length() << endl;
    if(data.length() >=  1362)
    {
        //dataBuffer.get(current_char);
        current_points.clear();
        current_points.push_back(timestamp);
        while (!dataBuffer.eof())
        {
            int sum = 0;
            for (int i = 0; i < 2; i++)
            {
                dataBuffer.get(current_char);
                if (i == 0)
                    sum += ( current_char - 48)*64;
                else if (i == 1)
                    sum += ( current_char - 48);
            }
            current_points.push_back(sum);
        }

        printCSV(current_points,filename);
    }
    else
    {
        cout << "Databuffer doesn't contain enough data!";
    }
}
int decodeTimestamp(string timestamp)
{
    //Timestamp from hokuyo URG04-LX is encoded using
    //4-char encoding (p. 9 in the datasheet).
    stringstream dataBuffer;
    dataBuffer.str(timestamp);
    char current_char;
    int sum = 0;

    dataBuffer.get(current_char);
    sum += ( current_char - 48) * (64*64*64);
    dataBuffer.get(current_char);
    sum += ( current_char - 48) * (64*64);
    dataBuffer.get(current_char);
    sum += ( current_char - 48) * (64);
    dataBuffer.get(current_char);
    sum += ( current_char - 48);

    return sum;
}

void startScanning(int number_of_scans, string filename, int COM_port)
{
    int timestamp = 0;

    int n,
            cport_nr=2,         // Note: This is linux format - cport2 is COM3 on windows.
            bdrate=115200;      // Should be 115200 - standard for the urg laser range.

    char mode[]={'8','N','1',0},
            str[2][512];
    unsigned char buf[4096];
/*
    data_points.reserve(800); //Allocating memory for vector beforehand
    for (vector<int> j : data_points)
    {
        for (int k : j)
        {
            out_file << k << ",";
        }
        out_file << endl;
    }

*/

    auto start = std::chrono::system_clock::now();

    if(RS232_OpenComport(COM_port, bdrate, mode))
    {
        cout << "Can not open COMport" << endl;
    }

    RS232_cputs(COM_port, "MS0044072500000\nhaps\n");

    std::string curr;
    std::stringstream dataBuffer, inBuffer;
    bool lastDataLine = false;


    //Sleep(50);
    int scans = 0;
    while(1)
    {
#ifdef _WIN32
        Sleep(50);
#else
        usleep(50000);  /* sleep for 1 Second */
#endif
        n = RS232_PollComport(COM_port, buf, 4095);
        buf[n] = 0;   /* always put a "null" at the end of a string! */


        if (n == 1435)//  discarded_scans > 2)
        {
            //cout << "Received " << n << " bytes of data." << endl;
            inBuffer << buf;


            while (!inBuffer.eof())
            {
                if(getline(inBuffer,curr))
                {
                    if (curr.size() == 65)
                    {
                        dataBuffer << curr.substr(0,64);
                        lastDataLine = true;
                    }
                    else
                    {
                        if (lastDataLine && 1)
                        {
                            dataBuffer << curr.substr(0,curr.size()-3);
                            lastDataLine = false;
                        }
                        else
                            //Add check for timestamp here
                            if (curr.length() == 5)
                            {
                                timestamp = decodeTimestamp(curr);
                                //cout << "Timestamp: " << timestamp << endl;
                            }
                    }
                }
                else
                {
                    //cout << "Buffer cleared!" << endl;
                    break;
                }
            }

            extractPoints(dataBuffer.str(), timestamp, filename);
            cout << "Number of scans: " << ++scans << endl;

            //Clear buffers
            inBuffer.str(std::string()); // Empty inBuffer
            inBuffer.seekg(0, inBuffer.beg); // Return iterator to beginning
            inBuffer.clear(); // Clear stateflags

            dataBuffer.str(std::string()); // Empty dataBuffer
            dataBuffer.seekg(0, dataBuffer.beg); // Return iterator to beginning
            dataBuffer.clear(); // Clear stateflags

            if (scans == number_of_scans)
            {
                RS232_cputs(COM_port, "QThaps\n");
                break;
            }
        }
    }
    auto end = std::chrono::system_clock::now();
    auto diff = end - start;
    cout << "Got " << scans << " scans in " << chrono::duration<double, std::milli>(diff).count() << " ms" << endl;
}

