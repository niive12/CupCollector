#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <unistd.h>


#include "arduino_interface.cpp"
#include "laserrange_functions.cpp"


using namespace std;

void startScanner()
{
    //startScanning( 1000, "output.csv", 2);
}

int main(int argc, char* argv[])
{
    atomic<bool> laserrange_clear_to_run(1), encoders_clear_to_run(1);
    int number_of_scans, COM;

    if (argc > 2)
    {
        number_of_scans = atoi( argv[1] );
        COM = atoi( argv[2] );


    }
    else
    {
        cout << "Not enough arguments, using standard values!" << endl;
        number_of_scans = 600;
        COM = 2;
    }


    //thread laser_range1(startScanning, number_of_scans, "test2.csv", COM, std::ref(laserrange_clear_to_run));
    startEnc("encoders.csv",3,std::ref(encoders_clear_to_run));
    Sleep(1000);
    laserrange_clear_to_run = 0;
    cout << "haps" << endl;

    //laser_range1.join();

}
