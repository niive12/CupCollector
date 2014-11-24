#include <string>
#include <iostream>

#include "laserrange_functions.cpp"

using namespace std;

int main(int argc, char* argv[])
{
    if (argc > 2)
    {
        int number_of_scans = atoi( argv[1] ), COM = atoi( argv[2] );
        startScanning(number_of_scans,"output.csv",COM);
    }
    else
    {
        cout << "Not enough arguments!" << endl;
    }
}
