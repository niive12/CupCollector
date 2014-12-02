#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <unistd.h>


#include "arduino_interface.cpp"
#include "laserrange_functions.cpp"


using namespace std;

void spamThread()
{
	while(1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		cout << "Hello from mr. spam-thread" << endl;
	}
}

int main(int argc, char* argv[])
{
	atomic<bool> laserrange_clear_to_run(0), encoders_clear_to_run(1);
	atomic<int> system_timestamp;
	int number_of_scans, COM;

	if (argc > 2)
	{
		number_of_scans = atoi( argv[1] );
		COM = atoi( argv[2] );


	}
	else
	{
		cout << "Not enough arguments, using standard values!" << endl;
		number_of_scans = 60000;
		COM = 24;
	}

	chrono::system_clock::time_point start = std::chrono::system_clock::now();
	thread laser_range1(startScanning, number_of_scans, "test2.csv", COM, std::ref(laserrange_clear_to_run), start);
	//thread test(spamThread);
	thread robot1 (startEnc, "encoders.csv",16,std::ref(encoders_clear_to_run),std::ref(laserrange_clear_to_run), start);
	sleep(1000);
	//laserrange_clear_to_run = 0;
	cout << "haps" << endl;
	laser_range1.join();
	robot1.join();

	//laser_range1.join();

}
