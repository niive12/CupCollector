void shared_ptrs_for_beginners()
{
	//anything represents any class (here, it's just int)
	typedef int anything;
	//How people did it before C++11:
	//Let's hope there aren't thrown any exceptions in anything's constructor!
	anything *theOldWay = new anything();
	*theOldWay = 5;
	delete theOldWay;
	// Uncomment to crash shit:
	// *theOldWay = 6;
	//The smart way (class name used only once):
	auto theSmartWay = make_shared<anything>(/* anything constructor args*/);
	*theSmartWay = 5;
	//The other smart way (class name used twice here):
	shared_ptr<anything> theOtherSmartWay(new anything());
	*theOtherSmartWay = 5;
	// If you have a function like this:
	// anything * foo() { return new anything(); };
	//
	// then theOtherSmartWay would be:
	// shared_ptr<anything> theOtherSmartWay(foo());
	//
	// Note that PPMLoader::load("filename") returns an Image pointer
	// allocated by new, so it can be used in the above manner.
	cout << "Come on guys!" << endl;
}
