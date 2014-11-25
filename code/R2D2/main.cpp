#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>


using namespace std;

#define PI 3.14159265359

#define DEGREE_TO_RAD(DEGREES) ((DEGREES*PI/180))

#define ANGLE_START DEGREE_TO_RAD(-120) // total coverage 240*, hence 0* = in front of robot
#define ANGLE_STEP DEGREE_TO_RAD(0.35) // assung 681 coordinates

// defines fot split and merge
#define MAKELINE_TRESHHOLD_DISTANCE 60

#define MERGELINE_TRESHHOLD_DISTANCE 40
#define MERGELINE_TRESHHOLD_ANGLE DEGREE_TO_RAD(20) // X* = (180-X)* between two walls

#define WEIGHT_OF_LOWEST_N 0.5
#define WEIGHT_DIFFERENCE (1-WEIGHT_OF_LOWEST_N)

#define MIN_N_IN_LINE_POST 0
#define MIN_N_IN_LINE_PRE 0

// parameters for RanSaC
#define RANSAC_MIN_POINTS_ON_LINE 80
#define RANSAC_MAX_DEVIATION_FROM_LINE 30

#define RANSAC_MAX_ITERATIONS_WITHOUT_LINE 60


// runmodes
#define NORMAL 0
#define TEST 1

#define RUNMODE NORMAL


struct line
{
	long double rho; // distance
	long double theta; // angle
};

using point = line;


/**
 * @brief findFeatures findes the lines in the data set
 * @param dataFromSensor an array of distance to nearest object from the sensor
 * @param linesInData returns the data in (\rho,\theta)
 */
void findFeatures_SplitMerge(vector<int> * dataFromSensor, vector<line> * linesInData);


/**
 * @brief findFeatures findes the lines in the data set
 * @param dataFromSensor an array of distance to nearest object from the sensor
 * @param linesInData returns the data in (\rho,\theta)
 */
void findFeatures_RanSaC(vector<int> * dataFromSensor, vector<line> * linesInData);


/**
 * @brief findLine fits a line to the data, includes the two points
 * @param start the first element in the vector
 * @param end the last element in the vector
 * @param lineInData returns the formel for the line in the data
 */
void findLine(vector<point>::iterator first, vector<point>::iterator last, line * lineInData);

/**
 * @brief findGreatesDeviation finds the point with the greatest deviation from the line
 * @param first first element in vector
 * @param last last element to consider in vector
 * @param lineFit pointer to the line which was drawn
 * @param thisPoint a pointer to the element with the greatest deviation
 * @return the deviation itself
 */
double findGreatesDeviation(vector<point>::iterator first, vector<point>::iterator last, line * lineFit, vector<point>::iterator *thisPoint);

/**
 * @brief makeToPoints takes the list from the sensor and converts it into points (rho [dist], theta [angle])
 * @param dataFromSensor a refrence to the list from sensor
 * @param dataPoints a refrence to where to put the new list, prev list will be cleared on call
 */
void makeToPoints(vector<int> * dataFromSensor, vector<point> * dataPoints);


double linearWeighting(int element, int arraySize);

int main()
{
	vector<int> simulatedDataFromSensor = {506,574,512,513,512,513,515,451,510,510,510,510,575,525,538,1014,970,970,906,957,943,937,936,917,853,890,890,882,872,858,857,848,838,770,827,816,812,800,799,789,780,776,704,765,758,752,746,745,731,730,730,727,710,708,640,699,694,686,684,673,673,667,664,662,658,652,577,638,630,630,630,626,623,620,614,614,611,610,602,599,593,589,588,586,583,582,583,515,569,569,569,569,568,568,571,566,556,552,552,549,538,538,538,538,537,537,534,526,526,526,526,523,523,455,511,508,507,507,507,506,505,505,503,498,498,498,496,496,496,496,495,489,489,486,486,486,488,486,485,481,481,478,477,477,473,473,473,473,473,476,473,473,471,467,467,465,464,465,465,464,459,461,460,459,459,460,460,459,457,457,454,453,453,451,451,451,451,451,453,453,452,452,453,453,453,454,454,453,453,448,448,453,453,448,451,387,507,451,451,449,449,453,453,456,456,456,458,456,452,454,390,511,454,390,447,511,454,454,454,458,458,458,458,457,457,458,458,458,462,463,463,463,463,463,466,462,464,464,464,464,467,467,467,467,467,474,474,472,473,472,470,466,468,468,470,478,480,482,480,482,486,482,416,428,416,407,402,397,386,322,378,378,373,372,368,357,355,353,350,348,340,340,338,330,325,261,383,261,318,318,309,305,303,295,294,290,289,289,289,289,289,295,295,295,303,306,307,313,314,314,314,314,378,322,323,323,323,322,321,321,321,257,316,312,311,311,311,303,303,303,303,303,300,291,288,288,288,284,284,283,283,280,280,277,275,275,275,269,269,269,266,265,265,265,265,265,266,266,266,272,272,262,196,255,254,254,254,254,254,248,248,248,248,247,247,246,244,246,242,235,240,238,235,235,235,235,235,234,233,233,233,233,233,233,233,235,233,233,235,236,236,236,230,230,229,229,230,230,233,236,236,235,235,236,235,232,233,232,231,230,229,227,227,229,229,227,226,226,226,225,225,225,225,225,224,225,224,222,224,222,219,215,219,219,222,223,223,223,222,223,217,217,212,212,209,209,213,213,213,213,215,215,219,220,222,222,222,222,222,221,221,219,219,219,221,221,221,222,222,222,222,222,222,225,225,225,224,226,226,224,226,230,226,224,225,225,225,230,235,235,234,234,234,228,228,228,228,232,232,235,235,235,235,236,236,236,242,242,236,236,242,242,234,234,243,241,241,243,243,253,318,256,257,257,192,252,251,251,251,251,252,255,255,255,319,262,262,262,262,262,263,262,263,265,268,272,273,273,277,277,277,277,278,281,283,283,286,286,287,292,292,292,291,291,291,292,296,302,306,306,312,312,313,317,317,382,321,322,321,322,326,326,336,336,336,336,339,334,334,333,265,159,181,72,126,190,136,154,247,294,335,321,350,438,393,397,398,404,407,409,416,423,423,425,439,440,505,452,455,455,458,458,476,478,487,491,498,510,575,513,522,530,541,547,554,558,630,582,583,587,590,612,613,616,688,642,660,668,673,685,764,705,705,577,601,587,514,561,448,510,508,506,503,502,502,502,495,495,494,494};
	vector<line> testOutputLines;

	ofstream lineFile("2D_linescanner_test/lineFile.csv", ios::trunc);

	findFeatures_RanSaC (&simulatedDataFromSensor, &testOutputLines);

	for(int i = 0; i < testOutputLines.size (); i++)
	{
		cout << (testOutputLines[i]).theta *180 / PI << ", " << (testOutputLines[i]).rho << " [theta,rho]" << endl;
		lineFile << (testOutputLines[i]).theta << "," <<(testOutputLines[i]).rho << "\n";
	}

	lineFile.close ();

	return false;
}


void findLine(vector<point>::iterator start, vector<point>::iterator end, line * lineInData)
{
	int sizeOfVector = distance(start,end);
	line bestFitLine;

	if(sizeOfVector > 1)
	{
		// the equation is divided into four part spliting at the sums, uniformly weigted datapoints assumed
		// I, II, III and IV are for the angle and X for the distance
		long double I = 0, II = 0, III = 0, IV = 0, X = 0, rhoSin = 0, rhoCos = 0;

		for(int i = 0; i < sizeOfVector; i++)
		{
			// calc rho*cos(theta), rho*sin(theta), first and third part
			rhoCos += ((start + i)->rho) * cos((start + i)->theta);
			rhoSin += ((start + i)->rho) * sin((start + i)->theta);
			I += (pow((start + i)->rho,2) * sin(2*((start + i)->theta)));
			III += (pow((start + i)->rho,2) * cos(2*((start + i)->theta)));
		}

		// calc second and fourth part
		II = 2 * rhoCos * rhoSin / sizeOfVector;
		IV = (pow(rhoCos, 2) - pow(rhoSin, 2)) / sizeOfVector;

		// calc total angle
		bestFitLine.theta = (0.5)*atan((I-II)/(III-IV)) - PI/2; // he removed the pi/2 because of identity but we need it to get correct angle

		// calc summation of r
		for(int i = 0; i < sizeOfVector; i++)
		{
			X += ((start + i)->rho)*(cos(((start + i)->theta) - bestFitLine.theta));
		}


		// calc total distance
		bestFitLine.rho = (X/sizeOfVector);

		// output
		*lineInData = bestFitLine;
	}
	else
	{
		cout << " ERROR: Set to make line of is " << sizeOfVector << "." << endl;
	}
}

void findFeatures_RanSaC(vector<int> * dataFromSensor, vector<line> * linesInData)
{
	linesInData->clear ();

	vector<point> dataSet;
	vector<point> dataInliers;
	line newLine;
	int testTakenWithoutResult = 0;
	int sizeOfArray;
	int randomPointA, randomPointB;

	makeToPoints (dataFromSensor,&dataSet);

	// generate two random points and draw line, if number of points on line great enough, save it and remove all points on line

	do
	{
		// count int up to see how many unsecesful trials were taken
		testTakenWithoutResult++;
		// clear inliers
		dataInliers.clear ();

		sizeOfArray = dataSet.size ();
		// find first point
		randomPointA = rand()%sizeOfArray;
		// find a point till it is in the array
		do
		{
			// generate B such that A != B
			randomPointB = rand()%sizeOfArray;
		}while(randomPointB == randomPointA);

		// fill dataset with the two points
		dataInliers.emplace_back(dataSet[randomPointA]);
		dataInliers.emplace_back(dataSet[randomPointB]);

		// find the line matching these two
		findLine (dataInliers.begin (),dataInliers.end (),&newLine);

		// find all inliears for these (setA)
		for(int i = 0; i < sizeOfArray; i++)
		{
			double di = abs(((((dataSet[i]).rho)*cos(((dataSet[i]).theta)-(newLine.theta))) - newLine.rho));
			if(di < RANSAC_MAX_DEVIATION_FROM_LINE && i != randomPointA && i != randomPointB)
			{
				// add point to list
				dataInliers.emplace_back(dataSet[i]);
			}
		}

		// if enough inliers, recompute line (line2), find inliers for this an remove simulaniously from dataSet, then recompute line (line final)
		// else do nothing
		if(dataInliers.size () > RANSAC_MIN_POINTS_ON_LINE)
		{
			int firstInliers = dataInliers.size ();
			testTakenWithoutResult = 0;
			// recompute line
			findLine (dataInliers.begin (),dataInliers.end (),&newLine);
			dataInliers.clear ();
			// find inliers and remove them
			for(int i = 0; i < sizeOfArray; i++)
			{
				double di = abs(((((dataSet[i]).rho)*cos(((dataSet[i]).theta)-(newLine.theta))) - newLine.rho));
				if(di < RANSAC_MAX_DEVIATION_FROM_LINE)
				{
					// add point to list
					dataInliers.emplace_back(dataSet[i]);
					dataSet.erase (dataSet.begin () + i);
					i--;
					sizeOfArray--;
				}
			}
			// if more inliers than before, add, else remove
			int newInliers = dataInliers.size ();
			if(newInliers > firstInliers)
			{
				// recompute final line
				findLine (dataInliers.begin (),dataInliers.end (),&newLine);
				// save line
				linesInData->emplace_back(newLine);
			}
			else
			{
				// put points back
				for (int i = 0; i < newInliers; i++)
				{
					dataSet.emplace_back(dataInliers[i]);
				}
			}
		}

	}while(testTakenWithoutResult < RANSAC_MAX_ITERATIONS_WITHOUT_LINE);

}


void findFeatures_SplitMerge(vector<int> * dataFromSensor, vector<line> * linesInData)
{
	linesInData->clear ();

	// make vector of vectors to divide data into and keep a second to store the line for the corresponding dataset (linesInData = already created)

	vector< vector<point> > dataSetDevisions;
	line newLine;
	vector<point>::iterator pointFurthestAway;
	long double deviationOfPoint;
	bool changeHappened;

	// set all points in array
	dataSetDevisions.emplace_back(vector<point>());
	makeToPoints (dataFromSensor,&(dataSetDevisions[0]));
	// find the first line
	findLine ((dataSetDevisions[0]).begin (), (dataSetDevisions[0]).end () , &newLine);
	// put line in array
	linesInData->emplace_back(newLine);

#if RUNMODE == TEST
	cout << " Test first line found in data: (" << newLine.rho << ", " << (newLine.theta * 180 / PI) << ") [theta = degrees]" << endl;
#endif
	// devide dataset
	do
	{
		changeHappened = false;
		int sizeOfVector = dataSetDevisions.size ();

		for(int i = 0; i < sizeOfVector; i++)
		{
			// test deviation if line is big enough to be divided (pre condition)
			if((dataSetDevisions[i]).size() > 2*MIN_N_IN_LINE_PRE)
			{
				deviationOfPoint = findGreatesDeviation ((dataSetDevisions[i]).begin () + MIN_N_IN_LINE_PRE, ((dataSetDevisions[i]).end ()) - MIN_N_IN_LINE_PRE , &(linesInData->at (i)), &pointFurthestAway);

				// if deviation too great, split
				if(deviationOfPoint >= MAKELINE_TRESHHOLD_DISTANCE)
				{
					// own expansion, if max distance point is at the edge of the dataset, remove it.
					if(((dataSetDevisions[i]).begin ()) == pointFurthestAway || (((dataSetDevisions[i]).end())-1) == pointFurthestAway)
					{
						// repeat loop once more when done
						changeHappened = true;
						(dataSetDevisions[i]).erase(pointFurthestAway);
					}
					else if( (distance((dataSetDevisions[i]).begin (),pointFurthestAway) < MIN_N_IN_LINE_POST) || (distance(pointFurthestAway,(dataSetDevisions[i]).end()) < MIN_N_IN_LINE_POST))
					{
						// don't repeat loop if point devides arrays into too small sizes = ignore (post condition)
					}
					else
					{
						// repeat loop once more when done
						changeHappened = true;
						// add space for vector
						dataSetDevisions.emplace_back(vector<point>());

						// fill from division point (including)
						for(int j = 0; j < distance(pointFurthestAway, (dataSetDevisions[i]).end ()--); j++)
						{
							// fill last vector (prev made) in vector
							(dataSetDevisions[(dataSetDevisions.size ()-1)]).emplace_back(*(pointFurthestAway + j));
						}
						// remove the end of the list (excluding division point)
						advance(pointFurthestAway,1);
						(dataSetDevisions[i]).erase((pointFurthestAway), (dataSetDevisions[i]).end ());
						// find line for vector which was split
						findLine ((dataSetDevisions[i]).begin (), (dataSetDevisions[i]).end () , &newLine);
						// input line to vector
						(*linesInData)[i] = newLine;
						// find line for vector at end of vector (new one)
						findLine ((dataSetDevisions[(dataSetDevisions.size ()-1)]).begin (), (dataSetDevisions[(dataSetDevisions.size ()-1)]).end () , &newLine);
						// input line to vector
						(*linesInData).emplace_back(newLine);
					}
				}
			}
		}
	}while (changeHappened);

	// merge lines
	do
	{
		changeHappened = false;
		int numberOfLines = linesInData->size ();
		// iterate through the whole vector and compare all (start over if two were combined)
		for(int i = 0; i < numberOfLines; i++)
		{
			for(int j = (i + 1); j < numberOfLines; j++)
			{
				// get deviation of the two lines
				double deviationOfDistance = abs((*linesInData)[i].rho - (*linesInData)[j].rho);
				double deviationOfAngle = abs((*linesInData)[i].theta - (*linesInData)[j].theta);
				// if appropiate to merge, do so
				if(deviationOfAngle < MERGELINE_TRESHHOLD_ANGLE && deviationOfDistance < MERGELINE_TRESHHOLD_DISTANCE)
				{
					changeHappened = true;
					// merge datasets
					for(int k = 0; k < dataSetDevisions[j].size(); k++)
					{
						dataSetDevisions[i].emplace_back(dataSetDevisions[j][k]);
					}
					// find new line (update the one)
					findLine ((dataSetDevisions[i]).begin (), (dataSetDevisions[i]).end () , &newLine);
					(*linesInData)[i] = newLine;
					// remove old dataset and line
					dataSetDevisions.erase (dataSetDevisions.begin () + j);
					(*linesInData).erase ((*linesInData).begin () + j);
					numberOfLines--;
				}
			}
		}
	}while(changeHappened);

}

double linearWeighting(int element, int arraySize)
{
	double result;

	int distance = abs(arraySize - element*2);

	result = 1 - (distance * WEIGHT_DIFFERENCE /arraySize);

	return result;
}


double findGreatesDeviation(vector<point>::iterator first, vector<point>::iterator last, line * lineFit, vector<point>::iterator * thisPoint)
{
	int sizeOfDataset = distance(first,last);
	long double deviation = 0;
	long double di; // deviation for current point (d_i)

	for(int i = 0; i < sizeOfDataset; i++)
	{
		di = abs(linearWeighting(i,sizeOfDataset)*((((first + i)->rho)*cos(((first + i)->theta)-(lineFit->theta))) - lineFit->rho));
		if(di >= deviation)
		{
			deviation = di;
			*thisPoint = (first + i);
		}
	}
	return deviation;
}

void makeToPoints(vector<int> * dataFromSensor, vector<point> * dataPoints)
{
	dataPoints->clear ();
	int sizeOfDataset = dataFromSensor->size ();
	point newPoint;
	for(int i = 0; i < sizeOfDataset; i++)
	{
		newPoint.rho = dataFromSensor->at (i);
		newPoint.theta = ((i*ANGLE_STEP) + ANGLE_START);

		dataPoints->emplace_back(newPoint);
#if RUNMODE == TEST
		// test output
		cout << " Point added to start list: (" << newPoint.rho << ", " << (newPoint.theta * (180 / PI)) << ") [theta = degrees]" << endl;
#endif
	}

}
