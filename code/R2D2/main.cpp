#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <time.h>
#include <string>

using namespace std;

#define PI 3.14159265359

#define DEGREE_TO_RAD(DEGREES) ((DEGREES*PI/180))

#define ANGLE_START DEGREE_TO_RAD(-120) // total coverage 240*, hence 0* = in front of robot
#define ANGLE_STEP DEGREE_TO_RAD(0.35) // assung 681 coordinates

// defines fot split and merge
#define SM_SPLITLINE_TRESHHOLD_DISTANCE 60
#define SM_SPLITLINE_MIN_POINTS_IN_SPLITLINE 20

#define SM_MERGELINE_TRESHHOLD_DISTANCE 40
#define SM_MERGELINE_TRESHHOLD_ANGLE DEGREE_TO_RAD(40) // X* = (180-X)* between two walls

#define SM_MIN_POINTS_IN_LINE 30

// parameters for RanSaC
#define RANSAC_MIN_POINTS_ON_LINE 100
#define RANSAC_MAX_DEVIATION_FROM_LINE 15

#define RANSAC_MAX_ITERATIONS_WITHOUT_LINE 30

// parameters for RanSaC
#define RANSAC_TRAD_MIN_POINTS_ON_LINE 150
#define RANSAC_TRAD_MAX_DEVIATION_FROM_LINE 15

#define RANSAC_TRAD_MAX_ITERATIONS_WITHOUT_LINE 50


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
 * @brief findFeatures_RanSaC_Trad findes the lines in the data set on the traditional RanSaC way
 * @param dataFromSensor an array of distance to nearest object from the sensor
 * @param linesInData returns the data in (\rho,\theta)
 */
void findFeatures_RanSaC_Trad(vector<int> * dataFromSensor, vector<line> * linesInData);


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


void loadSensorData(string* file, vector< vector<int> > * sensorInputStorage);

int main()
{
	vector< vector<int> > simulatedDataFromSensor;
	vector< vector<line> > testOutputLines;

	string file = "2D_linescanner_test/data_all.csv";
	loadSensorData (&file, &simulatedDataFromSensor);

	cout << "data elements: " << simulatedDataFromSensor.size () << endl;

    srand ( time(NULL) );
	ofstream lineFile("2D_linescanner_test/lineFile.csv", ios::trunc);
	for (int i = 0; i < simulatedDataFromSensor.size (); i++)
	{
        testOutputLines.emplace_back( vector< line >() );
        findFeatures_RanSaC (&(simulatedDataFromSensor[i]), &(testOutputLines[i]));
		int linesFound = (testOutputLines[i]).size ();
		for(int j = 0; j < linesFound; j++)
		{
//			cout << (testOutputLines[i][j]).theta *180 / PI << ", " << (testOutputLines[i][j]).rho << " [theta,rho]" << endl;
			lineFile << (testOutputLines[i][j]).theta << "," <<(testOutputLines[i][j]).rho;
			if(j < (linesFound - 1))
			{
				lineFile << ",";
			}
		}
		lineFile << ",\n";
	}

	lineFile.close ();

	return false;
}

void loadSensorData(string* file, vector< vector<int> > * sensorInputStorage)
{
	char input_char;
	int row = 0;
	int element;
	int skipFirstElements = 2;
	ifstream input_stream(file->c_str());

	sensorInputStorage->clear ();
    sensorInputStorage->emplace_back( vector< int >() );

	// load the file into memory
	while (!input_stream.eof())
	{
		input_char = input_stream.get();
		if (input_char == '\n')
		{
			skipFirstElements = 2;
			// insert element
			(sensorInputStorage->at (row)).emplace_back(element);
			// reset element
			element = 0;
			// jump to next row
            sensorInputStorage->emplace_back( vector< int >() );
			row++;
		}
		else if (input_char == ',' || input_stream.eof())
		{
			if(skipFirstElements == 0)
			{
				// insert element
				if(element != 0)
				{
					(sensorInputStorage->at (row)).emplace_back(element);
				}
				else if((sensorInputStorage->at (row)).size() < 1)
				{
					sensorInputStorage->erase (sensorInputStorage->end ()-1);
				}
				// reset element
				element = 0;
			}
			else
			{
				skipFirstElements--;
			}
		}
		else if (input_char == ' ')
		{
			// do nothing with it!!!
		}
		else
		{
			// apend char to element if no seperator is reached
			element *= 10;
			element += (input_char - '0');
		}

	}
	if (element != 0)
	{
		(sensorInputStorage->at (row)).emplace_back(element);
	}
	if((sensorInputStorage->at (row)).size() < 1)
	{
		sensorInputStorage->erase (sensorInputStorage->end ()-1);
	}

	// close stream
	input_stream.close();
}



void findLine(vector<point>::iterator start, vector<point>::iterator end, line * lineInData)
{
	int sizeOfVector = distance(start,end);
	line bestFitLine;

	if(sizeOfVector > 1)
	{
		// the equation is divided into four part spliting at the sums, uniformly weigted datapoints assumed
		// I, II, III and IV are for the angle and rhoSin and rhoCos both angle and distance
		long double I = 0, II = 0, III = 0, IV = 0, rhoSin = 0, rhoCos = 0;

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
		bestFitLine.theta = (0.5)*atan((I-II)/(III-IV));
		// use pi/2 depending on the orientation of the line
		// pi man
		double diffX = abs(cos((end-1)->theta) * (end-1)->rho - cos(start->theta) * start->rho);
		double diffY = abs(sin((end-1)->theta) * (end-1)->rho - sin(start->theta) * start->rho);
		if (diffX > diffY)
		{
			bestFitLine.theta -= PI/2;
		}

		// calc r
		bestFitLine.rho = (rhoCos * cos(bestFitLine.theta) + rhoSin * sin(bestFitLine.theta)) / sizeOfVector;

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
        randomPointA = rand() % sizeOfArray;
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
				double di = abs((((dataSet[i]).rho)*cos(((dataSet[i]).theta)-(newLine.theta))) - newLine.rho);
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
			if(newInliers >= firstInliers)
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
	}while(testTakenWithoutResult < RANSAC_MAX_ITERATIONS_WITHOUT_LINE && sizeOfArray >= RANSAC_MIN_POINTS_ON_LINE);

}


void findFeatures_RanSaC_Trad(vector<int> * dataFromSensor, vector<line> * linesInData)
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
        randomPointA = rand() % sizeOfArray;
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
            if(di < RANSAC_TRAD_MAX_DEVIATION_FROM_LINE && i != randomPointA && i != randomPointB)
            {
                // add point to list
                dataInliers.emplace_back(dataSet[i]);
            }
        }

        // if enough inliers, remove all inliers from dataSet, then recompute line (line final)
        // else do nothing
        if(dataInliers.size () > RANSAC_TRAD_MIN_POINTS_ON_LINE)
        {
            testTakenWithoutResult = 0;
            // find inliers and remove them
            for(int i = 0; i < sizeOfArray; i++)
            {
                double di = abs((((dataSet[i]).rho)*cos(((dataSet[i]).theta)-(newLine.theta))) - newLine.rho);
                if(di < RANSAC_TRAD_MAX_DEVIATION_FROM_LINE)
                {
                    // add point to list
                    dataSet.erase (dataSet.begin () + i);
                    i--;
                    sizeOfArray--;
                }
            }
            // recompute line
            findLine (dataInliers.begin (),dataInliers.end (),&newLine);
            dataInliers.clear ();
            // save line
            linesInData->emplace_back(newLine);
        }
    }while(testTakenWithoutResult < RANSAC_TRAD_MAX_ITERATIONS_WITHOUT_LINE && sizeOfArray >= RANSAC_TRAD_MIN_POINTS_ON_LINE);
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
    dataSetDevisions.emplace_back( vector< point >() );
	makeToPoints (dataFromSensor,&(dataSetDevisions[0]));
	// find the first line
	findLine ((dataSetDevisions[0]).begin (), (dataSetDevisions[0]).end () , &newLine);
	// put line in array
	linesInData->emplace_back(newLine);

	// devide dataset
	cout << " dividing dataset" << endl;
	do
	{
		changeHappened = false;
		int sizeOfVector = dataSetDevisions.size ();

		for(int i = 0; i < sizeOfVector; i++)
		{
			if((dataSetDevisions[i]).size() > 2*SM_SPLITLINE_MIN_POINTS_IN_SPLITLINE)
			{
				deviationOfPoint = findGreatesDeviation ((dataSetDevisions[i]).begin (), (dataSetDevisions[i]).end () , &(linesInData->at (i)), &pointFurthestAway);
				// if deviation too great, split
				if(deviationOfPoint >= SM_SPLITLINE_TRESHHOLD_DISTANCE)
				{
					// repeat loop once more when done
					changeHappened = true;
					// add space for vector
                    dataSetDevisions.emplace_back( vector< point >() );

					// move split point into dataset
					if(distance((dataSetDevisions[i]).begin (),pointFurthestAway) < SM_SPLITLINE_MIN_POINTS_IN_SPLITLINE)
					{
						pointFurthestAway = (dataSetDevisions[i]).begin () + SM_SPLITLINE_MIN_POINTS_IN_SPLITLINE;
					}
					else if(distance(pointFurthestAway, ((dataSetDevisions[i]).end())-1) < SM_SPLITLINE_MIN_POINTS_IN_SPLITLINE)
					{
						pointFurthestAway = ((dataSetDevisions[i]).end() - 1) - SM_SPLITLINE_MIN_POINTS_IN_SPLITLINE;
					}

					// fill from division point (including)
					for(int j = 0; j < distance(pointFurthestAway, (dataSetDevisions[i]).end ()); j++)
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
	}while (changeHappened);

	cout << " merging dataset" << endl;
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
				if(deviationOfAngle < SM_MERGELINE_TRESHHOLD_ANGLE && deviationOfDistance < SM_MERGELINE_TRESHHOLD_DISTANCE)
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

	cout << " removing datasets" << endl;
	// remove lines with too small datapoints
	for(int i = dataSetDevisions.size ()-1; i >= 0; i-- )
	{
		if ((dataSetDevisions[i]).size() < SM_MIN_POINTS_IN_LINE)
		{
			dataSetDevisions.erase (dataSetDevisions.begin () + i);
			(*linesInData).erase (linesInData->begin () + i);
		}
	}
}


double findGreatesDeviation(vector<point>::iterator first, vector<point>::iterator last, line * lineFit, vector<point>::iterator * thisPoint)
{
	int sizeOfDataset = distance(first,last);
	long double deviation = 0;
	long double di; // deviation for current point (d_i)

	for(int i = 0; i < sizeOfDataset; i++)
	{
		di = abs((((first + i)->rho)*cos(((first + i)->theta)-(lineFit->theta))) - lineFit->rho);
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
		if(newPoint.rho != 0) // only enter point if it is not 0 = unreachable
		{
			newPoint.theta = ((i*ANGLE_STEP) + ANGLE_START);
			dataPoints->emplace_back(newPoint);
		}
	}

}
