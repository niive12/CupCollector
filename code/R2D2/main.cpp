#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

#define PI 3.141592

#define DEGREE_TO_RAD(DEGREES) ((DEGREES*PI/180))

#define ANGLE_START 0.0
#define ANGLE_STEP DEGREE_TO_RAD(1) // 1 deg in rad

#define MAKELINE_TRESHHOLD_DISTANCE 10

#define MERGELINE_TRESHHOLD_DISTANCE 1
#define MERGELINE_TRESHHOLD_ANGLE DEGREE_TO_RAD(2)



struct line
{
	double rho; // distance
	double theta; // angle
};

using point = line;


/**
 * @brief findFeatures findes the lines in the data set
 * @param dataFromSensor an array of distance to nearest object from the sensor
 * @param linesInData returns the data in (\rho,\theta)
 */
void findFeatures(vector<int> * dataFromSensor, vector<line> * linesInData);

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


int main()
{
	vector<int> simulatedDataFromSensor = {170, 100, 100, 130, 100, 100};
	vector<line> testOutputLines;


	findFeatures (&simulatedDataFromSensor, &testOutputLines);

	vector<line>::iterator itForOutput = testOutputLines.begin ();

	for(int i = 0; i < testOutputLines.size (); i++)
	{
		cout << (testOutputLines[i]).rho << ", " << (testOutputLines[i]).theta << endl;
	}

	return false;
}


void findLine(vector<point>::iterator start, vector<point>::iterator end, line * lineInData)
{
	int sizeOfVector = distance(start,end);
	line bestFitLine;

	// the equation is divided into four part spliting at the sums, uniformly weigted datapoints assumed
	// I, II, III and IV are for the angle and X for the distance
	double I = 0.0, II = 0.0, III = 0.0, IV = 0.0, X = 0.0;

	for(int i = 0; i < sizeOfVector; i++)
	{
		// calc first and third part
		I += (pow((start + i)->rho,2) * sin(2*((start + i)->theta)));
		III += (pow((start + i)->rho,2) * cos(2*((start + i)->theta)));
		for(int j = 0; j < sizeOfVector; j++)
		{
			// calc second and fourth part
			II += ((start + i)->rho)*((start + j)->rho)*(cos((start + i)->theta) * sin((start + j)->theta));
			IV += ((start + i)->rho)*((start + j)->rho)*(cos(((start + i)->theta) + ((start + j)->theta)));
		}
	}
	II *= 2/sizeOfVector;
	IV *= 1/sizeOfVector;

	// calc total angle
	bestFitLine.theta = (0.5)*atan((I-II)/(III-IV));

	// calc summation of r
	for(int i = 0; i < sizeOfVector; i++)
	{
		X += ((start + i)->rho)*(cos(((start + i)->theta) - bestFitLine.theta));
	}

	// calc total distance
	bestFitLine.rho = X/sizeOfVector;

	// output
	*lineInData = bestFitLine;
}


void findFeatures(vector<int> * dataFromSensor, vector<line> * linesInData)
{
	linesInData->clear ();

	// make vector of vectors to divide data into and keep a second to store the line for the corresponding dataset (linesInData = already created)

	vector< vector<point> > dataSetDevisions;
	line newLine;
	vector<point>::iterator pointFurthestAway;
	double deviationOfPoint;
	bool changeHappened;

	// set all points in array
	dataSetDevisions.emplace_back(vector<point>());
	makeToPoints (dataFromSensor,&(dataSetDevisions[0]));
	// find the first line
	findLine ((dataSetDevisions[0]).begin (), (dataSetDevisions[0]).end ()-- , &newLine);
	// put line in array
	linesInData->emplace_back(newLine);

	// devide dataset
	do
	{
		changeHappened = false;
		int sizeOfVector = dataSetDevisions.size ();

		for(int i = 0; i < sizeOfVector; i++)
		{
			// test deviation
			deviationOfPoint = findGreatesDeviation ((dataSetDevisions[i]).begin (), ((dataSetDevisions[i]).end ()) , &(linesInData->at (i)), &pointFurthestAway);

			// if deviation too great, split
			if(deviationOfPoint >= MAKELINE_TRESHHOLD_DISTANCE)
			{
				// repeat loop once more when done
				changeHappened = true;
				// own expansion, if max distance point is at the edge of the dataset, remove it.
				if(((dataSetDevisions[i]).begin ()) == pointFurthestAway || (((dataSetDevisions[i]).end())-1) == pointFurthestAway)
				{
					(dataSetDevisions[i]).erase(pointFurthestAway);
				}
				else
				{
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
					findLine ((dataSetDevisions[i]).begin (), (dataSetDevisions[i]).end ()-- , &newLine);
					// input line to vector
					(*linesInData)[i] = newLine;
					// find line for vector at end of vector (new one)
					findLine ((dataSetDevisions[(dataSetDevisions.size ()-1)]).begin (), (dataSetDevisions[(dataSetDevisions.size ()-1)]).end ()-- , &newLine);
					// input line to vector
					(*linesInData).emplace_back(newLine);
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
					findLine ((dataSetDevisions[i]).begin (), (dataSetDevisions[i]).end ()-- , &newLine);
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


double findGreatesDeviation(vector<point>::iterator first, vector<point>::iterator last, line * lineFit, vector<point>::iterator * thisPoint)
{
	int sizeOfDataset = distance(first,last);
	double deviation = 0;
	double di; // deviation for current point (d_i)

	for(int i = 0; i < sizeOfDataset; i++)
	{
		di = ((((first + i)->rho)*cos(((first + i)->theta)-(lineFit->theta))) - lineFit->rho);
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
		newPoint.theta = (i*ANGLE_STEP + ANGLE_START);
		dataPoints->emplace_back(newPoint);
	}

}
