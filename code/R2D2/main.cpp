#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

#define PI 3.141592

#define ANGLE_START 0.0
#define ANGLE_STEP (1*PI/180) // 1 deg in rad

#define MAKELINE_TRESHHOLD_DISTANCE 10



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


void makeToPoints(vector<int> * dataFromSensor, vector<point> * dataPoints);


int main()
{
	vector<int> dataaaaa = {170, 100, 100, 130, 100, 100};
	vector<line> resullllllllt;


	findFeatures (&dataaaaa, &resullllllllt);

	vector<line>::iterator haha = resullllllllt.begin ();

	while (haha != resullllllllt.end())
	{
		cout << (*haha).rho << ", " << haha->theta << endl;
		haha++;
	}

	return false;
}


void findLine(vector<point>::iterator start, vector<point>::iterator end, line * lineInData)
{
	int size = distance(start,end);
	// calc onr line
	line daLine;

	double I = 0.0, II = 0.0, III = 0.0, IV = 0.0, X = 0.0;

	// calc first part
	for(int i = 0; i < size; i++)
	{
		I += (pow((start + i)->rho,2) * sin(2*((start + i)->theta)));
	}

	// calc second part
	for(int i = 0; i < size; i++)
	{
		for(int j = 0; j < size; j++)
		{
			II += ((start + i)->rho)*((start + j)->rho)*(cos((start + i)->theta) * sin((start + j)->theta));
		}
	}
	II *= 2/size;

	// calc third part
	for(int i = 0; i < size; i++)
	{
		III += (pow((start + i)->rho,2) * cos(2*((start + i)->theta)));
	}

	// calc fourth part
	for(int i = 0; i < size; i++)
	{
		for(int j = 0; j < size; j++)
		{
			IV += ((start + i)->rho)*((start + j)->rho)*(cos(((start + i)->theta) + ((start + j)->theta)));
		}
	}
	IV *= 1/size;


	// calc total angle
	daLine.theta = (0.5)*atan((I-II)/(III-IV));

	// calc summation of r
	for(int i = 0; i < size; i++)
	{
		X += ((start + i)->rho)*(cos(((start + i)->theta) - daLine.theta));
	}

	// calc totatl distance
	daLine.rho = X/size;

	// outpur
	*lineInData = daLine;
}


void findFeatures(vector<int> * dataFromSensor, vector<line> * linesInData)
{
	linesInData->clear ();

	// make vector of vectors to divide data into and keep a second to store the line for the corresponding dataset (linesInData)

	vector< vector<point> > dataSetDevisions;
	line trol;
	vector<point>::iterator swag;
	double dev;
	bool changeHappened;

	// set all points in array
	dataSetDevisions.emplace_back(vector<point>());
	makeToPoints (dataFromSensor,&(dataSetDevisions[0]));
	// find the first line
	findLine ((dataSetDevisions[0]).begin (), (dataSetDevisions[0]).end ()-- , &trol);
	// put line in array
	linesInData->emplace_back(trol);

	do
	{
		changeHappened = false;
		int sizeOfVector = dataSetDevisions.size ();
		for(int i = 0; i < sizeOfVector; i++)
		{
			// test deviation
			dev = findGreatesDeviation ((dataSetDevisions[i]).begin (), ((dataSetDevisions[i]).end ()) , &(linesInData->at (i)), &swag);

			// if deviation too great, split
			if(dev >= MAKELINE_TRESHHOLD_DISTANCE)
			{
				// repeat loop once more when done
				changeHappened = true;
				// own expansion, if max distance point is at the edge of the dataset, remove it.
				if(((dataSetDevisions[i]).begin ()) == swag || (((dataSetDevisions[i]).end())-1) == swag)
				{
					(dataSetDevisions[i]).erase(swag);
				}
				else
				{
					// add space for vector
					dataSetDevisions.emplace_back(vector<point>());

					// fill from division point (including)
					for(int j = 0; j < distance(swag, (dataSetDevisions[i]).end ()--); j++)
					{
						// fill last vector (prev made) in vector
						(dataSetDevisions[(dataSetDevisions.size ()-1)]).emplace_back(*(swag + j));
					}
					// remove the end of the list (excluding division point)
					advance(swag,1);
					(dataSetDevisions[i]).erase((swag), (dataSetDevisions[i]).end ());
					// find line for vector which was split
					findLine ((dataSetDevisions[i]).begin (), (dataSetDevisions[i]).end ()-- , &trol);
					// input line to vector
					(*linesInData)[i] = trol;
					// find line for vector at end of vector (new one)
					findLine ((dataSetDevisions[(dataSetDevisions.size ()-1)]).begin (), (dataSetDevisions[(dataSetDevisions.size ()-1)]).end ()-- , &trol);
					// input line to vector
					(*linesInData).emplace_back(trol);
				}
			}
		}
	}while (changeHappened);
}


double findGreatesDeviation(vector<point>::iterator first, vector<point>::iterator last, line * lineFit, vector<point>::iterator * thisPoint)
{
	int size = distance(first,last);
	double deviation = 0;
	double di; // deviation for current point (d_i)

	for(int i = 0; i < size; i++)
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
	int size = dataFromSensor->size ();
	point wakeUp;
	for(int i = 0; i < size; i++)
	{
		wakeUp.rho = dataFromSensor->at (i);
		wakeUp.theta = (i*ANGLE_STEP + ANGLE_START);
		dataPoints->emplace_back(wakeUp);
	}

}
