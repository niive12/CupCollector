#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include "localisation_defines.h"

using namespace std;

inline double lidar_cos(unsigned int index) {
	return cosarr[index];
}
inline double lidar_sin(unsigned int index) {
	return sinarr[index];
}

struct robot_state_polar {
	double r;
	double theta;
	double alpha;
	robot_state_polar(double r, double theta, double alpha):r(r),theta(theta),alpha(alpha){}
	robot_state_polar():r(0),theta(0),alpha(0){}
};

struct line_polar {
	double r;
	double theta;
	line_polar(double r, double theta):r(r),theta(theta){}
};

struct line_lidar {
	unsigned int millimeters;
	unsigned int index;
	line_lidar(unsigned int mm, unsigned int i):millimeters(mm),index(i){}
	inline double to_rads() {
		return double(index)*((240.0/680.0-120.0)*PI/180.0);
	}
};


class lidarbot {
public:

	vector<line_polar> feature_map;
	robot_state_polar last_state;


	lidarbot()
	{
		feature_map = {{LINE1_DISTANCE,LINE1_ANGLE},
					   {LINE2_DISTANCE,LINE2_ANGLE},
					   {LINE3_DISTANCE,LINE3_ANGLE},
					   {LINE4_DISTANCE,LINE4_ANGLE}};

		last_state = robot_state_polar(ROBOT_START_DISTANCE,
										ROBOT_START_ANGLE,
										ROBOT_START_ORIENTATION);



	}

	vector<line_polar> feature_mapping(const vector<line_lidar> &data_features) {
		vector<line_polar> mapped_features;
		for(auto relative_feature : data_features) {
			double theta2 = relative_feature.to_rads()+last_state.alpha;
			double x = (last_state.r*cos(last_state.theta))
					+ double(relative_feature.millimeters) * cos(theta2);
			double y = (last_state.r*sin(last_state.theta))
					+ double(relative_feature.millimeters) * sin(theta2);
			double r = sqrt(x*x+y*y);
			double theta = atan2(y,x);
			mapped_features.emplace_back(r*cos(theta2-theta),theta2);
		}

		return move(mapped_features);
	}

	std::pair<unsigned int,double> match_to_feature_map(const line_polar &mapped_feature)
	{
		vector<std::pair<double, double>> deviations;

		for(auto global_feature:feature_map)
		{
			double angledeviation = std::abs(global_feature.theta - mapped_feature.theta);
			double distancedeviation = std::abs(global_feature.r - mapped_feature.r);
			deviations.emplace_back(angledeviation,distancedeviation);
		}


		double best = ANGLE_DEVIATION_MAX;
		unsigned int index = ANGLE_INDEX_NOMATCH;

		for(size_t i=0; i<deviations.size(); ++i)
		{
			auto dev = deviations[i];
			if(dev.first<best) {
				best=dev.first;
				index=i;
			}
		}

		return move(make_pair(index,best));
	}


	void test() {
		for(auto i:feature_map)
			cout << "R: " << i.r << ", theta: " << i.theta << endl;

		vector<line_lidar> data_features = {{100,100},{200,80},{159,488}};

		vector<line_polar> mapped_features = feature_mapping(data_features);
		unordered_map<unsigned int, double> matched_features;
		for(auto m:mapped_features) {
			auto match = match_to_feature_map(m);
			if(match.first != ANGLE_INDEX_NOMATCH ) {
			//If we have several data features matched to the same global feature (unlikely):
				if( !(matched_features.insert(match).second) ) {
					//Pick the best one:
					if(match.second < matched_features[match.first]) {
						matched_features.erase(match.first);
						matched_features.insert(match);
					}
				}
			}
		}

		/** If you don't have line features: */
		//1) Update robot state with odometry

		/** If you have line features: */
		//1) For each line, calculate the orientation (theta2 from feature map, minus theta wall (the one from LIDAR)).
		//2) The average of all the orientations of step 1 is the robot's new state orientation.
		//3) Be happy that you were able to use the LIDAR to calculate orientation.

		/** CORNER CASE	(2 lines with different angle (they intersect in a real corner) ) */
		//1) All global feature corners are known (compiler pre-calculated).
		//2) We find out which corner we are dealing with. Corner V.
		//3) Subtract two vectors from the corner position: vec1 vec2.
		//		vec1 is (the shortest distance from robot to line 1 (measured from LIDAR); line 1's angle (in feature map) ).
		//		vec2 is (the shortest distance from robot to line 2 (measured from LIDAR); line 2's angle (in feature map) ).
		//4) The result is the robot's new state position.
		//5) Be happy that you were able to use the LIDAR to calculate position.

		/** PARALLEL LINES CASE */
		//1) Update robot position with odometry (not orientation).

		/** SINGLE LINE CASE */
		//1) Update robot position with odometry (not orientation).

		/** CORNER CASE (2 or 4 corners in total, 3 lines) */
		//1) Do the same as in CORNER CASE 1, but average all the calculated
		//   robot state position results to obtain the new robot state position.

	}


};






int main()
{
	cout << "Hello World!" << endl;

	lidarbot hej;

	hej.test();











	return 0;
}

