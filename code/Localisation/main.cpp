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

inline double adjust_rads(double unadjusted) {
    while(unadjusted<0.0) unadjusted+=2.0*PI;
    while(unadjusted>(2.0*PI)) unadjusted-=2.0*PI;
    return unadjusted;
}

struct robot_state_polar {
    double r;
    double theta;
    double alpha;
    robot_state_polar(double r, double theta, double alpha):r(r),theta(theta),alpha(alpha){}
    robot_state_polar():r(0),theta(0),alpha(0){}
};

struct polar;
struct cartesian {
    double x;
    double y;
    cartesian(double x, double y):x(x),y(y) {}
    polar to_polar();
};
struct polar {
    double r;
    double theta;
    polar(double r, double theta):r(r),theta(adjust_rads(theta)){}

    cartesian to_cartesian() {
        double x = r*std::cos(theta);
        double y = r*std::sin(theta);
        return move(cartesian(x,y));
    }
};

polar cartesian::to_polar() {
    double r = std::sqrt(x*x+y*y);
    double theta = std::atan2(y,x);
    return move(polar(r,theta));
}

/*
struct line_lidar {
    double millimeters;
    unsigned int index;
    line_lidar(double mm, unsigned int i):millimeters(mm),index(i){}
    inline double to_rads() {
        double val = -1.0*((double(index)*(240.0)/680.0)-120.0)*PI/180.0;
        return adjust_rads(val);
    }
};
*/


class lidarbot {
public:

    vector<polar> line_map;
    vector<cartesian> corner_map;
    robot_state_polar last_state;


    lidarbot()
        :line_map(
    {{LINE1_DISTANCE,LINE1_ANGLE},
    {LINE2_DISTANCE,LINE2_ANGLE},
    {LINE3_DISTANCE,LINE3_ANGLE},
    {LINE4_DISTANCE,LINE4_ANGLE}}),
          corner_map( {
    {LINE1_DISTANCE,-LINE2_DISTANCE},
    {-LINE3_DISTANCE,-LINE2_DISTANCE},
    {-LINE3_DISTANCE,LINE4_DISTANCE},
    {LINE1_DISTANCE,LINE4_DISTANCE}
}),
          last_state(
    {ROBOT_START_DISTANCE,
              ROBOT_START_ANGLE,
              ROBOT_START_ORIENTATION})
    {}

    unsigned int getCornerIndex(unsigned int line1, unsigned int line2) {
        unsigned int ci=INDEX_NOMATCH;
        unsigned int small = std::min(line1,line2);
        unsigned int large = std::max(line1,line2);
        switch(small) {
        case 0:
            switch (large) {
            case 1:
                ci=0;
                break;
            case 3:
                ci=3;
                break;
            default:
                break;
            }
            break;
        case 1:
            switch(large) {
            case 2:
                ci=1;
                break;
            default:
                break;
            }
            break;
        case 2:
            switch(large) {
            case 3:
                ci=2;
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
        return ci;
    }

    vector<polar> feature_mapping(const vector<polar> &data_features) {
        vector<polar> mapped_features;
        for(auto relative_feature : data_features) {
            double theta2 = adjust_rads( relative_feature.theta+last_state.alpha );
            double x = (last_state.r*std::cos(last_state.theta))
                    + (relative_feature.r * std::cos(theta2));
            double y = (last_state.r*std::sin(last_state.theta))
                    +(relative_feature.r * std::sin(theta2));
            double r = std::sqrt(x*x+y*y);
            double theta = adjust_rads( std::atan2(y,x) );
            mapped_features.emplace_back(std::abs(r*std::cos(theta2-theta)),theta2);
        }

        return move(mapped_features);
    }

    pair<unsigned int,double> match_to_feature_map(const polar &mapped_feature)
    {
        vector<pair<double, double>> deviations;

        for(auto global_feature:line_map)
        {
            double angledeviation = std::max(global_feature.theta,mapped_feature.theta)-std::min(global_feature.theta,mapped_feature.theta);
            double distancedeviation = std::max(global_feature.r,mapped_feature.r)-std::min(global_feature.r,mapped_feature.r);
            deviations.emplace_back(angledeviation,distancedeviation);
        }


        double best = ANGLE_DEVIATION_MAX;
        unsigned int index = INDEX_NOMATCH;

        for(size_t i=0; i<deviations.size(); i++)
        {
            auto dev = deviations[i];
            if(dev.first<best) {
                if(dev.second<800.0) {
                    best=dev.first;
                    index=i;
                }
            }
        }

        return move(make_pair(index,best));
    }


    void test() {
        cout << "Features in global map:" << endl;
        for(auto i:line_map)
            cout << "R: " << i.r << ", theta: " << i.theta << endl;

        vector<polar> data_features = {{2920.13,6.18463},
                                       {562.286,4.70623},
                                       {1896.9,1.73712}};

        vector<polar> mapped_features = feature_mapping(data_features);
        unordered_map<unsigned int, pair<unsigned int, double> > matched_features;
        //for(auto m:mapped_features) {
        for(size_t m=0; m<mapped_features.size(); ++m) {
            pair<unsigned int, double> match = match_to_feature_map(mapped_features[m]);
            if(match.first != INDEX_NOMATCH ) {
                pair<unsigned int, pair<unsigned int, double> > funnymatch;
                funnymatch.first=match.first;
                funnymatch.second.second=match.second;
                funnymatch.second.first=m;
                //If we have several data features matched to the same global feature (unlikely):
                if( !(matched_features.insert(funnymatch).second) ) {
                    //Pick the best one:
                    if(match.second < matched_features[match.first].second) {
                        matched_features.erase(match.first);
                        matched_features.insert(funnymatch);
                    }
                }
            }
        }

        for(unsigned int i = 0; i<data_features.size(); ++i) {
            cout << "R: " << data_features[i].r << ", theta: " << data_features[i].theta <<",\t"
                 << "MappedR: " << mapped_features[i].r << ", MappedTheta: " << mapped_features[i].theta
                 << " (" << i << ")" << endl;
        }
        double avg = 0;
        for(auto i:matched_features) {
            avg+=i.second.second;
            cout << i.first << i.second.first << " " << flush;
        }
        avg/=matched_features.size();
        cout << endl << matched_features.size() << "\t" << avg << endl;

        /** If you don't have line features: */
        //1) Update robot state with odometry

        cout << "Old alpha: " << last_state.alpha << ", new alpha: " << flush;
        /** If you have line features: */
        //1) For each line, calculate the orientation (theta2 from feature map, minus theta wall (the one from LIDAR)).
        //2) The average of all the orientations of step 1 is the robot's new state orientation.
        //3) Be happy that you were able to use the LIDAR to calculate orientation.
        if(!matched_features.empty()) {
            double alpha_new=0;
            for(auto i:matched_features) {
                double increment = line_map[i.first].theta - data_features[i.second.first].theta;
                if(increment<0)
                    increment+=2.0*PI;
                alpha_new+=increment;
            }
            alpha_new/=matched_features.size();
            last_state.alpha = alpha_new;
        }
        cout << last_state.alpha << endl;

        /** CORNER CASE	(2 lines with different angle (they intersect in a real corner) ) */
        //1) All global feature corners are known (compiler pre-calculated).
        //2) We find out which corner we are dealing with. Corner V.
        //3) Subtract two vectors from the corner position: vec1 vec2.
        //		vec1 is (the shortest distance from robot to line 1 (measured from LIDAR); line 1's angle (in feature map) ).
        //		vec2 is (the shortest distance from robot to line 2 (measured from LIDAR); line 2's angle (in feature map) ).
        //4) The result is the robot's new state position.
        //5) Be happy that you were able to use the LIDAR to calculate position.

        cout << "Old R: " << last_state.r << ", old theta: " << last_state.theta << ", " << flush;
        if(matched_features.size()==2) {
            unordered_map<unsigned int,pair<unsigned int,double> >::iterator shiterator=matched_features.begin();
            ++shiterator;
            unsigned int index_line1 = (*(matched_features.begin())).first;
            unsigned int index_line2 = (*(shiterator)).first;
            unsigned int corner = getCornerIndex(index_line1, index_line2);
            if(corner!=INDEX_NOMATCH) {
                polar vec1 = line_map[index_line1];
                polar vec2 = line_map[index_line2];
                vec1.r -= data_features[(*(matched_features.begin())).second.first].r;
                vec2.r -= data_features[(*(shiterator)).second.first].r;
                if(vec1.r<0) {
                    vec1.theta=adjust_rads(vec1.theta+PI);
                    vec1.r=-vec1.r;
                }
                if(vec2.r<0) {
                    vec2.theta=adjust_rads(vec2.theta+PI);
                    vec2.r=-vec2.r;
                }
                cartesian vec1c = vec1.to_cartesian();
                cartesian vec2c = vec2.to_cartesian();
                cartesian robotcart(vec1c.x+vec2c.x,vec1c.y+vec2c.y);
                //cout << "Robot position: ( " << robotcart.x << " , " << robotcart.y << " ), " << flush;
                polar robotpolar = robotcart.to_polar();
                last_state.r=robotpolar.r;
                last_state.theta=robotpolar.theta;
            }
        }
        cout << "new R: " << last_state.r << ", new theta: " << last_state.theta << endl;

        /** PARALLEL LINES CASE */
        //1) Update robot position with odometry (not orientation).

        /** SINGLE LINE CASE */
        //1) Update robot position with odometry (not orientation).

        /** CORNER CASE (2 or 4 corners in total, 3 lines) */
        //if(matched_features.size()>3)
        //1) Do the same as in CORNER CASE 1, but average all the calculated
        //   robot state position results to obtain the new robot state position.
        cout << "Old R: " << last_state.r << ", old theta: " << last_state.theta << ", " << flush;
        if(matched_features.size()==3) {
            unordered_map<unsigned int,pair<unsigned int,double> >::iterator shiterator=matched_features.begin();
            ++shiterator;
            unordered_map<unsigned int,pair<unsigned int,double> >::iterator shiterator1=matched_features.begin();
            ++shiterator1;
            ++shiterator1;
            unsigned int index_line1 = (*(matched_features.begin())).first;
            unsigned int index_line2 = (*(shiterator)).first;
            unsigned int index_line3 = (*(shiterator1)).first;


            vector<pair<unsigned int, unsigned int>> intersecting_lines;
            if(getCornerIndex(index_line1, index_line2)!=INDEX_NOMATCH)
                intersecting_lines.emplace_back(make_pair(index_line1,index_line2));

            if(getCornerIndex(index_line1, index_line3)!=INDEX_NOMATCH)
                intersecting_lines.emplace_back(make_pair(index_line1,index_line3));

            if(getCornerIndex(index_line2, index_line3)!=INDEX_NOMATCH)
                intersecting_lines.emplace_back(make_pair(index_line2,index_line3));

            cartesian robotcart(0,0);

            for(size_t i = 0; i<intersecting_lines.size(); i++){
                polar vec1 = line_map[intersecting_lines[i].first];
                polar vec2 = line_map[intersecting_lines[i].second];

                if(intersecting_lines[i].first == index_line1)
                    vec1.r -= data_features[(*(matched_features.begin())).second.first].r;
                if(intersecting_lines[i].first == index_line2)
                    vec1.r -= data_features[(*(shiterator)).second.first].r;
                if(intersecting_lines[i].first == index_line3)
                    vec1.r -= data_features[(*(shiterator1)).second.first].r;

                if(intersecting_lines[i].second == index_line1)
                    vec2.r -= data_features[(*(matched_features.begin())).second.first].r;
                if(intersecting_lines[i].second == index_line2)
                    vec2.r -= data_features[(*(shiterator)).second.first].r;
                if(intersecting_lines[i].second == index_line3)
                    vec2.r -= data_features[(*(shiterator1)).second.first].r;

                if(vec1.r<0) {
                    vec1.theta=adjust_rads(vec1.theta+PI);
                    vec1.r=-vec1.r;
                }
                if(vec2.r<0) {
                    vec2.theta=adjust_rads(vec2.theta+PI);
                    vec2.r=-vec2.r;
                }
                cartesian vec1c = vec1.to_cartesian();
                cartesian vec2c = vec2.to_cartesian();
                robotcart.x += vec1c.x+vec2c.x;
                robotcart.y +=vec2c.y+vec2c.y;
            }
            robotcart.x /= intersecting_lines.size()-1;
            robotcart.y /= intersecting_lines.size()-1;

            polar robotpolar = robotcart.to_polar();
            last_state.r=robotpolar.r;
            last_state.theta=robotpolar.theta;
        }
        cout << "new R: " << last_state.r << ", new theta: " << last_state.theta << endl;
    }

};






int main()
{
    lidarbot hej;

    hej.test();











    return 0;
}

