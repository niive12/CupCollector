/**
 * @file test.hpp
 * @author Mikael Westermann
 */
#pragma once
#include "tekmap/tekmap.hpp"
#include <cmath>
#include <algorithm> //merge
#include <iomanip>
#include <sstream>
#include <forward_list>


using namespace std;

class tester {
public:
	static unordered_set<pos_t> getBrushEdges(const brushfireMap &brush, size_t radius)
	{
		unordered_set<pos_t> result;
		for(coordIndexType x = 0; x<(coordIndexType)brush.getWidth(); ++x) {
			for(coordIndexType y = 0; y<(coordIndexType)brush.getHeight(); ++y) {
				if(((brush.const_coordVal(x,y)-1)%(2*radius))==radius)
				{
					result.emplace(x,y);
				}
			}
		}
		return move(result);
	}

	static pos_t getClosestCoord(const pixelshadeMap &img,
								 const unordered_set<pos_t> &coordinatesToSearch, const pos_t &currentPos)
	{
		pos_t result=currentPos;
		const array<array<int,2>,8> neighbours =
		{{ {-1,0}, /* W */ {1,0}, /* E */
		   {0,-1}, /* N */ {0,1}, /* S */
		   {-1,-1},/* NW */{1,-1}, /* NE */
		   {1,1}, /* SE */{-1,1} /* SW */
		 }} ;
		unordered_set<pos_t> visited;
		visited.insert(currentPos);
		queue<pos_t> q;
		q.push(currentPos);
		while((!q.empty())&&(result==currentPos)) {
			pos_t v = q.front();
			q.pop();
			for(auto n : neighbours) {
				pos_t w = v+pos_t(n.at(0),n.at(1));
				if( (visited.insert(w).second)
						&& (!(WSPACE_IS_OBSTACLE(img.const_coordVal(w)))) )
				{
					if( (coordinatesToSearch.find(w)!=coordinatesToSearch.end()) ) {
						result=w;
						break;
					}
					else
						q.push(w);
				}
			}
		}
		return move(result);
	}

	static void pad(pixelshadeMap &imgToPad, const brushfireMap &brush, const size_t padValue) {
		for(coordIndexType x=0; x<(coordIndexType)imgToPad.getWidth(); ++x)
			for(coordIndexType y=0; y<(coordIndexType)imgToPad.getHeight(); ++y)
				if(brush.const_coordVal(x,y) <= (brushfireMap::myValType)(padValue))
					imgToPad.coordVal(x,y) = WSPACE_OBSTACLE;
	}

	static unordered_set<pos_t> getLocalMaxima(const pixelshadeMap &img,
											   const brushfireMap &brush,
											   const pos_t &validFreespaceCoord,
											   bool twoPointMaxima=true)
	{
		unordered_set<pos_t> result;
		unordered_set<pos_t> freespace = img.findFreespace(validFreespaceCoord);
		for(coordIndexType x=0;x<(coordIndexType)(img.getWidth());++x) {
			coordIndexType y0=2, y1=1, y2=0;
			while(y0<(coordIndexType)(img.getHeight())) {
				if(		((brush.const_coordVal(x,y0))<(brush.const_coordVal(x,y1)))
						&&(twoPointMaxima?
							((brush.const_coordVal(x,y2))<=(brush.const_coordVal(x,y1)))
						:	((brush.const_coordVal(x,y2))<(brush.const_coordVal(x,y1))) )
						) {
					pos_t pos(x,y1);
					if(freespace.find(pos)!=freespace.end()) {
						if((brush.const_coordVal(x,y2))==(brush.const_coordVal(x,y1))) {
							if((brush.const_coordVal(x,y2-1))==(brush.const_coordVal(x,y0)))
								result.insert(pos);
						}
						else if((brush.const_coordVal(x,y2))==(brush.const_coordVal(x,y0))) {
							result.insert(pos);
						}
					}
				}
				++y0;
				++y1;
				++y2;
			}
		}
		for(coordIndexType y=0;y<(coordIndexType)(img.getHeight());++y) {
			coordIndexType x0=2, x1=1, x2=0;
			while(x0<(coordIndexType)(img.getWidth())) {
				if(		((brush.const_coordVal(x0,y))<(brush.const_coordVal(x1,y)))
						&&(twoPointMaxima?
							((brush.const_coordVal(x2,y))<=(brush.const_coordVal(x1,y)))
						:	((brush.const_coordVal(x2,y))<(brush.const_coordVal(x1,y))) )
						) {
					pos_t pos(x1,y);
					if(freespace.find(pos)!=freespace.end()) {
						if((brush.const_coordVal(x2,y))==(brush.const_coordVal(x1,y))) {
							if((brush.const_coordVal(x2-1,y))==(brush.const_coordVal(x0,y)))
								result.insert(pos);
						}
						else if((brush.const_coordVal(x2,y))==(brush.const_coordVal(x0,y))) {
							result.insert(pos);
						}
					}
				}
				++x0;
				++x1;
				++x2;
			}
		}
		return move(result);
	}

	static bool isRemainder(pos_t center, const unordered_set<pos_t> &coords, unsigned int radius)
	{
		static unsigned int r=0;
		static forward_list<pos_t> circle;
		bool remainder=true;
		if(r!=radius) {
			circle.clear();
			const array<array<int,2>,8> neighbours =
			{{  {-1,0}, /* W */ {1,0},  /* E */
				{0,-1}, /* N */ {0,1},  /* S */
				{-1,-1},/* NW */{1,-1}, /* NE */
				{1,1},  /* SE */{-1,1}  /* SW */
			 }};
			r=radius;
			int rsquared = r*r;
			queue<pos_t> q;
			q.emplace(0,0);
			unordered_set<pos_t> visited;
			while(!q.empty()) {
				pos_t cur = q.front();
				q.pop();
				circle.push_front(cur);
				for(auto n:neighbours)
					if( ((cur.cx()+n[0])*(cur.cx()+n[0])
						 +(cur.cy()+n[1])*(cur.cy()+n[1]))<=rsquared )
						if((visited.emplace(cur.cx()+n[0], cur.cy()+n[1])).second)
							q.emplace(cur.cx()+n[0], cur.cy()+n[1]);
			}
		}
		for(auto i:circle) {
			try {
				if(coords.find(center+i)!=coords.end()) {
					remainder=false;
					break;
				}
			} catch (const std::out_of_range& oor) {(void)oor;}
		}
		return remainder;
	}

	static void getRemainders(const pixelshadeMap &img, unordered_set<pos_t> &coordSet, const pos_t &freespaceCoord) {
		unordered_set<pos_t> freespace = img.findFreespace(freespaceCoord);
		for(auto c:freespace)
			if(isRemainder(c,coordSet,ROBOT_DYNAMICS_RADIUS))
				coordSet.insert(c);
	}

	static void test(shared_ptr<Image> img, bool banim=true)
	{
		const pos_t start = {ROBOT_START_X,ROBOT_START_Y};
		pixelshadeMap theImg(img), configurationSpace(img);
		brushfireMap brush(theImg,start);
		pad(configurationSpace,brush,ROBOT_DYNAMICS_RADIUS);
		unordered_set<pos_t> coords = getBrushEdges(brush,ROBOT_DYNAMICS_RADIUS);

		if(banim) {
			cout << "Saving configuration space as \"test_configuration_space.pgm\"... " << flush;
			configurationSpace.shade(img);
			img->saveAsPGM("test_configuration_space.pgm");
			theImg.shade(img);
			cout << "Done." << endl;
		}

		if(banim) {
			cout << "Saving brushfire edges as \"test_brushfire_edges.pgm\"... " << flush;
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM("test_brushfire_edges.pgm");
			theImg.shade(img);
			cout << "Done" << endl;
		}

//		//Local maxima are thrown into coords here:
//		unordered_set<pos_t> loMa = getLocalMaxima(configurationSpace,brush,start,true);
//		if(banim) {
//			cout << "Saving local maxima as \"test_local_maxima.pgm\"... " << flush;
//			for(auto c:loMa)
//				img->setPixel8U(c.cx(),c.cy(),20);
//			img->saveAsPGM("test_local_maxima.pgm");
//			theImg.shade(img);
//			cout << "Done." << endl;
//		}
//		coords.insert(loMa.begin(),loMa.end());
//		if(banim) {
//			cout << "Saving brushfire edges and local maxima as \"test_full_coordinate_set.pgm\"... " << flush;
//			for(auto c:coords)
//				img->setPixel8U(c.cx(),c.cy(),20);
//			img->saveAsPGM("test_full_coordinate_set.pgm");
//			theImg.shade(img);
//			cout << "Done." << endl;
//		}

		getRemainders(configurationSpace,coords,start);
		if(banim) {
			cout << "Saving brushfire edges and remainders as \"test_full_be_and_remainders.pgm\"... " << flush;
			for(auto c:coords)
				img->setPixel8U(c.cx(),c.cy(),20);
			img->saveAsPGM("test_full_be_and_remainders.pgm");
			theImg.shade(img);
			cout << "Done." << endl;
		}

		list<pos_t> robotPath;
		robotPath.push_back(start);

		auto n = coords.size();
		size_t progress = 1;
		cout << "Starting robot movement. Points to visit (C): " << n << "." << endl;
		while(!(coords.empty())) {
			auto lastProgress = progress;
			progress =(100*(n-coords.size()))/n;
			if(lastProgress!=progress) {
				cout << "\rPoints left in C: " << coords.size() << ", Traveled length: " << robotPath.size()
					 << ", Progress: " << progress << " %        " << flush;

				if(banim) {
					ostringstream anim;
					anim << "test_robot_path_" << setw(3) << setfill('0') << progress << ".pgm";
					img->saveAsPGM(anim.str());
				}
			}

			pos_t next = getClosestCoord(configurationSpace,coords,robotPath.back());
			if(next==robotPath.back()) {
				cout << "\nError encountered. Saving remaining coordinates as \"test_unreachable_coordinates.pgm\"..." << endl;
				theImg.shade(img);
				for(auto c:coords)
					img->setPixel8U(c.cx(),c.cy(),87);
				img->saveAsPGM("test_unreachable_coordinates.pgm");
				theImg.shade(img);
				cout << "Done.\nTest has ended because of error." << endl;
				return;
			}
			coords.erase(robotPath.back());
			coords.erase(next);
			list<pos_t> pathToNext = configurationSpace.getDijkstraPath(robotPath.back(),next);
			for(auto c:pathToNext) {
				robotPath.push_back(c);
				if(banim) img->setPixel8U(c.cx(),c.cy(),20);
				coords.erase(c);
			}
			robotPath.push_back(next);
			if(banim) img->setPixel8U(next.cx(),next.cy(),20);
		}
		cout << "\r                                                                " << endl;
		if(banim) {
			cout << "Saving robot path as \"test_robot_path_IDX.pgm\"... " << flush;
			for(auto v:robotPath)
				img->setPixel8U(v.cx(),v.cy(),20);
			img->saveAsPGM("test_robot_path_full.pgm");
			theImg.shade(img);
			cout << "Done." << endl;
		}

		cout << "Total traveled length: " << robotPath.size() << endl;
	}
};
