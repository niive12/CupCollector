/**
 * @file test.hpp
 * @author Mikael Westermann
 */
#pragma once
#include "tekmap/tekmap.hpp"
#include <cmath>

using namespace std;

class testMap : public brushfireMap {
public:
	testMap(shared_ptr<Image> img, const pos_t &reachableFreeSpace)
		:brushfireMap(img,reachableFreeSpace)
	{}
	void test(shared_ptr<Image> img)
	{
		set<pos_t> loma;
		for(coordIndexType x=0;x<(coordIndexType)(getWidth());++x)
		{
			coordIndexType y0=2, y1=1, y2=0;
			while(y0<(coordIndexType)(getHeight()))
			{
				if(
						((this->const_coordVal(x,y0))<(this->const_coordVal(x,y1)))
						&&((this->const_coordVal(x,y2))<(this->const_coordVal(x,y1)))
						)
					loma.emplace(x,y1);
				++y0;
				++y1;
				++y2;
			}
		}
		for(coordIndexType y=0;y<(coordIndexType)(getHeight());++y)
		{
			coordIndexType x0=2, x1=1, x2=0;
			while(x0<(coordIndexType)(getWidth()))
			{
				if(
						((this->const_coordVal(x0,y))<(this->const_coordVal(x1,y)))
						&&((this->const_coordVal(x2,y))<(this->const_coordVal(x1,y)))
						)
					loma.emplace(x1,y);
				++x0;
				++x1;
				++x2;
			}
		}
		for(auto i:loma)
			img->setPixel8U(i.cx(),i.cy(),0);
	}
};

class tester {
public:
	static unordered_set<pos_t> getBrushEdges(const brushfireMap &brush, size_t radius)
	{
		unordered_set<pos_t> result;
		for(coordIndexType x = 0; x<(coordIndexType)brush.getWidth(); ++x) {
			for(coordIndexType y = 0; y<(coordIndexType)brush.getHeight(); ++y) {
				if(((brush.const_coordVal(x,y))%(2*radius))==radius)
				{
					result.emplace(x,y);
				}
			}
		}
		return move(result);
	}

	static void test(shared_ptr<Image> img)
	{
		const pos_t start = {ROBOT_START_X,ROBOT_START_Y};
		pixelshadeMap theImg(img);
		brushfireMap brush(theImg,start);
		unordered_set<pos_t> coords = getBrushEdges(brush,ROBOT_DYNAMICS_RADIUS);

		list<pos_t> visited;
		visited.push_back(start);

		while(!(coords.empty())) {
//			list<pos_t> pathToNext=theImg.getDijkstraPath(visited.back(),*(coords.begin()));
//			for(auto c:coords) {
//				list<pos_t> last=pathToNext;
//				pathToNext=theImg.getDijkstraPath(visited.back(),c);
//				if(pathToNext.size()>=last.size()) {
//					pathToNext=last;
//				}
//			}
			pos_t next=*(coords.begin());
			for(auto c:coords) {
				if((std::abs((c-visited.back()).cx())+std::abs((c-visited.back()).cy()))
						<(std::abs((next-visited.back()).cx())+std::abs((next-visited.back()).cy()))) {
					next=c;
				}
			}
			list<pos_t> pathToNext = theImg.getDijkstraPath(visited.back(),next);
			for(auto c:pathToNext) {
				visited.push_back(c);
				coords.erase(c);
			}
		}
		cout << "V: " << visited.size() << endl;
		for(auto v:visited)
			img->setPixel8U(v.cx(),v.cy(),0);
		img->saveAsPGM("test.pgm");
		theImg.shade(img);
	}
};
