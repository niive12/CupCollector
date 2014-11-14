/** @file */
#include "scanner.h"
#include <array>
#include <set>

unordered_set<pos_t> scanner::getCircle(const pos_t &center, unsigned int radius) {
	unordered_set<pos_t> result;
	if(r!=radius)
		update_circle(radius);
	for(auto i:circle)
		result.insert(center+i);
	return move(result);
}

scanner::scanner() {}
scanner::scanner(unsigned int radius)
{
	update_circle(radius);
}

void scanner::update_circle(unsigned int radius)
{
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
			set<pos_t> visited;
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

size_t scanner::scan(pos_t center, const pixelshade_map &scanmap, unsigned int radius)
{
		size_t number_of_cups = 0;
		if(r!=radius)
				update_circle(radius);
		for(auto i:circle) {
				try {
						if( WSPACE_IS_CUP(scanmap.const_coordVal(center+i))) {
								++number_of_cups;
						}
				} catch (const std::out_of_range& oor) {(void)oor;}
		}
		return number_of_cups;
}

list<pos_t> scanner::scanlist(pos_t center, const pixelshade_map &scanmap, unsigned int radius)
{
	list<pos_t> cups;
	if(r!=radius)
			update_circle(radius);
	for(auto i:circle) {
		try {
			if( WSPACE_IS_CUP(scanmap.const_coordVal(center+i))) {
				cups.push_back(center+i);
			}
		} catch (const std::out_of_range& oor) {(void)oor;}
	}
	return move(cups);
}
