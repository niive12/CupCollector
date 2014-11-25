/** @file */
#include "scanner.h"
#include <array>
#include <set>
#include <cmath>

unordered_set<pos_t> scanner::getCircle(const pos_t &center, unsigned int radius) {
	unordered_set<pos_t> result;
	if(r!=radius)
		update_circle(radius);
	for(auto i:circle)
		result.insert(center+i);
	return move(result);
}

unordered_set<pos_t> scanner::getCircle(const pos_t &center)
{ return move(getCircle(center,r)); }

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
size_t scanner::scan(pos_t center, const pixelshade_map &scanmap) {
	return move(this->scan(center,scanmap,r));
}

list<pos_t> scanner::scanlistThroughWalls(pos_t center, const pixelshade_map &scanmap, unsigned int radius)
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
list<pos_t> scanner::scanlistThroughWalls(pos_t center, const pixelshade_map &scanmap) {
	return move(this->scanlistThroughWalls(center,scanmap,r));
}

list<pos_t> scanner::scanlistAroundWalls(pos_t center, const pixelshade_map &scanmap, unsigned int radius)
{
	list<pos_t> cups;
	if(r!=radius)
			update_circle(radius);

	list<pos_t> reachables = scanmap.getReachables(getCircle(center),center);

	for(auto i:reachables) {
		try {
			if( WSPACE_IS_CUP(scanmap.const_coordVal(i))) {
				cups.push_back(i);
			}
		} catch (const std::out_of_range& oor) {(void)oor;}
	}
	return move(cups);
}
list<pos_t> scanner::scanlistAroundWalls(pos_t center, const pixelshade_map &scanmap) {
	return move(this->scanlistAroundWalls(center,scanmap,r));
}

void scanner::getCupsInSight(const pixelshadeMap &scanmap, set<pos_t> &cups, const pos_t &to, const pos_t &from)
{
	pos_t rayDir = to-from;
	pos_t rayPos = from;

	//To make calculations easier to read:
	const double dx = rayDir.cx();
	const double dy = rayDir.cy();

	//Is the y-difference larger than the x-difference?
	const bool steep = std::fabs(dy) > std::fabs(dx);
	//If steep, then we always move 1 pixel in y-dimension
	// and dx/abs(dy) in x-dimension.
	// If not steep, we move 1 pixel in x-dimension
	// and dy/abs(dx) in y-dimension.
	const double ystep = steep ? ( ( dy > 0 ) ? 1 : -1 ) : ( dy / std::fabs(dx) );
	const double xstep = steep ? ( dx / std::fabs(dy) ) : ( ( dx > 0 ) ? 1 : -1 );

	double rayPosX = rayPos.cx();
	double rayPosY = rayPos.cy();

	// Stay in while-loop until obstacle or cup is found:
	while(1) /** @todo Move the loop-ending conditions to here. */
	{
		if(WSPACE_IS_CUP(scanmap.const_coordVal(rayPos)))
			cups.insert(rayPos);
		//Break loop if ray has found target
		if(rayPos==to) {
			break;
		}
		//Break loop if ray has exceeded image borders:
		if( !scanmap.isInMap(rayPos) )
			break;
		// Break loop if ray has hit obstacle:
		if( WSPACE_IS_OBSTACLE(scanmap.const_coordVal(rayPos)) )
			break;
		//Advance ray:
		rayPosX+=xstep;
		rayPosY+=ystep;
		rayPos.x() = std::round(rayPosX);
		rayPos.y() = std::round(rayPosY);
	}
}

list<pos_t> scanner::scanlistLineOfSight(pos_t center, const pixelshade_map &scanmap, unsigned int radius)
{
	set<pos_t> cups;
	forward_list<pos_t> perimeter = getPerimeter(center,radius);
	for(auto p:perimeter)
		getCupsInSight(scanmap,cups,p,center);
	return move( list<pos_t>( cups.begin(), cups.end() ) );
}
list<pos_t> scanner::scanlistLineOfSight(pos_t center, const pixelshade_map &scanmap) {
	return move(this->scanlistLineOfSight(center,scanmap,r));
}


forward_list<pos_t> scanner::getPerimeter(const pos_t &center, unsigned int radius)
{
	forward_list<pos_t> resultCircle;
	const array<array<int,2>,8> neighbours =
	{{  {-1,0}, /* W */ {1,0},  /* E */
		{0,-1}, /* N */ {0,1},  /* S */
		{-1,-1},/* NW */{1,-1}, /* NE */
		{1,1},  /* SE */{-1,1}  /* SW */
	 }};
	int rsquared = radius*radius;
	int r1squared = (radius-1)*(radius-1);
	queue<pos_t> q;
	q.emplace(0,0);
	set<pos_t> visited;
	while(!q.empty()) {
			pos_t cur = q.front();
			q.pop();
			if( (cur.cx()*cur.cx()+cur.cy()*cur.cy()) > r1squared )
				resultCircle.push_front(cur+center);
			for(auto n:neighbours) {
				int v=((cur.cx()+n[0])*(cur.cx()+n[0])+(cur.cy()+n[1])*(cur.cy()+n[1]));
					if( v<=rsquared )
							if((visited.emplace(cur.cx()+n[0], cur.cy()+n[1])).second)
									q.emplace(cur.cx()+n[0], cur.cy()+n[1]);
			}
	}
	return move(resultCircle);
}


unsigned int scanner::radius() {
	return r;
}
