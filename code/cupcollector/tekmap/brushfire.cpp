/** @file
* @author Lukas, Michael, Mikkel, Nikolaj & Mikael
* @date 21-10-2014
*
* Group project.
*/
#include "brushfire.h"
#include <vector>
#include <queue>
#include <set>
#include <unordered_set>


//--------Functions for brushfireMap start ------------/
list< pos_t > brushfireMap::findCoords(const pixelshadeMap &img, const pos_t &withinFreeSpace, const bool borders) const
{
	list<pos_t> resulting_coords;
	if(!img.isInMap(withinFreeSpace))
		cerr << "coord given to findCoords is not in image." << endl;
	else {
		const array<array<int,2>,8> neighbours =
		{{ {-1,0}, /* W */ {1,0}, /* E */
		   {0,-1}, /* N */ {0,1}, /* S */
		   {-1,-1},/* NW */{1,-1}, /* NE */
		   {1,1}, /* SE */{-1,1} /* SW */
		 }};
		vector<vector<bool> > visited(img.getWidth(),vector<bool>(img.getHeight(),false));
		queue<pos_t> q; //FIFO
		q.push(withinFreeSpace);
		while(!q.empty())
		{
			pos_t cur = q.front();
			q.pop();
			if(!borders) //We know it's unvisited.
				//resulting_coords.insert(cur);
				resulting_coords.push_back(cur);
			for(auto n : neighbours) {
				pos_t w = cur+pos_t(n.at(0),n.at(1));
				//If neighbour is within image borders:
				if( img.isInMap(w) ) {
					//if the neighbour is an unvisited non-obstacle:
					if( (!WSPACE_IS_OBSTACLE( img.const_coordVal(w) ))
							&& (!( (visited[w.cx()])[w.cy()] ) ) ) {
						//Add it to the wavefront
						q.push(w);
						(visited[w.cx()])[w.cy()]=true;
					}
					if(borders&&WSPACE_IS_OBSTACLE( img.const_coordVal(w) )){
						//If the neighbour WAS an obstacle, it must have been a border.
						if(!((visited[w.cx()])[w.cy()])) {
							//resulting_coords.emplace(cur.first+n.at(0),cur.second+n.at(1));
							resulting_coords.push_back(w);
							(visited[w.cx()])[w.cy()]=true;
						}
					}
				}
			}
			(visited[cur.cx()])[cur.cy()] = true;
		}
	}
	return move(resulting_coords);
}

list< pos_t > brushfireMap::findObstacleBorders(shared_ptr<Image> img) const
{
	set<pos_t> resulting_coords;
	const array<array<int,2>,8> neighbours =
	{{ {-1,0}, /* W */ {1,0}, /* E */
	   {0,-1}, /* N */ {0,1}, /* S */
	   {-1,-1},/* NW */{1,-1}, /* NE */
	   {1,1}, /* SE */{-1,1} /* SW */
	 }};
	for( int x = 0; x < (int)(img->getWidth()); ++x) {
		for( int y = 0; y < (int)(img->getHeight()); ++y) {
			if( WSPACE_IS_OBSTACLE( img->getPixelValuei(x,y,0) ) ) {
				for(auto n : neighbours) {
					pos_t w = pos_t(x+n.at(0),y+n.at(1));
					//If neighbour is within image borders:
					if( isInImage( img, w ) ) {
						//if the neighbour is not an obstacle:
						if( !WSPACE_IS_OBSTACLE( img->getPixelValuei(w.cx(),w.cy(),0) ) ) {
							resulting_coords.emplace(x,y);
							break;
						}
					}
				}
			}
		}
	}
	return move(list<pos_t>(resulting_coords.begin(),resulting_coords.end()));
}

list< pos_t > brushfireMap::findObstacleBorders(const pixelshadeMap &img) const
{
	set<pos_t> resulting_coords;
	const array<array<int,2>,8> neighbours =
	{{ {-1,0}, /* W */ {1,0}, /* E */
	   {0,-1}, /* N */ {0,1}, /* S */
	   {-1,-1},/* NW */{1,-1}, /* NE */
	   {1,1}, /* SE */{-1,1} /* SW */
	 }};
	for( int x = 0; x < (int)(img.getWidth()); ++x) {
		for( int y = 0; y < (int)(img.getHeight()); ++y) {
			if( WSPACE_IS_OBSTACLE( img.const_coordVal(x,y) ) ) {
				for(auto n : neighbours) {
					pos_t w = pos_t(x+n.at(0),y+n.at(1));
					//If neighbour is within image borders:
					if( img.isInMap(w) ) {
						//if the neighbour is not an obstacle:
						if( !WSPACE_IS_OBSTACLE( img.const_coordVal(w) ) ) {
							resulting_coords.emplace(x,y);
							break;
						}
					}
				}
			}
		}
	}
	return move(list<pos_t>(resulting_coords.begin(),resulting_coords.end()));
}

brushfireMap::brushfireMap(shared_ptr< Image > img)
	:tekMap(img)
{
	list<pos_t> coords;
	//Fill the coords set:
	coords = findObstacleBorders(img); //non-efficient
	wave(img,coords);
}

brushfireMap::brushfireMap(shared_ptr< Image > img, const pos_t &reachableFreeSpace)
	:tekMap(img)
{
	list<pos_t> coords;
	//Fill the coords set:
	coords = findObstacleBorders(img,reachableFreeSpace); //faster
	wave(img,coords);
}

brushfireMap::brushfireMap(const pixelshadeMap &img)
	:tekMap(img)
{
	list<pos_t> coords;
	//Fill the coords set:
	coords = findObstacleBorders(img); //non-efficient
	wave(img,coords);
}

brushfireMap::brushfireMap(const pixelshadeMap &img, const pos_t &reachableFreeSpace)
	:tekMap(img)
{
	list<pos_t> coords;
	//Fill the coords set:
	coords = findObstacleBorders(img,reachableFreeSpace); //faster
	wave(img,coords);
}

void brushfireMap::wave(const pixelshadeMap &img, const list<pos_t> &goals)
{
	const array<array<int,2>,8> neighbours =
	{{ {-1,0}, /* W */ {1,0}, /* E */
	   {0,-1}, /* N */ {0,1}, /* S */
	   {-1,-1},/* NW */{1,-1}, /* NE */
	   {1,1}, /* SE */{-1,1} /* SW */
	 }} ;
	myMap.clear();
	myMap.resize( img.getWidth(), vector<myValType>( img.getHeight() , WAVE_VAL_UNV));
	vector<vector<bool> > visited(img.getWidth(),vector<bool>(img.getHeight(),false));
	queue<pos_t> q; //FIFO
	for(auto i : goals) {
		(myMap[i.cx()])[i.cy()] = WAVE_VAL_GOAL;
		q.push(i);
	}
	while(!q.empty())
	{
		pos_t cur = q.front();
		q.pop();
		for(auto n : neighbours) {
			pos_t w=cur+pos_t(n.at(0),n.at(1));
			//If neighbour is within image borders:
			if( img.isInMap(w) ) {
				//if the neighbour is an unvisited non-obstacle:
				if( ( !WSPACE_IS_OBSTACLE( img.const_coordVal(w) ) )
						&& ( !( (visited[w.cx()])[w.cy()] ) )
						&& ((myMap[w.cx()])[w.cy()] != WAVE_VAL_GOAL)
						)
				{
					//increment:
					(myMap[w.cx()])[w.cy()]
							= (myMap[cur.cx()])[cur.cy()] +1;
					(visited[w.cx()])[w.cy()]=true;
					//Add it to the wavefront
					//q.emplace(w.cx(),w.cy());
					q.push(w);
				}
			}
		}
		(visited[cur.cx()])[cur.cy()]=true;
	}
}




//--------Functions for brushfireMap end ------------/





//--------Functions for norm2BrushfireMap start ------------/

list< pos_t > norm2BrushfireMap::findObstacleBorders(const pixelshadeMap &img) const
{
	unordered_set<pos_t> resulting_coords;
	const array<array<int,2>,8> neighbours =
	{{ {-1,0}, /* W */ {1,0}, /* E */
	   {0,-1}, /* N */ {0,1}, /* S */
	   {-1,-1},/* NW */{1,-1}, /* NE */
	   {1,1}, /* SE */{-1,1} /* SW */
	 }};
	for( int x = 0; x < (int)(img.getWidth()); ++x) {
		for( int y = 0; y < (int)(img.getHeight()); ++y) {
			if( WSPACE_IS_OBSTACLE( img.const_coordVal(x,y) ) ) {
				for(auto n : neighbours) {
					pos_t w = pos_t(x+n.at(0),y+n.at(1));
					//If neighbour is within image borders:
					if( img.isInMap(w) ) {
						//if the neighbour is not an obstacle:
						if( !WSPACE_IS_OBSTACLE( img.const_coordVal(w) ) ) {
							resulting_coords.emplace(x,y);
							break;
						}
					}
				}
			}
		}
	}
	return move(list<pos_t>(resulting_coords.begin(),resulting_coords.end()));
}

void norm2BrushfireMap::wave(const pixelshadeMap &img, const list<pos_t> &goals)
{
	const double sqrt2 =1.4142135623730950488016887242097;
	const array<array<int,2>,8> neighbours =
	{{ {-1,0}, /* W */ {1,0}, /* E */
	   {0,-1}, /* N */ {0,1}, /* S */
	   {-1,-1},/* NW */{1,-1}, /* NE */
	   {1,1}, /* SE */{-1,1} /* SW */
	 }};
	myMap.clear();
	//4000 looks nicer when printed, but any high value (INF) works:
	myMap.resize( img.getWidth(), vector<myValType>( img.getHeight() , 4000));
	//myMp.resize( img->getWidth(), vector<myValType>( img->getHeight() , numeric_limits<myValType>::max()));
	vector<vector<bool> > visited(img.getWidth(),vector<bool>(img.getHeight(),false));
	priority_queue<edgeType,deque<edgeType>,edgeComp > q;
	for(auto i : goals) {
		(myMap[i.cx()])[i.cy()] = WAVE_VAL_GOAL;
		q.emplace(i,(myMap[i.cx()])[i.cy()]);
	}
	while(!q.empty()) {
		edgeType cur = q.top();
		q.pop();
		visited[cur.first.cx()][cur.first.cy()]=true;
		for(auto n : neighbours) {
			pos_t w = cur.first+pos_t(n.at(0),n.at(1));
			//If neighbour is within image borders:
			if( img.isInMap(w) ) {
				//if the neighbour is an unvisited non-obstacle:
				if( ( !WSPACE_IS_OBSTACLE( img.const_coordVal(w) ) )
						&& ( !( (visited[w.cx()])[w.cy()] ) )
						&& ((myMap[w.cx()])[w.cy()] != WAVE_VAL_GOAL)
						) {
					if(
							((myMap[cur.first.cx()])[cur.first.cy()]
							 +(((n.at(0)==0) || (n.at(1)==0))?1.0:sqrt2))
							<((myMap[w.cx()])[w.cy()])
							) {
						//increment:
						(myMap[w.cx()])[w.cy()]
								= (myMap[cur.first.cx()])[cur.first.cy()]
								+(((n.at(0)==0) || (n.at(1)==0))?1.0:sqrt2);
						//Add it to the wavefront
						q.emplace(w,(myMap[w.cx()])[w.cy()]);
					}
				}
			}
		}
	}
}

norm2BrushfireMap::norm2BrushfireMap(const pixelshadeMap& img) {
	list<pos_t> borders = this->findObstacleBorders(img);
	wave(img,borders);
}


//--------Functions for norm2BrushfireMap end ------------/
