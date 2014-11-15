/** @file
* @author Lukas, Michael, Mikkel, Nikolaj & Mikael
* @date 21-10-2014
*
* Group project.
*/
#include "wavefront.h"



wavefrontMap::wavefrontMap(shared_ptr< Image > img, const list<pos_t> &coords)
	:tekMap(img)
{
	if(coords.empty())
		cerr << "No coords passed to wavefrontMap constructor!" << endl;
	wave(img,coords);
}



dijkstraMap::dijkstraMap(shared_ptr< Image > img, const list<pos_t> &coords)
	:tekMap(img)
{
	if(coords.empty())
		cerr << "No coords passed to dijkstraMap constructor!" << endl;
	wave(img,coords);
}
/**
* @brief wave Stores all Dijkstra's algorithm values in myMap.
* @param img Image pointer
* @param goals set of goal coordinates
*/
void dijkstraMap::wave(shared_ptr<Image> img, const list<pos_t> &goals)
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
	myMap.resize( img->getWidth(), vector<myValType>( img->getHeight() , 4000));
	//myMp.resize( img->getWidth(), vector<myValType>( img->getHeight() , numeric_limits<myValType>::max()));
	vector<vector<bool> > visited(img->getWidth(),vector<bool>(img->getHeight(),false));
	priority_queue<edgeType,deque<edgeType>,edgeComp > q;
	for(auto i : goals) {
		(myMap[i.cx()])[i.cy()] = WAVE_VAL_GOAL;
		q.emplace(i,(myMap[i.cx()])[i.cy()]);
	}
	while(!q.empty())
	{
		edgeType cur = q.top();
		q.pop();
		visited[cur.first.cx()][cur.first.cy()]=true;
		for(auto n : neighbours) {
			pos_t w = cur.first+pos_t(n.at(0),n.at(1));
			//If neighbour is within image borders:
			if( isInImage( img, w ) ) {
				//if the neighbour is an unvisited non-obstacle:
				if( ( !WSPACE_IS_OBSTACLE( img->getPixelValuei(w.cx(),w.cy(),0) ) )
						&& ( !( (visited[w.cx()])[w.cy()] ) )
						&& ((myMap[w.cx()])[w.cy()] != WAVE_VAL_GOAL)
						)
				{
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

list<pos_t> dijkstraMap::getShortestPath(const pos_t &from) {
	list<pos_t> resultPath;

	const array<array<int,2>,8> neighbours =
	{{ {-1,0}, /* W */ {1,0}, /* E */
	   {0,-1}, /* N */ {0,1}, /* S */
	   {-1,-1},/* NW */{1,-1}, /* NE */
	   {1,1}, /* SE */{-1,1} /* SW */
	 }};

	pos_t nextPos=from;
	while(const_coordVal(nextPos)>(myValType(WAVE_VAL_GOAL)+0.1)) {
		set<edgeType,edgeComp> nebs;
		for(auto n : neighbours) {
			pos_t w = nextPos+pos_t(n.at(0),n.at(1));
			//If neighbour is within image borders:
			if( isInMap(w) ) {
				nebs.emplace(w,const_coordVal(w));
			}
		}
		nextPos=(*(nebs.rbegin())).first; //The one closest to goal.
		resultPath.push_back(nextPos);
	}
	return move(resultPath);
}
