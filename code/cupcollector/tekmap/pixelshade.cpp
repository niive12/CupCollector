/** @file
* @author Lukas, Michael, Mikkel, Nikolaj & Mikael
* @date 21-10-2014
*
* Group project.
*/
#include "pixelshade.h"
#include <vector>
#include <unordered_map>
#include <queue>

using namespace std;
using namespace rw::sensor;

pixelshadeMap::pixelshadeMap(shared_ptr< Image > img)
	:tekMap(img)
{
	myMap.clear();
	myMap.resize( img->getWidth(), vector<myValType>( img->getHeight() ));
	//Just give pixel values to the map and be done with it.
	for( size_t x = 0; x < img->getWidth(); ++x)
	{
		for( size_t y = 0; y < img->getHeight(); ++y)
			( myMap.at(x) ).at(y) = (myValType)( img->getPixelValuei(x,y,0) );
		( myMap.at(x) ).shrink_to_fit();
	}
}


list<pos_t> pixelshadeMap::getWavefrontPath(const pos_t &from, const pos_t &to) const
{
	list<pos_t> resulting_path;
	bool thereIsAPath = false;
	const array<array<int,2>,8> neighbours =
	{{ {-1,0}, /* W */ {1,0}, /* E */
	   {0,-1}, /* N */ {0,1}, /* S */
	   {-1,-1},/* NW */{1,-1}, /* NE */
	   {1,1}, /* SE */{-1,1} /* SW */
	 }} ;
	vector<vector<bool> > visited(getWidth(),vector<bool>(getHeight(),false));
	unordered_map<pos_t,pos_t> parent;
	queue<pos_t> q; //FIFO
	q.push(to);
	while(!q.empty())
	{
		pos_t cur = q.front();
		if(cur==from) {
			thereIsAPath=true;
			break;
		}
		q.pop();
		for(auto n : neighbours) {
			pos_t w = cur+pos_t(n.at(0),n.at(1));
			//If neighbour is within image borders:
			if( isInMap(w) ) {
				//if the neighbour is an unvisited non-obstacle:
				if( ( !WSPACE_IS_OBSTACLE( const_coordVal(w)))
						&& ( !( (visited[w.cx()])[w.cy()] ) )
						&& (w!=to)
						)
				{
					parent[w]=cur;
					(visited[w.cx()])[w.cy()]=true;
					//Add it to the wavefront
					q.push(w);
				}
			}
		}
		(visited[cur.cx()])[cur.cy()]=true;
	}
	if(thereIsAPath) {
		pos_t p=from;
		while(p!=to)
		{
			p=parent[p];
			resulting_path.push_back(p);
		}
	}
	return move(resulting_path);
}

list<pos_t> pixelshadeMap::getDijkstraPath(const pos_t &from, const pos_t &to) const
{
	const double sqrt2 =1.4142135623730950488016887242097;
	list<pos_t> resulting_path;
	bool thereIsAPath = false;
	const array<array<int,2>,8> neighbours =
	{{ {-1,0}, /* W */ {1,0}, /* E */
	   {0,-1}, /* N */ {0,1}, /* S */
	   {-1,-1},/* NW */{1,-1}, /* NE */
	   {1,1}, /* SE */{-1,1} /* SW */
	 }} ;
	unordered_map<pos_t,pair<edgeType,bool> > parents;
	auto visited = [&parents](const pos_t &in)->bool& {
		try { return ((parents.at(in)).second); }
		catch (out_of_range &ex) {
			(void)(ex);
			(parents[in].second)=false;
			return ((parents.at(in)).second);
		}
	};
	unordered_set<pos_t> paths;
	auto path = [&parents, &paths](const pos_t &in)->pair<pos_t*,bool> {
		return make_pair(&((parents[in].first).first),!(paths.insert(in).second)); };
	auto pathCost = [&parents](const pos_t &in)->edgeType::second_type& {
		return ((parents[in].first).second); };
	auto parentEdge = [&parents](const pos_t &child)->edgeType& {
		return parents[child].first; };
	auto eCost = [](const edgeType &v)->const edgeType::second_type& {return (v.second); };
	auto eVert = [](const edgeType &v)->const pos_t& {return (v.first); };

	priority_queue<edgeType,deque<edgeType>,edgeComp > q;
	q.emplace(to,edgeType::second_type(WAVE_VAL_GOAL));
	while(!q.empty())
	{
		edgeType cur = q.top();
		q.pop();
		if(eVert(cur)==from) {
			thereIsAPath=true;
			break;
		}
		visited(eVert(cur))=true;
		for(auto n : neighbours) {
			pos_t w = pos_t(eVert(cur).cx()+n.at(0), eVert(cur).cy()+n.at(1));
			//If neighbour is within image borders:
			if( isInMap(w) ) {
				//if the neighbour is an unvisited non-obstacle:
				if( ( !WSPACE_IS_OBSTACLE( const_coordVal(w)))
						&& (!visited(w))
						&& (w!=to)
						)
				{
					edgeType::second_type incr=(((n.at(0)==0) || (n.at(1)==0))?1.0:sqrt2);
					if(
							(!(path(w).second))
							||
							( pathCost(w) > (eCost(cur)+incr) )
							) {
						parentEdge(w)=edgeType(eVert(cur),eCost(cur)+incr);
						//Add it to q
						q.emplace(w, eCost(cur)+incr);
					}
				}
			}
		}
	}
	if(thereIsAPath) {
		for(pos_t i=from; i!=to; i=*(path(i).first))
			resulting_path.push_front(i);
	}
	return move(resulting_path);
}

unordered_set< pos_t > pixelshadeMap::findFreespace(const pos_t &withinFreeSpace) const {
	unordered_set<pos_t> resulting_coords;
	if(!isInMap(withinFreeSpace))
		cerr << "coord given to findFreespace is not in pixmap." << endl;
	else {
		const array<array<int,2>,8> neighbours =
		{{ {-1,0}, /* W */ {1,0}, /* E */
		   {0,-1}, /* N */ {0,1}, /* S */
		   {-1,-1},/* NW */{1,-1}, /* NE */
		   {1,1}, /* SE */{-1,1} /* SW */
		 }};
		vector<vector<bool> > visited(getWidth(),vector<bool>(getHeight(),false));
		queue<pos_t> q; //FIFO
		q.push(withinFreeSpace);
		while(!q.empty()) {
			pos_t cur = q.front();
			q.pop();
			resulting_coords.insert(cur);
			for(auto n : neighbours) {
				pos_t w = cur+pos_t(n.at(0),n.at(1));
				//If neighbour is within image borders:
				if( isInMap(w)) {
					//if the neighbour is an unvisited non-obstacle:
					if( (!WSPACE_IS_OBSTACLE( const_coordVal(w) ))
							&& (!( (visited[w.cx()])[w.cy()] ) ) ) {
						//Add it to the wavefront
						q.push(w);
						(visited[w.cx()])[w.cy()]=true;
					}
				}
			}
			(visited[cur.cx()])[cur.cy()] = true;
		}
	}
	return move(resulting_coords);
}


list<pos_t> pixelshadeMap::getReachables(const unordered_set<pos_t> &coordsToSearch, const pos_t &center) const
{
	list<pos_t> result;
	if(coordsToSearch.find(center)!=coordsToSearch.end()) {
		const array<array<int,2>,8> neighbours =
		{{ {-1,0}, /* W */ {1,0}, /* E */
		   {0,-1}, /* N */ {0,1}, /* S */
		   {-1,-1},/* NW */{1,-1}, /* NE */
		   {1,1}, /* SE */{-1,1} /* SW */
		 }} ;
		unordered_set<pos_t> visited;
		visited.insert(center);
		queue<pos_t> q;
		q.push(center);
		while(!q.empty()) {
			pos_t v = q.front();
			q.pop();
			result.push_back(v);
			for(auto n : neighbours) {
				pos_t w = v+pos_t(n.at(0),n.at(1));
				if(isInMap(w))
					if( (visited.insert(w).second) && (!(WSPACE_IS_OBSTACLE(const_coordVal(w)))) )
						if( (coordsToSearch.find(w)!=coordsToSearch.end()) )
							q.push(w);
			}
		}
	}
	return move(result);
}
