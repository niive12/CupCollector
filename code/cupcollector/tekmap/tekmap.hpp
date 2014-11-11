/** @file
* @author Lukas, Michael, Mikkel, Nikolaj & Mikael
* @date 21-10-2014
*
* Group project.
*/
#pragma once
#include "../libraries/Image.hpp"
#include "../assignment.h"
#include <memory>
#include <vector>
#include <array>
#include <utility>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <queue>
#include <iostream>
#include <limits>
using namespace std;
using namespace rw::sensor;
using coordIndexType = long int;
struct pos_t : pair<coordIndexType, coordIndexType> {
		using x_type = first_type;
		using y_type = second_type;
		/** @brief to get reference to x-coordinate of pos_t coord, do coord.x(). */
		x_type &x() {return first;}
		y_type &y() {return second;}
		/** @brief to get const-reference to x-coordinate of pos_t coord, do coord.cx(). */
		x_type &cx() const {return const_cast<first_type&>(first);}
		y_type &cy() const {return const_cast<second_type&>(second);}
		/** Lvalue constructor, calls base class constructor */
		pos_t(const x_type &inx, const y_type &iny):pair(inx,iny) {}
		/** Rvalue constructor, calls base class constructor */
		pos_t(x_type &&inx, y_type &&iny):pair(move(inx),move(iny)) {}
		/** Default constructor, calls base class constructor */
		pos_t():pair() {}
		/** Copy constructor, calls base class constructor */
		pos_t(const pair<x_type,y_type> &in):pair(in) {}
		/** Move constructor, calls base class constructor */
		pos_t(pair<x_type,y_type> &&in):pair(move(in)) {}
		pos_t make_pos(const x_type &inx, const y_type &iny) {return make_pair(inx,iny);}
		pos_t make_pos(x_type &&inx, y_type &&iny) {return make_pair(move(inx),move(iny));}
		friend const pos_t operator+(const pos_t& a, const pos_t& b)
		{ return move(pos_t(a.cx()+b.cx(),a.cy()+b.cy())); }
		friend const pos_t operator+(pos_t&& a, pos_t&& b)
		{ return move(pos_t(a.cx()+b.cx(),a.cy()+b.cy())); }
		friend const pos_t operator-(const pos_t& a, const pos_t& b)
		{ return move(pos_t(a.cx()-b.cx(),a.cy()-b.cy())); }
		friend const pos_t operator-(pos_t&& a, pos_t&& b)
		{ return move(pos_t(a.cx()-b.cx(),a.cy()-b.cy())); }
} ;
namespace std {
template <>
/** @brief The hash<pos_t> struct is for pos_t used as key in unordered map/set. */
struct hash<pos_t> {
		/**
	 * @brief operator () Cantor's pairing function.
	 * @param k They pos_t key.
	 * @return Unique hash value.
	 */
		size_t operator()(const pos_t& k) const {
				return (((size_t)(k.cx()+k.cy()))*((size_t)(k.cx()+k.cy()+1)))/2+(size_t)(k.cy());
		}
};
} //namespace std

/** @brief coordValType The value type for each pixel */
template<typename coordValType=long int>
/**
 * @brief The tekMap class
 * The parent class.
 */
class tekMap
{
public:
		using myValType = coordValType;
		/** @brief edgeType Pairs a position vertex with a cost. Used in Dijkstra. */
		using edgeType = pair<pos_t,double>;
		/** @brief The edgeComp struct Compares two edges. Used in priority queues. */
		struct edgeComp {
				bool operator() ( const edgeType &lhs, const edgeType &rhs) const
				{ return (lhs.second)>(rhs.second); }
		};
		/**
	* @brief Convert map to pgm to visualize wave functionality.
	* @param img. The image that I load and save to.
	* @param fileName. Name of the output file.
	*/
		void convert_into_pgm( shared_ptr<Image> img, const std::string& fileName )
		{
				unsigned char val;
				for( coordIndexType x = 0; x < img->getWidth(); ++x){
						for ( coordIndexType y = 0; y < img->getHeight(); ++y){
								// val = ( myMap.at(x) ).at(y) % 255;
								val = (unsigned char) (const_coordVal(x, y) %255 );
								img->setPixel8U(x,y,val);
						}
				}
				img->saveAsPGM( fileName );
		}
		/**
	 * @brief tekMap Constructor doing almost nothing.
	 * @param img Image to check.
	 */
		tekMap(shared_ptr<Image> img) {
				if( !img )
						cerr << "No image passed to map ctor" << endl;
		}

		/**
	 * @brief coordVal Returns a reference to the coordinate value.
	 * coordVal can take a pos_t or a x and y coord.
	 * The value can be written to, eg. coordVal(my_pos)=500;
	 *
	 * const_coordVal Returns a const-reference to the coordinate value.
	 * This value can only be read, eg. myValType val = const_coordVal(my_pos);
	 */
		inline coordValType &const_coordVal(const pos_t::x_type &x, const pos_t::y_type &y) const
		{ return const_cast<coordValType&>(((myMap.at(x)).at(y))); }
		inline coordValType &const_coordVal(pos_t::x_type &&x, pos_t::y_type &&y) const
		{ return const_cast<coordValType&>(((myMap.at(move(x))).at(move(y)))); }
		inline coordValType &coordVal(const pos_t::x_type &x, const pos_t::y_type &y)
		{ return ((myMap.at(x)).at(y)); }
		inline coordValType &coordVal(pos_t::x_type &&x, pos_t::y_type &&y)
		{ return ((myMap.at(move(x))).at(move(y))); }
		inline coordValType &const_coordVal(const pos_t &ofThisCoord) const
		{ return const_coordVal(ofThisCoord.first,ofThisCoord.second); }
		inline coordValType &const_coordVal(pos_t &&ofThisCoord) const
		{ return const_coordVal(move(ofThisCoord.first),move(ofThisCoord.second)); }
		inline coordValType &coordVal(const pos_t &ofThisCoord)
		{ return coordVal(ofThisCoord.first,ofThisCoord.second); }
		inline coordValType &coordVal(pos_t &&ofThisCoord)
		{ return coordVal(move(ofThisCoord.first),move(ofThisCoord.second)); }
		/** @brief getHeight and getWidth return the cooresponding dimensions of the map. */
		inline size_t getHeight() const
		{ return (myMap.at(0)).size(); }
		inline size_t getWidth() const
		{ return myMap.size(); }
		/**
	 * @brief shade Shades an image according to the map's coordinate values.
	 * @param img Image to paint in.
	 */
		void shade(shared_ptr<Image> img)
		{
				coordValType mymin=numeric_limits<coordValType>::max();
				coordValType mymax=numeric_limits<coordValType>::min();
				for(unsigned int x=0; x<getWidth(); ++x)
						for(unsigned int y=0; y<getHeight(); ++y) {
								if(myMap[x][y]>mymax && myMap[x][y]!=WAVE_VAL_UNV)
										mymax=myMap[x][y];
								if(myMap[x][y]<mymin)
										mymin=myMap[x][y];
						}
				for(unsigned int x=0; x<getWidth(); ++x)
						for(unsigned int y=0; y<getHeight(); ++y) {
								img->setPixel8U(x,y,
												(unsigned char)(((myMap[x][y]==WAVE_VAL_UNV?(mymax-mymin):myMap[x][y])*255)/((mymax-mymin)))
												);
						}
		}

		/**
	 * @brief getOffloadingStations Searches image for offloading stations.
	 * @param img Image to search in.
	 * @return Coordinates of offloading stations.
	 */
		static list<pos_t> getOffloadingStations(const shared_ptr<Image> img) {
				list<pos_t> result;
				for(coordIndexType x=0; x<coordIndexType(img->getWidth()); ++x)
						for(coordIndexType y=0; y<coordIndexType(img->getHeight()); ++y)
								if(WSPACE_IS_OL_STATION(img->getPixelValuei(x,y,0)))
										result.emplace_back(x,y);
				return move(result);
		}

		/**
	 * @brief isInMap Bounds checking of a coordinate.
	 * @param x x-coordinate.
	 * @param y y-coordinate.
	 * @return True if coordinate is within map borders. False otherwise.
	 */
		inline bool isInMap(const coordIndexType &x, const coordIndexType &y) const {
				return ((((int)(getWidth())>x)&&((int)(getHeight())>y))&&((x>0)&&(y>0)));
		}
		inline bool isInMap(const pos_t &pos) const {
				return isInMap(pos.cx(),pos.cy());
		}

		/**
	 * @brief getWavefrontPath Finds the shortest path using normInf Wavefront algorithm.
	 * @param img Image to find path in.
	 * @param from Starting position.
	 * @param to Goal position.
	 * @return Path coordinates, excluding to and from.
	 */
		list<pos_t> getWavefrontPath(shared_ptr< Image > img, const pos_t &from, const pos_t &to) const;
		/**
	 * @brief getDijsktraPath Finds the shortest path using norm2 Dijsktra's algorithm.
	 * @param img Image to find path in.
	 * @param from Starting position.
	 * @param to Goal position.
	 * @return Path coordinates, excluding to and from.
	 */
		list<pos_t> getDijkstraPath(shared_ptr< Image > img, const pos_t &from, const pos_t &to) const;

protected:
		/** @brief The map (dynamic 2D array) */
		vector<vector<coordValType> > myMap;
		/**
	* @brief isInImage Checks if the pixel is inside the image
	* @param img The image
	* @param x The x value...
	* @param y The y value...
	* @return true if pixel is inside image.
	*/
		inline bool isInImage(const shared_ptr<Image> img, const coordIndexType &x,
							  const coordIndexType &y) const
		{
				return ((((coordIndexType)(img->getWidth()) > x) && ((coordIndexType)(img->getHeight()) > y))
						&&((x>0) && (y>0)) );
		}
		inline bool isInImage(const shared_ptr<Image> img, const pos_t &pos) const
		{ return isInImage(img,pos.cx(),pos.cy()); }

		/**
* @brief wave Stores all wavefront algorithm values in myMap.
* @param img Image pointer
* @param goals set of goal coordinates
*/
		virtual void wave(shared_ptr<Image> img, const list<pos_t> &goals);
		/**
* @brief findCoords Finds workspace (freespace) coordinates or relevant borders.
* @param img Image pointer
* @param withinFreeSpace A coordinate within the freespace, eg. a valid coordinate.
* @param borders If true, only coords bordering the freespace are returned,
* if false, only coords within the freespace are returned.
* @return Coords bordering or within the freespace, depending on the borders parameter.
*/
		virtual list< pos_t > findCoords(shared_ptr<Image> img, const pos_t &withinFreeSpace, const bool borders=true) const;
};
// -------------- Functions for tekmap start
template<typename coordValType>
void tekMap<coordValType>::wave(shared_ptr<Image> img, const list<pos_t> &goals)
{
		const array<array<int,2>,8> neighbours =
		{{ {-1,0}, /* W */ {1,0}, /* E */
		   {0,-1}, /* N */ {0,1}, /* S */
		   {-1,-1},/* NW */{1,-1}, /* NE */
		   {1,1}, /* SE */{-1,1} /* SW */
		 }} ;
		myMap.clear();
		myMap.resize( img->getWidth(), vector<coordValType>( img->getHeight() , WAVE_VAL_UNV));
		vector<vector<bool> > visited(img->getWidth(),vector<bool>(img->getHeight(),false));
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
						if( isInImage( img, w ) ) {
								//if the neighbour is an unvisited non-obstacle:
								if( ( !WSPACE_IS_OBSTACLE( img->getPixelValuei(w.cx(),w.cy(),0) ) )
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
template<typename coordValType>
list< pos_t > tekMap<coordValType>::findCoords(shared_ptr<Image> img, const pos_t &withinFreeSpace, const bool borders) const
{
		list<pos_t> resulting_coords;
		if(!isInImage(img,withinFreeSpace))
				cerr << "coord given to findCoords is not in image." << endl;
		else {
				const array<array<int,2>,8> neighbours =
				{{ {-1,0}, /* W */ {1,0}, /* E */
				   {0,-1}, /* N */ {0,1}, /* S */
				   {-1,-1},/* NW */{1,-1}, /* NE */
				   {1,1}, /* SE */{-1,1} /* SW */
				 }};
				vector<vector<bool> > visited(img->getWidth(),vector<bool>(img->getHeight(),false));
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
								if( isInImage( img, w ) ) {
										//if the neighbour is an unvisited non-obstacle:
										if( (!WSPACE_IS_OBSTACLE( img->getPixelValuei(w.cx(),w.cy(),0) ))
														&& (!( (visited[w.cx()])[w.cy()] ) ) ) {
												//Add it to the wavefront
												q.push(w);
												(visited[w.cx()])[w.cy()]=true;
										}
										if(borders&&WSPACE_IS_OBSTACLE( img->getPixelValuei(w.cx(),w.cy(),0) )){
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
template<typename coordValType>
list<pos_t> tekMap<coordValType>::getWavefrontPath(shared_ptr< Image > img, const pos_t &from, const pos_t &to) const
{
		list<pos_t> resulting_path;
		bool thereIsAPath = false;
		const array<array<int,2>,8> neighbours =
		{{ {-1,0}, /* W */ {1,0}, /* E */
		   {0,-1}, /* N */ {0,1}, /* S */
		   {-1,-1},/* NW */{1,-1}, /* NE */
		   {1,1}, /* SE */{-1,1} /* SW */
		 }} ;
		vector<vector<bool> > visited(img->getWidth(),vector<bool>(img->getHeight(),false));
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
						if( isInImage( img, w ) ) {
								//if the neighbour is an unvisited non-obstacle:
								if( ( !WSPACE_IS_OBSTACLE( img->getPixelValuei(w.cx(),w.cy(),0) ) )
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

template<typename coordValType>
list<pos_t> tekMap<coordValType>::getDijkstraPath(shared_ptr< Image > img, const pos_t &from, const pos_t &to) const
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
						if( isInImage(img, w) ) {
								//if the neighbour is an unvisited non-obstacle:
								if( ( !WSPACE_IS_OBSTACLE( img->getPixelValuei(w.cx(),w.cy(),0) ))
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

// -------------- Functions for tekmap end
class pixelshadeMap : public tekMap<unsigned char>
{
public:
		pixelshadeMap(shared_ptr< Image > img)
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

		list<pos_t> getWavefrontPath(const pos_t &from, const pos_t &to) const
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

		list<pos_t> getDijkstraPath(const pos_t &from, const pos_t &to) const
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

		virtual unordered_set< pos_t > findFreespace(const pos_t &withinFreeSpace) const {
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

};
class brushfireMap : public tekMap<unsigned char>
{
protected:
		virtual list< pos_t > findCoords(const pixelshadeMap &img, const pos_t &withinFreeSpace, const bool borders=true) const
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
		/**
* @brief findObstacleBorders returns all obstacle borders in image.
* @param img The image to find obstacle borders in.
* @return set of obstacle border coordinates.
* How it works:
* For all pixels in image:
* if pixel is an obstacle:
* if pixel has at least one freespace-neighbour:
* return_value.insert(posType(x,y));
* return return_value.
*
* So this is O(N).
*/
		virtual list< pos_t > findObstacleBorders(shared_ptr<Image> img) const
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
																resulting_coords.insert(w);
																break;
														}
												}
										}
								}
						}
				}
				return move(list<pos_t>(resulting_coords.begin(),resulting_coords.end()));
		}
		virtual list< pos_t > findObstacleBorders(const pixelshadeMap &img) const
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
																resulting_coords.insert(w);
																break;
														}
												}
										}
								}
						}
				}
				return move(list<pos_t>(resulting_coords.begin(),resulting_coords.end()));
		}
		/**
* @brief findObstacleBorders returns all RELEVANT obstacle borders in image.
* @param img The image to find relevant obstacle borders in.
* @param validFreeSpaceCoord A free space coordinate the robot can reach.
* @return set of relevant obstacle border coordinates.
* How it works:
* Do
* Propagate wave from validFreeSpaceCoord.
* If wave hits an obstacle pixel:
* return_value.insert(obstacle_pixel);
* until wave has visited all freespace.
* return return_value.
*
* So this is O(N) but performs better than the other findObstacleBorders.
*/
		virtual inline list< pos_t > findObstacleBorders(shared_ptr<Image> img, const pos_t &validFreeSpaceCoord) const
		{
				return move(findCoords(img,validFreeSpaceCoord,true));
		}
		virtual inline list< pos_t > findObstacleBorders(const pixelshadeMap &img, const pos_t &validFreeSpaceCoord) const
		{
				return move(findCoords(img,validFreeSpaceCoord,true));
		}
public:
		brushfireMap(shared_ptr< Image > img)
				:tekMap(img)
		{
				list<pos_t> coords;
				//Fill the coords set:
				coords = findObstacleBorders(img); //non-efficient
				wave(img,coords);
		}
		brushfireMap(shared_ptr< Image > img, const pos_t &reachableFreeSpace)
				:tekMap(img)
		{
				list<pos_t> coords;
				//Fill the coords set:
				coords = findObstacleBorders(img,reachableFreeSpace); //faster
				wave(img,coords);
		}
		brushfireMap(const pixelshadeMap &img)
				:tekMap(img)
		{
				list<pos_t> coords;
				//Fill the coords set:
				coords = findObstacleBorders(img); //non-efficient
				wave(img,coords);
		}
		brushfireMap(const pixelshadeMap &img, const pos_t &reachableFreeSpace)
				:tekMap(img)
		{
				list<pos_t> coords;
				//Fill the coords set:
				coords = findObstacleBorders(img,reachableFreeSpace); //faster
				wave(img,coords);
		}
		virtual void wave(const pixelshadeMap &img, const list<pos_t> &goals)
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

};
class wavefrontMap : public tekMap<long unsigned int>
{
public:
		wavefrontMap(shared_ptr< Image > img, const list<pos_t> &coords = list< pos_t >())
				:tekMap(img)
		{
				if(coords.empty())
						cerr << "No coords passed to wavefrontMap constructor!" << endl;
				wave(img,coords);
		}
};
using brushfire_map = brushfireMap;
using wavefront_map = wavefrontMap;
using pixelshade_map = pixelshadeMap;

/**
* @brief The dijkstraMap class Same as integer wavefront, except it's norm2 potential function.
*/
class dijkstraMap : public tekMap<double> {
public:
		dijkstraMap(shared_ptr< Image > img, const list<pos_t> &coords = list< pos_t >())
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
		virtual void wave(shared_ptr<Image> img, const list<pos_t> &goals) override
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
};
