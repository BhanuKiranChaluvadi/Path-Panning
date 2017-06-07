#pragma once
#include "main.h"

struct waypoint {
	public:
		Location location;
		double cost;
		waypoint(Location locationIn, double costIn) : location(locationIn), cost(costIn){};

};

// hash function
namespace std {
	template <>
	struct hash<tuple<int, int> > {
		inline size_t operator()(const tuple<int, int>& location) const {
			int x, y;
			tie(x, y) = location;
			return x * 100 + y;
		}
	};
}

// fills nodesIn and waypointsIn from the inputs
void readInputs(ifstream& inFile, int no_wayPointsIn, unordered_map<Location, cost_pos>& nodesIn, vector<waypoint>& wayPointsIn)
{
	// add start node(0, 0);
	nodesIn.insert({ { 0,0 },{ 0, 0} });
	wayPointsIn.push_back(waypoint({ 0, 0 }, 0));

	// add way points
	for (int i = 0; i < no_wayPointsIn; i++) {
		int xPos, yPos;
		double cost;
		inFile >> xPos >> yPos >> cost;
		nodesIn.insert({ { xPos , yPos}, {cost, i+1}});
		wayPointsIn.push_back(waypoint({ xPos , yPos }, cost));
	}
	// add the goal node(100, 100) - huge cost to cross goal
	nodesIn.insert({ { 100, 100 },{ 0, wayPointsIn.size()} });
	wayPointsIn.push_back(waypoint({100, 100}, 0));
}

// wrapper around priority queue
template<typename T, typename priority_t>
struct PriorityQueue {
	typedef pair<priority_t, T> PQElement;
	priority_queue<PQElement, vector<PQElement>, std::greater<PQElement>> elements;

	inline bool empty() const { return elements.empty(); }

	inline void put(T item, priority_t priority) {
		elements.emplace(priority, item);
	}

	inline T get() {
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};

// cost incurrend while travelling i.e., time taken for travelling from one location to another location
// cost = time = distance/speed
double distance_cost(Location point1, Location point2) {
	const int speed = 2;
	int x1, y1, x2, y2;
	tie(x1, y1) = point1;
	tie(x2, y2) = point2;
	double del_x = (double)(fabs(x1-x2));
	double del_y = (double)(fabs(y1-y2));
	double distance = sqrt(pow(del_x, 2) + pow(del_y, 2));
	return (distance/speed);
}

// Heuristics
double heuristic(Location locationIn) {
	Location goal = { 100, 100 };
	return distance_cost(locationIn, goal);
}

int max_index(int current_index, vector<waypoint>& wayPointsIn) {
	// 151.4213 + safety factor = 200
	int size = wayPointsIn.size();
	int max_index= current_index+1;
	
	double max_cost_can_accumulate = 200;
	double cost_accumulate = 0;

	while((max_index) < size-1){
		cost_accumulate += wayPointsIn[max_index].cost;
		if (cost_accumulate < max_cost_can_accumulate) {
			max_index++;
		}
		else
			return max_index;
	}
	return max_index;
}

// As of now its just dijkstra_search
void aStar_search(unordered_map<Location, cost_pos>& nodesIn,
					vector<waypoint>& wayPointsIn,
					Location start,  Location goal, 
					unordered_map<Location, Location>& parent,
					unordered_map<Location, double>& cost_so_far) {

	PriorityQueue <Location, double> frontier;
	frontier.put(start, 0);

	parent[start] = start;
	cost_so_far[start] = 0;

	while (!frontier.empty()) {
		auto current = frontier.get();

		if (current == goal)
			break;

		// find neighbors - neighbors are obtained from vector<Locations>
		// By finding the neighbor in the map<Location, cost_pos> we can get total information of the location/node.
		double missing_cost;
		int pos;		// only needed
		tie(missing_cost, pos)= nodesIn.find(current)->second;
		double nodes_missing_cost = 0; // accumulating cost because of missing nodes/locations

		// maximum bearable cost is 151.42 (math)considering factor of safety of factor = 200
		int maximum_index = max_index(pos, wayPointsIn);

		for (int index = pos + 1; index <= maximum_index; index++) {

			Location next = wayPointsIn[index].location;

			double neighbor_missing_cost;
			int neightbor_pos;
			tie(neighbor_missing_cost, neightbor_pos) = nodesIn.find(next)->second;


			double moving_cost = distance_cost(current, next); // moving time between nodes/locations
			double loading_cost = 10;  //waiting time at each waypoint
			double total_cost = moving_cost + loading_cost + nodes_missing_cost;

			double new_cost = cost_so_far[current] + total_cost;
			
			// accumulating cost if you keep on missing locatons.nodes
			nodes_missing_cost = nodes_missing_cost + neighbor_missing_cost;

			if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
				cost_so_far[next] = new_cost;
				double priority_cost = new_cost + heuristic(next); 
				parent[next] = current;
				frontier.put(next, priority_cost);
			}
		}
	}
}

// building a path from the goal to start and reversing it.
vector<Location> reconstruct_path(
	Location start,
	Location goal,
	unordered_map<Location, Location>& came_from )
{
	vector<Location> path;
	Location current = goal;
	path.push_back(current);
	while (current != start) {
		current = came_from[current];
		path.push_back(current);
	}
	std::reverse(path.begin(), path.end());
	return path;
}