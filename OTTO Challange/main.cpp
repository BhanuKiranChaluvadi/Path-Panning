#include "main.h"
#include "mainFunctions.h"

int main()
{
	//int start_s = clock();
	//Open input- output file
	ifstream inFile("input.txt");
	fstream outfile("output.txt", ios::out | ios::trunc);
	assert(inFile.is_open());
	assert(outfile.is_open());

	//read number of waypoints for the first iteraton
	int no_wayPoints;
	inFile >> no_wayPoints;

	while(true){
		if (no_wayPoints == 0) {
			inFile.close();
			outfile.close();
			 break;
		}
		// This vector is for maintaining waypoints in sequence and can find the possible neighbours.
		// possible neighbours for a given waypoint are all the waypoints coming after the current waypoint
		vector<waypoint>wayPoints;

		// maintains all the attributes of waypoint like location(x,y), cost and position (sequence number).
		// this sequence of input number helps to find the possible neighbours from the vector<Locaion>.
		// unordered_map - helps to find the key(Locations) in constant time.
		unordered_map<Location, cost_pos> nodes;
		
		// read waypoints from input files and assign them to nodes and waypoints
		readInputs(inFile, no_wayPoints, nodes, wayPoints);

		// perforn a* algorithm on the given inputs from start to goal.
		Location start{ 0, 0 };
		Location goal{ 100, 100 };
		unordered_map<Location, Location> parent;
		unordered_map<Location, double> cost_so_far;
		aStar_search(nodes, wayPoints,
					start, goal,
					parent, cost_so_far);

		// shortest path
		vector<Location> path = reconstruct_path(start, goal, parent);

		double final_cost = cost_so_far.find(goal)->second;

		outfile << fixed << setprecision(3)<< final_cost << endl;

		// read number of waypoints for next iteration
		inFile >> no_wayPoints;
	}
	//int stop_s = clock();
	//cout << "time: " << (stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 << endl;
	return 0;
}