/*
 ==============================================================================
 Name        : RigidBodySimulation.cpp
 Author      : Mridul & Srinidhi
 Version     :
 Copyright   : Copyleft
 Description : Serial implementation of Rigid Body Dynamics on CPU using CPP
 ==============================================================================
 */

#include <unordered_map>

using namespace std;

template <typename T>
struct triple {
	T x;
	T y;
	T z;
};

class RigidBody {
public:
	/* Vertex array */
	Vertex vertices[];

	/* Adjacency matrix for edges */
	int edges[][];

	/* Using unordered_map for plane with triple<int> for storing verticeIDs. */
	unordered_map<int, triple<int>> faces;

	/**
	 *  Constructor creations to be done in subsequent commits.
	 */
};

class Vertex {
public:
	/* Constant quantities */
	int vertexID;			// Vertex ID
	double mass;			// Mass

	/* State variables */
	triple<double> x;		// Coordinates
	triple<double> P;		// Linear momentum

	/* Derived quantities */
	triple<double> v;		// Velocity

	/* Computed quantities*/
	triple<double> F;		// Force

};
