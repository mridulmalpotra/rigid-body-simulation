#ifndef RIGIDBODY_H
#define RIGIDBODY_H


#include <cstdio>
#include <vector>

#include "vertex.h"
#include "triple.h"

using namespace std;

class RigidBody {
public:
    /* Constant parameters */
    int rigidBodyID;

    /* Vertex array */
    vector<Vertex> vertices;
    long int numVertices;

    /* Adjacency matrix for edges */
    int **edges;
    long int numEdges;

    /* Using unordered_map for plane with triple<int> for storing verticeIDs. */
    vector<triple<int>> faces;
    long int numFaces;

    /* Constant parameters of rigid body*/
    double rbMass;

    /* Linear parameters of rigid body. */
    triple<double> rbX, rbP, rbV, rbF;

    /* Constructor and other utilities. */
    RigidBody(int numVertices);
    void initVertices(int numVertices);
    void initEdges(int numVertices);
    void initFaces();
    void initRBParameters();

    void update(double t);
    void setRBParameters();
    void printParameters();

};

#endif // RIGIDBODY_H
