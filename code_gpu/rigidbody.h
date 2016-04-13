#ifndef RIGIDBODY_H
#define RIGIDBODY_H


#include <cstdio>
#include <vector>
#include <utility>

#include "vertex.h"
#include "triple.h"

using namespace std;

class RigidBody {
protected:
    /* Initialization and setting. */
    void initVertices();
    void initEdges();
    void initFaces();

    void setRBParameters();
    void changeParameters(RigidBody rbody, triple<double> v1, triple<double> v2);

public:
    /* Vertex array */
    vector<Vertex> vertices;
    long int numVertices;

    /* Adjacency matrix for edges */
    int **edges;
    long int numEdges;

    /* Using vector for plane with triple<int> for storing verticeIDs. */
    vector< triple<int> > faces;
    long int numFaces;

    /* Constant parameters of rigid body*/
    int rigidBodyID;
    double rbMass;

    /* Linear parameters of rigid body. */
    triple<double> rbX, rbP, rbV, rbF;
    pair<double, double> boundX, boundY, boundZ;

    /* Last time when rigid body had collision. */
    double tlcol;

    /* Constructor and other utilities. */
    RigidBody(long int numV);
    void initRBParameters();

    void update(double t);
    void printParameters();
    void simulateIntersection(RigidBody rbody, double colT);

};

#endif // RIGIDBODY_H
