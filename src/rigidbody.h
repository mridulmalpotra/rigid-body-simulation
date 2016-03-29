#ifndef RIGIDBODY_H
#define RIGIDBODY_H


#include <cstdio>
#include <unordered_map>

#include "vertex.h"
#include "triple.h"

using namespace std;

class RigidBody {
public:
    /* Vertex array */
    Vertex vertices[];
    long int numVertices;

    /* Adjacency matrix for edges */
    int *edges[];

    /* Using unordered_map for plane with triple<int> for storing verticeIDs. */
    unordered_map<int, triple<int> > faces;

    /* Constant parameters of rigid body*/
    double rbMass;

    /* Linear parameters of rigid body. */
    triple<double> rbX, rbP, rbV, rbF;

    /* Constructor and other utilities. */
    RigidBody();
    void initVetices();
    void initEdges();
    void initFaces();
    void initRBParameters();

    void update(double t);
    void setRBParameters();
    void printParameters();

};

#endif // RIGIDBODY_H
