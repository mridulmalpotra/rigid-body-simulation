#ifndef RIGIDBODY_H
#define RIGIDBODY_H


#include <unordered_map>
#include "vertex.h"
#include "triple.h"

using namespace std;

class RigidBody {
public:
    /* Vertex array */
    Vertex vertices[];

    /* Adjacency matrix for edges */
    int *edges[];

    /* Using unordered_map for plane with triple<int> for storing verticeIDs. */
    unordered_map<int, triple<int>> faces;

    /**
     *  Constructor creations to be done in subsequent commits.
     */
};

#endif // RIGIDBODY_H
