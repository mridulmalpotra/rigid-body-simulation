#ifndef VERTEX_H
#define VERTEX_H

#include "triple.h"

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


    /* Constructor and other utilities. */
    Vertex();

};

#endif // VERTEX_H
