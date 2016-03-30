

#include <iostream>

#include "rigidbody.h"

RigidBody::RigidBody(int numV)
{
    rigidBodyID = -1;
    rbMass = 0.0;
    numVertices = numV;
    initVertices(numVertices);
    cout << "1. Created vertices" << endl;
    initEdges(numVertices);
    cout << "2. Created edges" << endl;
    initFaces();
    cout << "3. Created faces" << endl;
    initRBParameters();
}

void RigidBody::initVertices(int numVertices)
{
    for (int i = 0; i < numVertices; ++i) {
        Vertex temp;
        vertices.push_back(temp);
    }
}

void RigidBody::initEdges(int numVertices)
{
    edges = new int*[numVertices];
    for (int i = 0; i < numVertices; ++i) {
        edges[i] = new int[numVertices];
    }
    for (int i = 0; i < numVertices; ++i) {
        for (int j = 0; j < numVertices; ++j) {
            edges[i][j] = 0;
        }
    }

}

void RigidBody::initFaces()
{
    // no initialization needed as unordered_map already declared
}

void RigidBody::initRBParameters()
{
    // Calculating mass.
    rbMass = 0.0f;
    for (long int i = 0; i < numVertices; ++i)
        rbMass += vertices[i].mass;

    // Calculating position of rigid body.

    // Calculating velocity of rigid body.
    rbV = triple<double>(0.0f, 0.0f, 0.0f);
    for (long int i = 0; i < numVertices; ++i)
        rbV = rbV.add(vertices[i].v);

    // Calculating momentum of rigid body.
    rbP = rbV.multiply(rbMass);

    // Force is initially equal to force provided by the force field.
    rbF = triple<double>(0.0f, 0.0f, 0.0f); //Replace null vector by force field.
}

void RigidBody::update(double t)
{
    triple<double> tmpPos;
    // TODO: Calculate new parameter values for each vertex.
    for (long int i = 0; i < numVertices; ++i)
    {
        // Position
        tmpPos = vertices[i].F.multiply(0.5*t*t/vertices[i].mass);
        vertices[i].x  = vertices[i].u.multiply(t);
        vertices[i].x = vertices[i].x.add(tmpPos);

        // Velocity
        vertices[i].v = vertices[i].u.add(vertices[i].F.multiply(t/vertices[i].mass));

        // Momentum
        vertices[i].P = vertices[i].v.multiply(vertices[i].mass);

        // TODO: Force (Non-constant)

    }

    // Update rigid body parameter values using parameters of each vertex.
    setRBParameters();
}

void RigidBody::setRBParameters()
{
    // Calculating position of rigid body.

    // Calculating velocity of rigid body.
    rbV = triple<double>(0.0f, 0.0f, 0.0f);
    for (long int i = 0; i < numVertices; ++i)
        rbV = rbV.add(vertices[i].v);

    // Calculating momentum of rigid body.
    rbP = rbV.multiply(rbMass);

    // Calculating force on rigid body.
    rbF = triple<double>(0.0f, 0.0f, 0.0f);
    for (long int i = 0; i < numVertices; ++i)
        rbF = rbF.add(vertices[i].F);
}

void RigidBody::printParameters()
{
    printf("Mass: %lf kg\n", this->rbMass);
    printf("Position: ( %lf, %lf, %lf) m\n", this->rbX.X(), this->rbX.Y(), this->rbX.Z());
    printf("Velocity: ( %lf, %lf, %lf) m/s\n", this->rbV.X(), this->rbV.Y(), this->rbV.Z());
}
