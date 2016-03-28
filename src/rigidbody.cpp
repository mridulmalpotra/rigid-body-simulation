#include "rigidbody.h"

RigidBody::RigidBody()
{
    initVetices();
    initEdges();
    initFaces();
}

void RigidBody::initVetices()
{

}

void RigidBody::initEdges()
{

}

void RigidBody::initFaces()
{

}

void RigidBody::update(double currTime)
{
    // TODO: Calculate new parameter values for each vertex.

    // Update rigid body parameter values using parameters of each vertex.
    setRBParameters();
}

void RigidBody::setRBParameters()
{

}

void RigidBody::printParameters()
{
    printf("Mass: %lf kg\n", this->rbMass);
    printf("Position: ( %lf, %lf, %lf) m\n", this->rbX.X(), this->rbX.Y(), this->rbX.Z());
    printf("Velocity: ( %lf, %lf, %lf) m/s\n", this->rbV.X(), this->rbV.Y(), this->rbV.Z());
}
