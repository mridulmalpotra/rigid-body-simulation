#include "rigidbody.h"
#include <climits>

RigidBody::RigidBody(long int numV)
{
    tlcol = 0.0f;
    rigidBodyID = -1;
    rbMass = 0.0;
    numVertices = numV;

    initVertices();
    initEdges();
    initFaces();
}

void RigidBody::initVertices()
{
    for (int i = 0; i < numVertices; ++i) {
        Vertex temp;
        vertices.push_back(temp);
    }
}

void RigidBody::initEdges()
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
    triple<double> tmp;
    // Calculating mass.
    rbMass = 0.0f;
    for (long int i = 0; i < numVertices; ++i)
        rbMass += vertices[i].mass;

    // Calculating initial velocity and position of all vertices
    for (long int i = 0; i < numVertices; ++i)
    {
//        vertices[i].u = vertices[i].v.multiply(1);
        vertices[i].xi = vertices[i].x.multiply(1);
    }

    // Calculating position of rigid body.
    rbX = triple<double>(0.0f, 0.0f, 0.0f);
    for (long int i = 0; i < numVertices; ++i)
    {
        tmp = vertices[i].x.multiply(1);
        rbX = rbX.add(tmp.multiply(vertices[i].mass));
    }
    rbX = rbX.multiply(1/rbMass);

    // Calculating bounds of bounding box.
    boundX.first = LONG_MAX;
    boundX.second = LONG_MIN;
    for (long int i = 0; i < numVertices; ++i)
    {
        boundX.first = min(boundX.first, vertices[i].x.X());
        boundX.second = max(boundX.second, vertices[i].x.X());
    }

    boundY.first = LONG_MAX;
    boundY.second = LONG_MIN;
    for (long int i = 0; i < numVertices; ++i)
    {
        boundY.first = min(boundY.first, vertices[i].x.Y());
        boundY.second = max(boundY.second, vertices[i].x.Y());
    }

    boundZ.first = LONG_MAX;
    boundZ.second = LONG_MIN;
    for (long int i = 0; i < numVertices; ++i)
    {
        boundZ.first = min(boundZ.first, vertices[i].x.Z());
        boundZ.second = max(boundZ.second, vertices[i].x.Z());
    }

    // Calculating velocity of rigid body.
    rbV = triple<double>(0.0f, 0.0f, 0.0f);
    tmp = rbV.multiply(1);
    for (long int i = 0; i < numVertices; ++i)
    {
        tmp = vertices[i].v.multiply(1);
        rbV = rbV.add(tmp.multiply(vertices[i].mass));
    }
    rbV = rbV.multiply(1/rbMass);

    // Calculating momentum of rigid body
    tmp = rbV.multiply(1);
    rbP = tmp.multiply(rbMass);

    // Force is initially equal to force provided by the force field.
    rbF = triple<double>(0.0f, 0.0f, 0.0f); //Replace null vector by force field.
}

void RigidBody::update(double t)
{
    triple<double> tmpPos,tmpvel;
    // TODO: Calculate new parameter values for each vertex.
    for (long int i = 0; i < numVertices; ++i)
    {
        // Position
        tmpvel = vertices[i].u;
        tmpPos = vertices[i].F.multiply(0.5*(t-tlcol)*(t-tlcol)/vertices[i].mass);
        vertices[i].x  = tmpvel.multiply((t-tlcol));
        vertices[i].x = vertices[i].x.add(tmpPos);
        vertices[i].x = vertices[i].x.add(vertices[i].xi);

        // Velocity
        vertices[i].v = vertices[i].u/*.add(vertices[i].F.multiply((t-tlcol)/vertices[i].mass))*/;

        // Momentum
        tmpvel = vertices[i].v.multiply(1);
        vertices[i].P = tmpvel.multiply(vertices[i].mass);

        // TODO: Force (Non-constant)

    }

    // Update rigid body parameter values using parameters of each vertex.
    setRBParameters();
}

void RigidBody::setRBParameters()
{
    // Calculating position of rigid body.
    rbX = triple<double>(0.0f, 0.0f, 0.0f);
    for (long int i = 0; i < numVertices; ++i)
        rbX = rbX.add(vertices[i].x.multiply(vertices[i].mass));
    rbX = rbX.multiply(1/rbMass);

    // Calculating bounds of bounding box.
    boundX.first = LONG_MAX;
    boundX.second = LONG_MIN;
    for (long int i = 0; i < numVertices; ++i)
    {
        boundX.first = min(boundX.first, vertices[i].x.X());
        boundX.second = max(boundX.second, vertices[i].x.X());
    }

    boundY.first = LONG_MAX;
    boundY.second = LONG_MIN;
    for (long int i = 0; i < numVertices; ++i)
    {
        boundY.first = min(boundY.first, vertices[i].x.Y());
        boundY.second = max(boundY.second, vertices[i].x.Y());
    }

    boundZ.first = LONG_MAX;
    boundZ.second = LONG_MIN;
    for (long int i = 0; i < numVertices; ++i)
    {
        boundZ.first = min(boundZ.first, vertices[i].x.Z());
        boundZ.second = max(boundZ.second, vertices[i].x.Z());
    }

    // Calculating velocity of rigid body.
    rbV = triple<double>(0.0f, 0.0f, 0.0f);
    triple<double> tmp = rbV.multiply(1);
    for (long int i = 0; i < numVertices; ++i)
    {
        tmp = vertices[i].v.multiply(1);
        rbV = rbV.add(tmp.multiply(vertices[i].mass));
    }
    rbV = rbV.multiply(1/rbMass);

    // Calculating momentum of rigid body.
    tmp = rbV.multiply(1);
    rbP = tmp.multiply(rbMass);

    // Calculating force on rigid body.
    rbF = triple<double>(0.0f, 0.0f, 0.0f);
    for (long int i = 0; i < numVertices; ++i)
        rbF = rbF.add(vertices[i].F);
}

void RigidBody::changeParameters(RigidBody rbody, triple<double> v1, triple<double> v2)
{
    for (long int i = 0; i < numVertices; ++i)
    {
        vertices[i].x = vertices[i].xi;
        vertices[i].u = vertices[i].v = v1;
    }
    for (long int i = 0; i < rbody.numVertices; ++i)
     {
        rbody.vertices[i].x = rbody.vertices[i].xi;
        rbody.vertices[i].u = rbody.vertices[i].v = v2;
    }

    rbody.update(tlcol);
}

void RigidBody::printParameters()
{
    printf("Mass: %lf kg\n", this->rbMass);
    printf("Position: ( %lf, %lf, %lf) m\n", this->rbX.X(), this->rbX.Y(), this->rbX.Z());
    printf("Velocity: ( %lf, %lf, %lf) m/s\n", this->rbV.X(), this->rbV.Y(), this->rbV.Z());
}

void RigidBody::simulateIntersection(RigidBody rbody, double colT)
{
    triple<double> fvel1 = triple<double>(rbV.X(), rbV.Y(), rbV.Z()), fvel2 = triple<double>(rbody.rbV.X(), rbody.rbV.Y(), rbody.rbV.Z()),t1 = fvel1.multiply(1), t2 = fvel2.multiply(1);

    fvel1 = fvel1.multiply((rbMass-rbody.rbMass) / (rbMass+rbody.rbMass));
    fvel1 = fvel1.add(t2.multiply( 2*rbody.rbMass / (rbMass+rbody.rbMass) ));

    fvel2 = fvel2.multiply((rbody.rbMass - rbMass) / (rbMass+rbody.rbMass));
    fvel2 = fvel2.add(t1.multiply( 2*rbMass / (rbMass+rbody.rbMass) ));

    // Collision has happened now, update parameters.
    tlcol = rbody.tlcol = colT;
    changeParameters(rbody, fvel1, fvel2);
}
