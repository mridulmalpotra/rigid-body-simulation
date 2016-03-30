/*
 ==============================================================================
 Name        : RigidBody.cpp
 Author      : Mridul & Srinidhi
 Version     :
 Copyright   : Copyleft
 Description : Serial implementation of Rigid Body Dynamics on CPU using CPP
 ==============================================================================
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include "rigidbody.h"

#define TIMESTEP 0.01
#define CHECK(F,X) \
F >> temp; \
if(temp != X){ throw ios_base::failure(X);}
//else { cout << "All OK for " << X << endl;}

using namespace std;

vector<RigidBody> rbodies;
double currTime, timeSpan = 100;

void getInputData(string fileName)
{
    cout << "Reading the specification file..." << endl;
    std::ifstream confFile (fileName);
    std::stringstream inFile;
    inFile << confFile.rdbuf();
    try {
        string temp;
        int numRigidBody;
        CHECK(inFile, "Num");
        inFile >> numRigidBody;
        for (int count = 0; count < numRigidBody; ++count) {
            cout << "Creating rigid body no. " << (count+1) << "..." << endl;
            int numVertices;

            CHECK(inFile,"=");

            CHECK(inFile, "NumVertices");
            inFile >> numVertices;

            RigidBody *rigidBody = new RigidBody(numVertices);

            for (int i = 0; i < rigidBody->numVertices; ++i) {
                int inputX[3];
                int inputV[3];

                /* vertexID */
                inFile >> rigidBody->vertices[i].vertexID >> temp;
                /* vertex position vector */
                inFile >> inputX[0];        
                inFile >> inputX[1];
                inFile >> inputX[2] >> temp;

                rigidBody->vertices[i].x.X(inputX[0]);
                rigidBody->vertices[i].x.Y(inputX[1]);
                rigidBody->vertices[i].x.Z(inputX[2]);

                /* vertex velocity vector */
                inFile >> inputV[0];
                inFile >> inputV[1];
                inFile >> inputV[2] >> temp;

                rigidBody->vertices[i].v.X(inputV[0]);
                rigidBody->vertices[i].v.Y(inputV[1]);
                rigidBody->vertices[i].v.Z(inputV[2]);

                /* vertex mass */
                inFile >> rigidBody->vertices[i].mass;
            }

            CHECK(inFile, "NumEdges");
            inFile >> rigidBody->numEdges;

            for (int i = 0; i < rigidBody->numEdges; ++i) {
                int vertex1, vertex2;
                inFile >> vertex1 >> vertex2;
                vertex1-=1;
                vertex2-=1;
                (rigidBody->edges)[vertex1][vertex2] = (rigidBody->edges)[vertex2][vertex1] = 1;
            }

            CHECK(inFile, "NumFaces");
            inFile >> rigidBody->numFaces;
            for (int i = 0; i < rigidBody->numFaces; ++i) {
                int faceVertices[3];
                int faceID;
                inFile >> faceID >> temp >> faceVertices[0] >> faceVertices[1] >> faceVertices[2];
                triple <int> faceVerticesTriple(faceVertices[0], faceVertices[1], faceVertices[2]);
                rigidBody->faces.push_back(faceVerticesTriple);
            }

            CHECK(inFile,"ID");
            inFile >> rigidBody->rigidBodyID;

            rbodies.push_back(*rigidBody);
            cout << "Created" << endl;
        }

        CHECK(inFile, "EOF");

        cout << "Parsed the file successfully." << endl;

    } catch (const exception &ex) {
        cerr << "Failed to read " << ex.what() << endl;
        cerr << "Error in specification file!" << endl;
        cerr << "Quitting..." << endl;
        exit(1);
    }
}

void printData()
{
    for (int count = 0; count < rbodies.size(); ++count) {
        cout << "\nRIGID BODY ID: " << rbodies[count].rigidBodyID << endl;

        /* Printing vertices */
        cout << "\nNumber of vertices = " << rbodies[count].numVertices << endl;
        for (int i = 0; i < rbodies[count].numVertices; ++i) {
            cout << "ID:" << (i+1) << " X: (" << rbodies[count].vertices[i].x.X() << "," << \
                    rbodies[count].vertices[i].x.Y() << "," << \
                    rbodies[count].vertices[i].x.Z() << ") " << \
                " V: (" << rbodies[count].vertices[i].v.X() << "," << \
                    rbodies[count].vertices[i].v.Y() << "," << \
                    rbodies[count].vertices[i].v.Z() << ") " << \
                " Mass: " << rbodies[count].vertices[i].mass << endl;

        }

        /* Printing edges*/
        cout << "\nNumber of edges = " << rbodies[count].numEdges << endl;
        for (int i = 0; i < rbodies[count].numVertices; ++i) {
            for (int j = 0; j < rbodies[count].numVertices; ++j) {
                cout << rbodies[count].edges[i][j] << " ";
            }
            cout << endl;
        }

        /* Printing faces */
        cout << "\nNumber of faces = " << rbodies[count].numFaces << endl;
        for (int i = 0; i < rbodies[count].numFaces; ++i) {
            cout << "ID:" << (i+1) << " (" << rbodies[count].faces[i].X() << "," << \
                    rbodies[count].faces[i].Y() << "," << \
                    rbodies[count].faces[i].Z() << ") " << endl;
        }
    }
}

void writeOutputData(string fileName)
{
    // COLLADA convertor coming in the next commit
}

void computeParameters()
{
    for (long int i = 0; i < rbodies.size(); ++i)
    {
        rbodies[i].update(currTime);
        cout<<"Rigid Body "<< i+1 << ":" << endl;
        rbodies[i].printParameters();
    }
}

int main()
{
    cout << "Welcome to Rigid Body Simulator!" << endl;

    string inFileName = "sample_specs.txt";
    string outFileName = "";
    getInputData(inFileName);
    printData();
    exit(0);
    currTime = 0.0f;

    while( currTime < timeSpan)
    // Or we can have an infinite simulation loop.
    // while( 1 )
    {
        cout<<"Time: "<<currTime<<" s"<<endl;
//        computeParameters();
        writeOutputData(outFileName);

        currTime += TIMESTEP;
    }

    return 0;
}

