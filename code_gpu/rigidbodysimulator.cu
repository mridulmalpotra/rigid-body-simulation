/*
 ===============================================================================
 Name        : RigidBodySimulation.cu
 Author      : Mridul & Srinidhi
 Version     :
 Copyright   : Copyleft
 Description : Parallel implementation of Rigid Body Dynamics on GPU using CUDA
 ===============================================================================
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

#include "rigidbody.h"

#define TIMESTEP 0.5
#define EPSILON 1
#define CHECK(F,X) \
F >> temp; \
if(temp != X){ throw ios_base::failure(X);}
//else { cout << "All OK for " << X << endl;}

using namespace std;

vector<RigidBody> rbodies;
vector< pair<double, double> > xList, yList, zList;
double currTime, timeSpan = 60;

void objstoSpec(string dirName) 
{
    // TODO: Convert multiplsimulateIntersection(rbodies[j], currTime)e input obj files to single spec file
}

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

//                /* vertex initial position */
//                rigidBody->vertices[i].xi.X(inputX[0]);
//                rigidBody->vertices[i].xi.Y(inputX[1]);
//                rigidBody->vertices[i].xi.Z(inputX[2]);

                /* vertex current position */
                rigidBody->vertices[i].x.X(inputX[0]);
                rigidBody->vertices[i].x.Y(inputX[1]);
                rigidBody->vertices[i].x.Z(inputX[2]);

                /* vertex velocity vector */
                inFile >> inputV[0];
                inFile >> inputV[1];
                inFile >> inputV[2] >> temp;

//                /* vertex initial velocity */
                rigidBody->vertices[i].u.X(inputV[0]);
                rigidBody->vertices[i].u.Y(inputV[1]);
                rigidBody->vertices[i].u.Z(inputV[2]);

                /* vertex current velocity */
//                rigidBody->vertices[i].v.X(inputV[0]);
//                rigidBody->vertices[i].v.Y(inputV[1]);
//                rigidBody->vertices[i].v.Z(inputV[2]);

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

    // Initialize rigid body cumulative values
    for (long int i = 0; i < rbodies.size(); ++i)
        rbodies[i].initRBParameters();
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
    cout << "Writing to "+fileName+"_"+to_string(currTime)+".obj" << endl;
    std::ofstream outFile(fileName+"_"+to_string(currTime)+".obj");
    outFile << "# Obj file generated from rigid body simulation engine" << endl;
    outFile << "# Note: Vertice points and faces specified." << endl;
    outFile << "# Material, normal etc. to be added as needed." << endl;
    for (int count = 0; count < rbodies.size(); ++count) {
        outFile << "o rigid_body_" << rbodies[count].rigidBodyID << endl;
        for (int i = 0; i < rbodies[count].numVertices; ++i) {
            outFile << "v "<< rbodies[count].vertices[i].x.X() << " " \
            << rbodies[count].vertices[i].x.Y() << " " \
            << rbodies[count].vertices[i].x.Z() << endl;
        }
        outFile << "s off" << endl;
        for (int i = 0; i < rbodies[count].numFaces; ++i) {
            outFile << "f "<< rbodies[count].faces[i].X() + count*rbodies[count].numVertices << " " \
            << rbodies[count].faces[i].Y() + count*rbodies[count].numVertices << " " \
            << rbodies[count].faces[i].Z() + count*rbodies[count].numVertices << endl;
        }
    }
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

int checkIntersection(long int a, long int b)
{

    if((xList[b].first < (xList[a].second + EPSILON) && xList[a].first < (xList[b].second + EPSILON)) &&
    (yList[b].first < (yList[a].second + EPSILON) && yList[a].first < (yList[b].second + EPSILON)) &&
    (zList[b].first < (zList[a].second + EPSILON) && zList[a].first < (zList[b].second + EPSILON)))
        return 1;
    return 0;
}

void checkCollisions()
{
    // Clear all lists
    xList.clear();
    yList.clear();
    zList.clear();

    // Create a sorted list of bounding boxes for all rigid bodies
    for (long int i = 0; i < rbodies.size(); ++i)
    {
        xList.push_back(rbodies[i].boundX);
        yList.push_back(rbodies[i].boundY);
        zList.push_back(rbodies[i].boundZ);
    }

    // Simulate collision on detection.
    for (long int i = 0; i < rbodies.size(); ++i)
    {
        for (long int j = 0; j < rbodies.size(); ++j)
        {
            if(i != j && checkIntersection(i,j))
                rbodies[i].simulateIntersection(rbodies[j], currTime);
        }
    }
}

int main()
{
    cout << "Welcome to Rigid Body Simulator!" << endl;

//    string inFileName = "specs/sample_specs.txt";
    string inFileName = "specs/sample_specs.txt";
    string outFileName = "obj/blocks";
    getInputData(inFileName);
//    printData();
    currTime = 0.0f;

    while( currTime < timeSpan)
    // Or we can have an infinite simulation loop.
//    while( 1 )
    {
        cout<<"\n\nTime: "<<currTime<<" s"<<endl;
        //computeParameters();
        updateParameters(&rbodies, currTime);
        //checkCollisions();
        cin.ignore();
        // Enable for debugging purposes. May slow down computation due to I/O bottleneck
        printData(); 
        writeOutputData(outFileName);

        currTime += TIMESTEP;
    }

    return 0;
}
