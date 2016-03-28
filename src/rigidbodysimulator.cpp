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
#include <cstring>
#include <string>
#include <vector>

#include "rigidbody.h"

#define TIMESTEP 0.01
using namespace std;

vector<RigidBody> rbodies;
double currTime, timeSpan = 100;

void getInputData(string fileName)
{
    // Calculate timeSpan here
}

void writeOutputData(string fileName)
{

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

    string inFileName = "", outFileName = "";
    getInputData(inFileName);
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

