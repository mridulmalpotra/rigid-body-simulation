/*
===============================================================================
Name        : bodyutils.cu
Author      : Mridul & Srinidhi
Version     :
Copyright   : Copyleft
Description : Parallel implementation of Rigid Body Dynamics on GPU using CUDA
===============================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <iostream>

#include <rigidbody.h>

#define EPSILON 1

/* Device code */
__global__ void updateVectors(double *vertex_IDs, double *vertex_masses, double *vertex_u, 
	double *vertex_xi, double *vertex_x, double *vertex_P, double *vertex_v, double *vertex_F, 
	double *tlcols, double currTime, double numVertices)
{
	int i = threadIdx.x + blockDim.x * blockIdx.x;

	// Position
	double tmpPos,tmpvel, deltaT;
	deltaT = currTime - tlcols[i/3];
	tmpvel = vertex_u[i];
	tmpPos = vertex_F[i]* 0.5 * deltaT * deltaT / vertex_masses[i/3];
    vertex_x[i]  = tmpvel * deltaT;
    vertex_x[i] = vertex_x[i] + tmpPos;
    vertex_x[i] = vertex_x[i] + vertex_xi[i];

	// Velocity
	vertex_v[i] = vertex_u[i]/*.add(vertices[i].F.multiply((t-tlcol)/vertices[i].mass))*/;

	// Momentum
	vertex_P[i] = vertex_v[i] * vertex_masses[i];

	// TODO: Force (Non-constant)
}

__global__ void updateRB()
{
	// TODO
}

/* Host code */
__host__ void checkCudaErrors(int err_code)
{
	if (cudaSuccess != err_code ) {
		fprintf(stderr, "checkCudaErrors() failed with error message :%s\n", err_code);
		exit(1);
	}
}


__host__ void updateParameters(vector<RigidBody> *rbodies, double currTime)
{
	int numVertices = (*rbodies)[0].numVertices;
	int numRigidBodies = (*rbodies).size();
	printf("Number of rigid bodies to be updated: %d\n", numRigidBodies);
	printf("Number of vertices per rigid body: %d\n", numVertices);
	string temp;
	cin >> temp;

	/* Host Vertex SoA */
	double vertex_IDs[numRigidBodies][numVertices];
	double vertex_masses[numRigidBodies][numVertices];
	double vertex_u[numRigidBodies][numVertices][3];
	double vertex_xi[numRigidBodies][numVertices][3];
	double vertex_x[numRigidBodies][numVertices][3];
	double vertex_P[numRigidBodies][numVertices][3];
	double vertex_v[numRigidBodies][numVertices][3];
	double vertex_F[numRigidBodies][numVertices][3];
	double tlcols[numRigidBodies];

	for (int i = 0; i < numRigidBodies; ++i) {
		RigidBody *rbody = (*rbodies)[i];
		for (int j = 0; j < rbody->numVertices; ++j) {
			vertex_IDs[i][j] = (rbody->vertices[j]).vertexID;
			vertex_masses[i][j] = (rbody->vertices[j]).mass;

			vertex_u[i][j][0] = (rbody->vertices[j]).u.X();
			vertex_u[i][j][1] = (rbody->vertices[j]).u.Y();
			vertex_u[i][j][2] = (rbody->vertices[j]).u.Z();

			vertex_xi[i][j][0] = (rbody->vertices[j]).xi.X();
			vertex_xi[i][j][1] = (rbody->vertices[j]).xi.Y();
			vertex_xi[i][j][2] = (rbody->vertices[j]).xi.Z();

			vertex_x[i][j][0] = (rbody->vertices[j]).x.X();
			vertex_x[i][j][1] = (rbody->vertices[j]).x.Y();
			vertex_x[i][j][2] = (rbody->vertices[j]).x.Z();

			vertex_P[i][j][0] = (rbody->vertices[j]).P.X();
			vertex_P[i][j][1] = (rbody->vertices[j]).P.Y();
			vertex_P[i][j][2] = (rbody->vertices[j]).P.Z();

			vertex_v[i][j][0] = (rbody->vertices[j]).v.X();
			vertex_v[i][j][1] = (rbody->vertices[j]).v.Y();
			vertex_v[i][j][2] = (rbody->vertices[j]).v.Z();

			vertex_F[i][j][0] = (rbody->vertices[j]).F.X();
			vertex_F[i][j][1] = (rbody->vertices[j]).F.Y();
			vertex_F[i][j][2] = (rbody->vertices[j]).F.Z();
		}
		tlcols[i] = rbody->tlcol;
	}

	/* Device Vertex SoA */
	double *d_vertex_IDs;
	double *d_vertex_masses;
	double *d_vertex_u;
	double *d_vertex_xi;
	double *d_vertex_x;
	double *d_vertex_P;
	double *d_vertex_v;
	double *d_vertex_F;
	double *d_tlcols;

	/* Vertex Memory allocation*/
	checkCudaErrors(cudaMalloc(&d_vertex_IDs, sizeof(double)*numRigidBodies*numVertices));
    checkCudaErrors(cudaMalloc(&d_vertex_masses, sizeof(double)*numRigidBodies*numVertices));
    checkCudaErrors(cudaMalloc(&d_vertex_u, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_xi, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_x, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_P, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_v, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_F, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_tlcols, sizeof(long int)*numRigidBodies));

    /* Copy Host to Device memory */
    checkCudaErrors(cudaMemcpy(d_vertex_IDs, vertex_IDs, sizeof(double)*numRigidBodies*numVertices, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_vertex_masses, vertex_masses, sizeof(double)*numRigidBodies*numVertices, cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(d_vertex_u, vertex_u, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_vertex_xi, vertex_xi, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_vertex_u, vertex_x, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_vertex_P, vertex_P, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_vertex_v, vertex_v, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_vertex_F, vertex_F, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_tlcols, tlcols, sizeof(double)*numRigidBodies, cudaMemcpyHostToDevice));

//    int threadPerRB = numVertices * 3;
//    if(threadPerRB > 1024) {
//    	fprintf(stderr, "Too many vertices! Please reduce to at least 340\n");
//    	exit(1);
//    }
//   	int RBThreads = 1024/min(1024, threadPerRB);
//   	dim3 blockDimension(RBThreads, numVertices, 3);

    /* Kernel launch */
    printf("About to launch kernel\n");
   	int totalThreads = numRigidBodies * numVertices * 3;
   	int numThreads = min(1008, totalThreads); // 1008 is a multiple of 24 (= 8 vertices * 3 dimensions)
   	int numBlocks = totalThreads/numThreads;
    updateVectors <<<numBlocks, numThreads>>> (d_vertex_IDs, d_vertex_masses, d_vertex_u, 
    	d_vertex_xi, d_vertex_x, d_vertex_P, d_vertex_v, d_vertex_F, d_tlcols, currTime, numVertices);

    /* Copy Device to Host memory */
    checkCudaErrors(cudaMemcpy(vertex_x, d_vertex_x, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(vertex_P, d_vertex_P, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(vertex_v, d_vertex_v, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyDeviceToHost));

    for (int i = 0; i < numRigidBodies; ++i) {
		RigidBody *rbody = (*rbodies)[i];
		for (int j = 0; j < rbody->numVertices; ++j) {

			(rbody->vertices[j]).x.X(vertex_x[i][j][0]);
			(rbody->vertices[j]).x.Y(vertex_x[i][j][1]);
			(rbody->vertices[j]).x.Z(vertex_x[i][j][2]);

			(rbody->vertices[j]).v.X(vertex_v[i][j][0]);
			(rbody->vertices[j]).v.Y(vertex_v[i][j][1]);
			(rbody->vertices[j]).v.Z(vertex_v[i][j][2]);

			(rbody->vertices[j]).P.X(vertex_P[i][j][0]);
			(rbody->vertices[j]).P.Y(vertex_P[i][j][1]);
			(rbody->vertices[j]).P.Z(vertex_P[i][j][2]);
		}
	}

	/* Host RigidBody SoA */
    /*int rigidBodyIDs[numRigidBodies];
    double rbMasses[numRigidBodies];
    int rbXs[numRigidBodies][3];
    int rbPs[numRigidBodies][3];
    int rbVs[numRigidBodies][3];
    int rbFs[numRigidBodies][3];

    double boundXs[numRigidBodies][2];
    double boundYs[numRigidBodies][2];
    double boundZs[numRigidBodies][2];*/


    /* Device RigidBody SoA */
    /*int *d_rigidBodyIDs;
    double *d_rbMasses;
    int *d_rbXs;
    int *d_rbPs;
    int *d_rbVs;
    int *d_rbFs;

    double *d_boundXs;
    double *d_boundYs;
    double *d_boundZs;*/


    /* RigidBody Memory allocation*/
    /*checkCudaErrors(cudaMalloc(&d_rigidBodyIDs, sizeof(double)*numRigidBodies));
    checkCudaErrors(cudaMalloc(&d_rbMasses, sizeof(double)*numRigidBodies));
    checkCudaErrors(cudaMalloc(&d_rbXs, sizeof(int)*numRigidBodies*3));
    checkCudaErrors(cudaMalloc(&d_rbPs, sizeof(int)*numRigidBodies*3));
    checkCudaErrors(cudaMalloc(&d_rbVs, sizeof(int)*numRigidBodies*3));
    checkCudaErrors(cudaMalloc(&d_rbFs, sizeof(int)*numRigidBodies*3));
    checkCudaErrors(cudaMalloc(&d_boundXs, sizeof(double)*numRigidBodies*2));
    checkCudaErrors(cudaMalloc(&d_boundYs, sizeof(double)*numRigidBodies*2));
    checkCudaErrors(cudaMalloc(&d_boundZs, sizeof(double)*numRigidBodies*2));
    checkCudaErrors(cudaMalloc(&d_tlcols, sizeof(double)*numRigidBodies));*/





    return;
}