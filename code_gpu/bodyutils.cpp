/*
===============================================================================
Name        : bodyutils.cpp
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

extern void printData(void);

/* Device code */
__global__ void updateVectors(double *vertex_masses, double *vertex_u, 	double *vertex_xi, 
	double *vertex_x, double *vertex_P, double *vertex_v, double *vertex_F, double *tlcols, 
	double currTime, int numVertices)
{
	int i = threadIdx.x + blockDim.x * blockIdx.x;
	// Position
	double tmpPos, tmpvel, deltaT;
	deltaT = currTime - tlcols[i/(3*numVertices)]; // one tlcol defined for each rigid body
	tmpvel = vertex_u[i];
	tmpPos = vertex_F[i]* 0.5 * deltaT * deltaT / vertex_masses[i/3]; // vertex mass defined once per vertex
    vertex_x[i]  = tmpvel * deltaT;
    vertex_x[i] = vertex_x[i] + tmpPos;
    vertex_x[i] = vertex_x[i] + vertex_xi[i];

	// Velocity
	vertex_v[i] = vertex_u[i]/*.add(vertices[i].F.multiply((t-tlcol)/vertices[i].mass))*/;

	// Momentum
	vertex_P[i] = vertex_v[i] * vertex_masses[i/3];

	// TODO: Force (Non-constant)
}


__global__ void updateRB (double *vertex_masses, double *vertex_x, double *vertex_v, double *vertex_F, 
	double *rbMasses, double *rbXs, double *rbPs, double *rbVs, double *rbFs, int numVertices)
{
	int i = threadIdx.x + (blockIdx.x * blockDim.x);
	for (int offset = 0; offset < numVertices; ++offset) {
		rbXs[i] += (vertex_x[numVertices*i+offset] * vertex_masses[(numVertices*i/3)+offset]);
		rbVs[i] += (vertex_v[numVertices*i+offset] * vertex_masses[(numVertices*i/3)+offset]);
		rbFs[i] += (vertex_F[numVertices*i+offset]);
	}
	rbPs[i] = (rbVs[i] * rbMasses[i/3]);
	rbXs[i] /= rbMasses[i/3];
	rbVs[i] /= rbMasses[i/3];
}


__global__ void updateBounds(double *boundXs, double *boundYs, double *boundZs, double *vertex_x, 
	int numVertices)
{
    int i = threadIdx.x + (blockDim.x * blockIdx.x);
    for (int offset = 0; offset < numVertices; ++offset) {
		boundXs[2*i] = min(boundXs[2*i], vertex_x[numVertices*i+offset]);
		boundXs[2*i+1] = max(boundXs[2*i], vertex_x[numVertices*i+offset]);
		boundYs[2*i] = min(boundYs[2*i], vertex_x[numVertices*i+offset]);
		boundYs[2*i+1] = max(boundYs[2*i], vertex_x[numVertices*i+offset]);
		boundZs[2*i] = min(boundZs[2*i], vertex_x[numVertices*i+offset]);
		boundZs[2*i+1] = max(boundZs[2*i], vertex_x[numVertices*i+offset]);
    }    
}


/* Host code */
__host__ void checkCudaErrors(int err_code)
{
	if (cudaSuccess != err_code ) {
		fprintf(stderr, "checkCudaErrors() failed with error message :%s\n", err_code);
		exit(1);
	}
}


__host__ void updateParameters(vector<RigidBody> &rbodies, double currTime)
{
	int numVertices = (rbodies)[0].numVertices;
	int numRigidBodies = (rbodies).size();
	printf("Number of rigid bodies to be updated: %d\n", numRigidBodies);
	printf("Number of vertices per rigid body: %d\n", numVertices);
	
	/*CUDA events for measuring time*/
	// cudaEvent_t start, stop;
// 	float time;
//	cudaEventCreate(&start);
//	cudaEventCreate(&stop);

	/* Host Vertex SoA */
	double vertex_masses[numRigidBodies][numVertices];
	double vertex_u[numRigidBodies][numVertices][3];
	double vertex_xi[numRigidBodies][numVertices][3];
	double vertex_x[numRigidBodies][numVertices][3];
	double vertex_P[numRigidBodies][numVertices][3];
	double vertex_v[numRigidBodies][numVertices][3];
	double vertex_F[numRigidBodies][numVertices][3];
	double tlcols[numRigidBodies];

	for (int i = 0; i < numRigidBodies; ++i) {
		RigidBody *rbody = &((rbodies)[i]);
		for (int j = 0; j < rbody->numVertices; ++j) {
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
	double *d_vertex_masses;
	double *d_vertex_u;
	double *d_vertex_xi;
	double *d_vertex_x;
	double *d_vertex_P;
	double *d_vertex_v;
	double *d_vertex_F;
	double *d_tlcols;

	/* Vertex Memory allocation*/
    checkCudaErrors(cudaMalloc(&d_vertex_masses, sizeof(double)*numRigidBodies*numVertices));
    checkCudaErrors(cudaMalloc(&d_vertex_u, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_xi, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_x, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_P, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_v, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_vertex_F, sizeof(double)*numRigidBodies*numVertices*3));
    checkCudaErrors(cudaMalloc(&d_tlcols, sizeof(long int)*numRigidBodies));

    /* Copy Host to Device memory */
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
    printf("About to launch vertex update kernel\n");
   	int totalThreads = numRigidBodies * numVertices * 3;
   	int numThreads = min(1008, totalThreads); // 1008 is a multiple of 24 (= 8 vertices * 3 dimensions)
   	int numBlocks = max(1,totalThreads/numThreads);
   	printf("Number of threads %d blocks %d\n", numThreads, numBlocks);
   	cin.ignore();
   	
//	cudaEventRecord(start,0);
	updateVectors <<<numBlocks, numThreads>>> (d_vertex_masses, d_vertex_u, d_vertex_xi, d_vertex_x, 
		d_vertex_P, d_vertex_v, d_vertex_F, d_tlcols, currTime, numVertices);
	printf("Launch completed. Please press enter to continue:\n");
	cin.ignore();
//	cudaEventRecord(stop,0);
//	cudaEventSynchronize(stop);

//	cudaEventElapsedTime(&time, start, stop);
//	printf("Time taken by the kernel: %f ms\n", time);	
	
    /* Copy Device to Host memory */
    checkCudaErrors(cudaMemcpy(vertex_x, d_vertex_x, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(vertex_P, d_vertex_P, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(vertex_v, d_vertex_v, sizeof(double)*numRigidBodies*numVertices*3, cudaMemcpyDeviceToHost));
    printf("Successfully copied the memory back\n");

    for (int i = 0; i < numRigidBodies; ++i) {
		RigidBody *rbody = &((rbodies)[i]);
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
	printf("transferred the values to rigid bodies. Printing...\n");
	cin.ignore();
	printData();

	printf("Press enter to continue...\n");
	cin.ignore();

	/* Host RigidBody SoA */
    double rbMasses[numRigidBodies];
    double rbXs[numRigidBodies][3];
    double rbPs[numRigidBodies][3];
    double rbVs[numRigidBodies][3];
    double rbFs[numRigidBodies][3];

    double boundXs[numRigidBodies][2];
    double boundYs[numRigidBodies][2];
    double boundZs[numRigidBodies][2];

    for (int i = 0; i < numRigidBodies; ++i) {
    	RigidBody *rbody = &((rbodies)[i]);
    	rbMasses[i] = rbody->rbMass;

    	rbXs[i][0] = (rbody->rbX).X();
    	rbXs[i][1] = (rbody->rbX).Y();
    	rbXs[i][2] = (rbody->rbX).Z();

		rbPs[i][0] = (rbody->rbP).X();
    	rbPs[i][1] = (rbody->rbP).Y();
    	rbPs[i][2] = (rbody->rbP).Z();

    	rbVs[i][0] = (rbody->rbV).X();
    	rbVs[i][1] = (rbody->rbV).Y();
    	rbVs[i][2] = (rbody->rbV).Z();

    	rbFs[i][0] = (rbody->rbF).X();
    	rbFs[i][1] = (rbody->rbF).Y();
    	rbFs[i][2] = (rbody->rbF).Z();

    	boundXs[i][0] = LONG_MAX;
    	boundXs[i][1] = LONG_MIN;
    	boundYs[i][0] = LONG_MAX;
    	boundYs[i][1] = LONG_MIN;
    	boundZs[i][0] = LONG_MAX;
    	boundZs[i][1] = LONG_MIN;
    }

    /* Device RigidBody SoA */
    double *d_rbMasses;
    double *d_rbXs;
    double *d_rbPs;
    double *d_rbVs;
    double *d_rbFs;

    double *d_boundXs;
    double *d_boundYs;
    double *d_boundZs;

    /* RigidBody Memory allocation*/
    checkCudaErrors(cudaMalloc(&d_rbMasses, sizeof(double)*numRigidBodies));
    checkCudaErrors(cudaMalloc(&d_rbXs, sizeof(double)*numRigidBodies*3));
    checkCudaErrors(cudaMalloc(&d_rbPs, sizeof(double)*numRigidBodies*3));
    checkCudaErrors(cudaMalloc(&d_rbVs, sizeof(double)*numRigidBodies*3));
    checkCudaErrors(cudaMalloc(&d_rbFs, sizeof(double)*numRigidBodies*3));
    checkCudaErrors(cudaMalloc(&d_boundXs, sizeof(double)*numRigidBodies*2));
    checkCudaErrors(cudaMalloc(&d_boundYs, sizeof(double)*numRigidBodies*2));
    checkCudaErrors(cudaMalloc(&d_boundZs, sizeof(double)*numRigidBodies*2));

    /* Copy Host to Device memory */
    checkCudaErrors(cudaMemcpy(d_rbMasses, rbMasses, sizeof(double)*numRigidBodies, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_rbXs, rbXs, sizeof(double)*numRigidBodies*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_rbPs, rbPs, sizeof(double)*numRigidBodies*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_rbVs, rbVs, sizeof(double)*numRigidBodies*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_rbFs, rbFs, sizeof(double)*numRigidBodies*3, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_boundXs, boundXs, sizeof(double)*numRigidBodies*2, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_boundYs, boundYs, sizeof(double)*numRigidBodies*2, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_boundZs, boundZs, sizeof(double)*numRigidBodies*2, cudaMemcpyHostToDevice));

    printf("Memory copying done. Press enter to continue...\n");
	cin.ignore();
    /* Stream creation for asych bounds kernel launch */
    cudaStream_t stream1_rigidbody, stream2_bound;
    cudaStreamCreateWithFlags(&stream1_rigidbody, cudaStreamNonBlocking);
    cudaStreamCreateWithFlags(&stream2_bound, cudaStreamNonBlocking);

    /* Kernel launches */
    printf("About to launch rigid body update kernel\n");
   	totalThreads = numRigidBodies * 3;
   	numThreads = min(1008, totalThreads); // 1008 is a multiple of 24 (= 8 vertices * 3 dimensions)
   	numBlocks = max(1,totalThreads/numThreads);
   	printf("Number of threads %d blocks %d\n", numThreads, numBlocks);

   	updateRB <<<numBlocks, numThreads, 0, stream1_rigidbody>>> (d_vertex_masses, d_vertex_x, d_vertex_v, 
   		d_vertex_F,	d_rbMasses, d_rbXs, d_rbPs, d_rbVs, d_rbFs, numVertices);

   	printf("About to launch bounds update kernel\n");
   	totalThreads = numRigidBodies;
   	numThreads = min(1008, totalThreads); // 1008 is a multiple of 24 (= 8 vertices * 3 dimensions)
   	numBlocks = max(1,totalThreads/numThreads);
   	printf("Number of threads %d blocks %d\n", numThreads, numBlocks);

	updateBounds <<<numBlocks, numThreads, 0, stream2_bound>>> (d_boundXs, d_boundYs, d_boundZs, 
		d_vertex_x, numVertices);

   	cudaStreamSynchronize(stream2_bound);

   	checkCudaErrors(cudaMemcpy(rbXs, d_rbXs, sizeof(double)*numRigidBodies*3, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(rbPs, d_rbPs, sizeof(double)*numRigidBodies*3, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(rbVs, d_rbVs, sizeof(double)*numRigidBodies*3, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(rbFs, d_rbFs, sizeof(double)*numRigidBodies*3, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(boundXs, d_boundXs, sizeof(double)*numRigidBodies*2, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(boundYs, d_boundYs, sizeof(double)*numRigidBodies*2, cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(boundZs, d_boundZs, sizeof(double)*numRigidBodies*2, cudaMemcpyDeviceToHost));

    for (int i = 0; i < numRigidBodies; ++i) {
    	RigidBody *rbody = &((rbodies)[i]);

    	(rbody->rbX).X(rbXs[i][0]);
    	(rbody->rbX).Y(rbXs[i][1]);
    	(rbody->rbX).Y(rbXs[i][2]);

		(rbody->rbP).X(rbPs[i][0]);
    	(rbody->rbP).Y(rbPs[i][1]);
    	(rbody->rbP).Y(rbPs[i][2]);

    	(rbody->rbV).X(rbVs[i][0]);
    	(rbody->rbV).Y(rbVs[i][1]);
    	(rbody->rbV).Y(rbVs[i][2]);

    	(rbody->rbF).X(rbFs[i][0]);
    	(rbody->rbF).Y(rbFs[i][1]);
    	(rbody->rbF).Y(rbFs[i][2]);

    	rbody->boundX.first = boundXs[i][0];
    	rbody->boundX.second = boundXs[i][1];
    	rbody->boundY.first = boundYs[i][0];
    	rbody->boundY.second = boundYs[i][1];
    	rbody->boundZ.first = boundZs[i][0];
    	rbody->boundZ.second = boundZs[i][1];
    }

    return;
}
