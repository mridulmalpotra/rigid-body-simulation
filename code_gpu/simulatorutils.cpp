/*
 ===============================================================================
 Name        : simulatorutils.cpp
 Author      : Mridul & Srinidhi
 Version     :
 Copyright   : Copyleft
 Description : Parallel implementation of Rigid Body Dynamics on GPU using CUDA
 ===============================================================================
 */

#include <stdio.h>

#define EPSILON 1


__device__ int checkIntersection(double x1,double x2,double y1,double y2,double z1,double z2,double a1,double a2,double b1,double b2,double c1, double c2)
{
    if((a1 < (x2 + EPSILON) && x1 < (a2 + EPSILON)) && (b1 < (y2 + EPSILON) && y1 < (b2 + EPSILON)) &&
          (c1 < (z2 + EPSILON) && z1 < (c2 + EPSILON)))
        return 1;
    return 0;
}

__global__ void checkCollision(double *d_x, double *d_y, double *d_z, int *d_mat, int n)
{
    int row = threadIdx.y ;
    int col = threadIdx.x ;
    int idx = row*n + col;
    
    for (long int j = 0; j < n; ++j)
    {    	
        if(idx != j && checkIntersection(d_x[2*idx], d_x[2*idx+1], d_y[2*idx], d_y[2*idx+1], d_z[2*idx], d_z[2*idx+1], d_x[2*j], d_x[2*j+1], d_y[2*j], d_y[2*j+1], d_z[2*j], d_z[2*j+1]))
            d_mat[idx*n+j] = 1;
        else
            d_mat[idx*n+j] = 0;

        __syncthreads();
    }
}

__host__ void collisionWrapper(double *x, double *y, double *z, int *mat, int n)
{
    double *d_x, *d_y, *d_z, *h_x, *h_y, *h_z;
    int *d_mat;
    
    // Creating streams
    cudaStream_t stream1,stream2,stream3;
    cudaStreamCreateWithFlags(&stream1, cudaStreamNonBlocking);
    cudaStreamCreateWithFlags(&stream2, cudaStreamNonBlocking);
    cudaStreamCreateWithFlags(&stream3, cudaStreamNonBlocking);

    // Creating pinned memory for hosts
    cudaMallocHost(&h_x, n*sizeof(double));
    cudaMallocHost(&h_y, n*sizeof(double));
    cudaMallocHost(&h_z, n*sizeof(double));
    
    for(int i=0; i<n; i++)
    {
    	h_x[i] = x[i];
    	h_y[i] = y[i];
    	h_z[i] = z[i];
    }
    
    cudaMalloc(&d_x, n*sizeof(double));
    cudaMalloc(&d_y, n*sizeof(double));
    cudaMalloc(&d_z, n*sizeof(double));
    cudaMalloc(&d_mat, n*n*sizeof(int));

//    cudaMemcpy(d_x, x, n*sizeof(double), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_y, y, n*sizeof(double), cudaMemcpyHostToDevice);
//    cudaMemcpy(d_z, z, n*sizeof(double), cudaMemcpyHostToDevice);


    // Asynchronous memCpy using different streams with synchronization
    cudaMemcpyAsync(d_x, h_x, n*sizeof(double), cudaMemcpyHostToDevice, stream1);
    cudaMemcpyAsync(d_y, h_y, n*sizeof(double), cudaMemcpyHostToDevice, stream2);
    cudaMemcpyAsync(d_z, h_z, n*sizeof(double), cudaMemcpyHostToDevice, stream3);
    cudaStreamSynchronize(stream3);
    
    dim3 dimGrid(n,1);
    dim3 dimBlock(n,1);

    checkCollision <<< dimGrid, dimBlock >>> (d_x, d_y, d_z, d_mat, n);

    cudaMemcpy(mat, d_mat, n*n*sizeof(int), cudaMemcpyDeviceToHost);
    
    // Destroying streams
    cudaStreamDestroy(stream1);
    cudaStreamDestroy(stream2);
    cudaStreamDestroy(stream3);
    
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_z);
    
    cudaFreeHost(h_x);
    cudaFreeHost(h_y);
    cudaFreeHost(h_z);
    

}
