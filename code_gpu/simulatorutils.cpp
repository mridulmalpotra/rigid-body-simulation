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
    double *d_x, *d_y, *d_z;
    int *d_mat;

    cudaMalloc(&d_x, n*sizeof(double));
    cudaMalloc(&d_y, n*sizeof(double));
    cudaMalloc(&d_z, n*sizeof(double));
    cudaMalloc(&d_mat, n*n*sizeof(int));

    cudaMemcpy(d_x, x, n*sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y, n*sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(d_z, z, n*sizeof(double), cudaMemcpyHostToDevice);

    dim3 dimGrid(n,1);
    dim3 dimBlock(n,1);

    checkCollision <<< dimGrid, dimBlock >>> (d_x, d_y, d_z, d_mat, n);

    cudaMemcpy(mat, d_mat, n*n*sizeof(int), cudaMemcpyDeviceToHost);

}
