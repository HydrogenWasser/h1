
#pragma once
#ifndef _BEV_KERNELS_H_
#define _BEV_KERNELS_H_

#define PILLARS_PER_BLOCK 64 //one thread deals with one pillar and a block has PILLARS_PER_BLOCK threads
#define FEATURE_SIZE 64 //feature count for one pillar depands on "params.h"
#define POINTS_PER_PILLAR 32 //points in one pillar depands on "params.h

#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}

cudaError_t scatterBEV_kernel_launcher(const float *pillar_features_data,
                                       const float *coords_data, const unsigned int *params_data,
                                       unsigned int featureX, unsigned int featureY,
                                       float *spatial_feature_data,
                                       cudaStream_t stream = 0);

//for N*32*64
cudaError_t reduceMax_kernel_launcher(const float *in,
                                      float *out, unsigned int count,
                                      cudaStream_t stream = 0);

#endif
