

#include <cuda.h>
#include <cuda_runtime_api.h>

#include "params.h"

//For the function below
#define THREADS_FOR_VOXEL 256 // threads number for a block
#define POINTS_PER_VOXEL 32 //depands on "params.h"

cudaError_t generateVoxels_random_launch(float *points, size_t points_size,
                                         float min_x_range, float max_x_range,
                                         float min_y_range, float max_y_range,
                                         float min_z_range, float max_z_range,
                                         float pillar_x_size, float pillar_y_size, float pillar_z_size,
                                         int grid_y_size, int grid_x_size,
                                         unsigned int *mask, float *voxels,
                                         cudaStream_t stream = 0);

cudaError_t generateVoxels_launch(float *points, size_t points_size,
                                  float min_x_range, float max_x_range,
                                  float min_y_range, float max_y_range,
                                  float min_z_range, float max_z_range,
                                  float pillar_x_size, float pillar_y_size, float pillar_z_size,
                                  int grid_y_size, int grid_x_size,
                                  unsigned int *mask, float *voxels, int *voxelsList,
                                  cudaStream_t stream = 0);

cudaError_t generateBaseFeatures_launch(unsigned int *mask, float *voxels,
                                        int grid_y_size, int grid_x_size,
                                        unsigned int *pillar_num,
                                        float *voxel_features,
                                        float *voxel_num_points,
                                        float *coords,
                                        cudaStream_t stream = 0);

//For the function below
#define WARP_SIZE 32 //one warp(32 threads) for one pillar
#define WARPS_PER_BLOCK 4 //four warp for one block
#define FEATURES_SIZE 10 //features maps number depands on "params.h"

cudaError_t generateFeatures_launch(float *voxel_features,
                                    float *voxel_num_points,
                                    float *coords,
                                    unsigned int *params,
                                    float voxel_x, float voxel_y, float voxel_z,
                                    float range_min_x, float range_min_y, float range_min_z,
                                    float *features,
                                    cudaStream_t stream = 0);

