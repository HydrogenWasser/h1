//
// Created by root on 1/8/22.
//

#ifndef SECOND_DETECTOR_VOXELIZE_H
#define SECOND_DETECTOR_VOXELIZE_H

#include <iostream>
#include <torch/script.h>
#include <ATen/ATen.h>
#include <ATen/cuda/CUDAContext.h>
#include <c10/cuda/CUDAGuard.h>
#include <torch/types.h>
//#include <xtensor/xarray.hpp>
//#include <xtensor/xnpy.hpp>
#include <vector>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"



#define CHECK_CUDA(x) \
  TORCH_CHECK(x.device().is_cuda(), #x " must be a CUDA tensor")
#define CHECK_CONTIGUOUS(x) \
  TORCH_CHECK(x.is_contiguous(), #x " must be contiguous")
#define CHECK_INPUT(x) \
  CHECK_CUDA(x);       \
  CHECK_CONTIGUOUS(x)

template <typename T>
__host__ __device__ __forceinline__ T ATenCeilDiv(T a, T b) {
    return (a + b - 1) / b;
}


#define CUDA_1D_KERNEL_LOOP(i, n)                            \
  for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < n; \
       i += blockDim.x * gridDim.x)

int const threadsPerBlock = sizeof(unsigned long long) * 8;
torch::Tensor const temp = torch::empty({0});
template <typename T, typename T_int>
__global__ void dynamic_voxelize_kernel(
        const T* points, T_int* coors, const float voxel_x, const float voxel_y,
        const float voxel_z, const float coors_x_min, const float coors_y_min,
        const float coors_z_min, const float coors_x_max, const float coors_y_max,
        const float coors_z_max, const int grid_x, const int grid_y,
        const int grid_z, const int num_points, const int num_features,
        const int NDim) {
    //   const int index = blockIdx.x * threadsPerBlock + threadIdx.x;
    CUDA_1D_KERNEL_LOOP(index, num_points) {
        // To save some computation
        auto points_offset = points + index * num_features;
        auto coors_offset = coors + index * NDim;
        int c_x = floor((points_offset[0] - coors_x_min) / voxel_x);
        if (c_x < 0 || c_x >= grid_x) {
            coors_offset[0] = -1;
            return;
        }

        int c_y = floor((points_offset[1] - coors_y_min) / voxel_y);
        if (c_y < 0 || c_y >= grid_y) {
            coors_offset[0] = -1;
            coors_offset[1] = -1;
            return;
        }

        int c_z = floor((points_offset[2] - coors_z_min) / voxel_z);
        if (c_z < 0 || c_z >= grid_z) {
            coors_offset[0] = -1;
            coors_offset[1] = -1;
            coors_offset[2] = -1;
        } else {
            coors_offset[0] = c_z;
            coors_offset[1] = c_y;
            coors_offset[2] = c_x;
        }
    }
}


void dynamic_voxelize_gpu(const at::Tensor& points, at::Tensor& coors,
                          const std::vector<float> voxel_size,
                          const std::vector<float> coors_range,
                          const int NDim = 3);
void testDynamicVoxel();


// int hard_voxelize_gpu(const at::Tensor& points, at::Tensor& voxels,
//                       at::Tensor& coors, at::Tensor& num_points_per_voxel,
//                       const std::vector<float> voxel_size,
//                       const std::vector<float> coors_range,
//                       const int max_points, const int max_voxels,
//                       const int NDim = 3);

template <typename T_int>
__global__ void point_to_voxelidx_kernel(const T_int* coor,
                                         T_int* point_to_voxelidx,
                                         T_int* point_to_pointidx,
                                         const int max_points,
                                         const int max_voxels,
                                         const int num_points, const int NDim) {
    CUDA_1D_KERNEL_LOOP(index, num_points) {
        auto coor_offset = coor + index * NDim;
        // skip invalid points
        if ((index >= num_points) || (coor_offset[0] == -1)) return;

        int num = 0;
        int coor_x = coor_offset[0];
        int coor_y = coor_offset[1];
        int coor_z = coor_offset[2];
        for (int i = 0; i < index; ++i) {
            auto prev_coor = coor + i * NDim;
            if (prev_coor[0] == -1) continue;

            if ((prev_coor[0] == coor_x) && (prev_coor[1] == coor_y) &&
                (prev_coor[2] == coor_z)) {
                num++;
                if (num == 1) {
                    point_to_pointidx[index] = i;
                } else if (num >= max_points) {
                    return;
                }
            }
        }
        if (num == 0) {
            point_to_pointidx[index] = index;
        }
        if (num < max_points) {
            point_to_voxelidx[index] = num;
        }
    }
}

template <typename T_int>
__global__ void determin_voxel_num(
        T_int* num_points_per_voxel, T_int* point_to_voxelidx,
        T_int* point_to_pointidx, T_int* coor_to_voxelidx, T_int* voxel_num,
        const int max_points, const int max_voxels, const int num_points) {
    for (int i = 0; i < num_points; ++i) {
        // if (coor[i][0] == -1)
        //    continue;
        int point_pos_in_voxel = point_to_voxelidx[i];
        // record voxel
        if (point_pos_in_voxel == -1) {
            continue;
        } else if (point_pos_in_voxel == 0) {
            // record new voxel
            int voxelidx = voxel_num[0];
            if (voxel_num[0] >= max_voxels) continue;
            voxel_num[0] += 1;
            coor_to_voxelidx[i] = voxelidx;
            num_points_per_voxel[voxelidx] = 1;
        } else {
            int point_idx = point_to_pointidx[i];
            int voxelidx = coor_to_voxelidx[point_idx];
            if (voxelidx != -1) {
                coor_to_voxelidx[i] = voxelidx;
                num_points_per_voxel[voxelidx] += 1;
            }
        }
    }
}


template <typename T, typename T_int>
__global__ void assign_point_to_voxel(const int nthreads, const T* points,
                                      T_int* point_to_voxelidx,
                                      T_int* coor_to_voxelidx, T* voxels,
                                      const int max_points,
                                      const int num_features,
                                      const int num_points, const int NDim) {
    CUDA_1D_KERNEL_LOOP(thread_idx, nthreads) {
        // const int index = blockIdx.x * threadsPerBlock + threadIdx.x;
        int index = thread_idx / num_features;

        int num = point_to_voxelidx[index];
        int voxelidx = coor_to_voxelidx[index];
        if (num > -1 && voxelidx > -1) {
            auto voxels_offset =
                    voxels + voxelidx * max_points * num_features + num * num_features;

            int k = thread_idx % num_features;
            voxels_offset[k] = points[thread_idx];
        }
    }
}

template <typename T, typename T_int>
__global__ void assign_voxel_coors(const int nthreads, T_int* coor,
                                   T_int* point_to_voxelidx,
                                   T_int* coor_to_voxelidx, T_int* voxel_coors,
                                   const int num_points, const int NDim) {
    CUDA_1D_KERNEL_LOOP(thread_idx, nthreads) {
        // const int index = blockIdx.x * threadsPerBlock + threadIdx.x;
        // if (index >= num_points) return;
        int index = thread_idx / NDim;
        int num = point_to_voxelidx[index];
        int voxelidx = coor_to_voxelidx[index];
        if (num == 0 && voxelidx > -1) {
            auto coors_offset = voxel_coors + voxelidx * NDim;
            int k = thread_idx % NDim;
            coors_offset[k] = coor[thread_idx];
        }
    }
}

struct VoxelInfo{
    torch::Tensor voxel;
    torch::Tensor coors;
    torch::Tensor num_points_per_voxel;
};


// VoxelInfo generate_voxel(torch::Tensor &points,
//                          const std::vector<float> &voxel_size,
//                          const std::vector<float> &coors_range,
//                          const int max_point,
//                          const int max_voxel,
//                          const int NDim);



/*==== scatter points =====*/
typedef enum { SUM = 0, MEAN = 1, MAX = 2 } reduce_t;


std::vector<torch::Tensor> dynamic_point_to_voxel_forward_gpu(const torch::Tensor &feats,
                                                              const torch::Tensor &coors,
                                                              const reduce_t reduce_type);

// Dynamic
torch::Tensor generate_voxel(torch::Tensor &points,
                             const std::vector<float> &voxel_size,
                             const std::vector<float> &coors_range,
                             const int NDim = 3);
std::vector<at::Tensor> DynamicSimpleVFE(torch::Tensor &points,
                                         torch::Tensor &coors );

#endif //SECOND_DETECTOR_VOXELIZE_H
