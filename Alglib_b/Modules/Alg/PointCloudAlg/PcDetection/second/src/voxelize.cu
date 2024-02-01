//
// Created by root on 1/8/22.
//

#include "voxelize.h"
#include <ATen/ATen.h>
#include <ATen/cuda/CUDAContext.h>
#include <torch/types.h>
//#include <ATen/cuda/CUDAApplyUtils.cuh>

int const maxGridDim = 50000;

__device__ __forceinline__ static void reduceMax(float *address, float val) {
    int *address_as_i = reinterpret_cast<int *>(address);
    int old = *address_as_i, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_i, assumed,
                        __float_as_int(fmaxf(val, __int_as_float(assumed))));
    } while (assumed != old || __int_as_float(old) < val);
}

__device__ __forceinline__ static void reduceMax(double *address, double val) {
    unsigned long long *address_as_ull =
            reinterpret_cast<unsigned long long *>(address);
    unsigned long long old = *address_as_ull, assumed;
    do {
        assumed = old;
        old = atomicCAS(
                address_as_ull, assumed,
                __double_as_longlong(fmax(val, __longlong_as_double(assumed))));
    } while (assumed != old || __longlong_as_double(old) < val);
}

// get rid of meaningless warnings when compiling host code
#ifdef __CUDA_ARCH__

__device__ __forceinline__ static void reduceAdd(float *address, float val) {
#if (__CUDA_ARCH__ < 200)
#warning \
    "compute capability lower than 2.x. fall back to use CAS version of atomicAdd for float32"
    int *address_as_i = reinterpret_cast<int *>(address);
    int old = *address_as_i, assumed;
    do {
      assumed = old;
      old = atomicCAS(address_as_i, assumed,
                      __float_as_int(val + __int_as_float(assumed)));
    } while (assumed != old);
#else
    atomicAdd(address, val);
#endif
}

__device__ __forceinline__ static void reduceAdd(double *address, double val) {
#if (__CUDA_ARCH__ < 600)
#warning \
    "compute capability lower than 6.x. fall back to use CAS version of atomicAdd for float64"
    unsigned long long *address_as_ull =
        reinterpret_cast<unsigned long long *>(address);
    unsigned long long old = *address_as_ull, assumed;
    do {
      assumed = old;
      old = atomicCAS(address_as_ull, assumed,
                      __double_as_longlong(val + __longlong_as_double(assumed)));
    } while (assumed != old);
#else
    atomicAdd(address, val);
#endif
}
#endif



void dynamic_voxelize_gpu(const at::Tensor& points, at::Tensor& coors,
                          const std::vector<float> voxel_size,
                          const std::vector<float> coors_range,
                          const int NDim) {
    // check device
    CHECK_INPUT(points);

    at::cuda::CUDAGuard device_guard(points.device());

    const int num_points = points.size(0);
    const int num_features = points.size(1);

    const float voxel_x = voxel_size[0];
    const float voxel_y = voxel_size[1];
    const float voxel_z = voxel_size[2];
    const float coors_x_min = coors_range[0];
    const float coors_y_min = coors_range[1];
    const float coors_z_min = coors_range[2];
    const float coors_x_max = coors_range[3];
    const float coors_y_max = coors_range[4];
    const float coors_z_max = coors_range[5];

    const int grid_x = round((coors_x_max - coors_x_min) / voxel_x);
    const int grid_y = round((coors_y_max - coors_y_min) / voxel_y);
    const int grid_z = round((coors_z_max - coors_z_min) / voxel_z);

//    const int col_blocks = at::cuda::ATenCeilDiv(num_points, threadsPerBlock);
    const int col_blocks = ATenCeilDiv(num_points, threadsPerBlock);
    dim3 blocks(col_blocks);
    dim3 threads(threadsPerBlock);
    cudaStream_t stream = at::cuda::getCurrentCUDAStream();

//    dynamic_voxelize_kernel<scalar_t, int><<<blocks, threads, 0, stream>>>(
//            points.contiguous().data_ptr<scalar_t>(),
//            coors.contiguous().data_ptr<int>(), voxel_x, voxel_y, voxel_z,
//            coors_x_min, coors_y_min, coors_z_min, coors_x_max, coors_y_max,
//            coors_z_max, grid_x, grid_y, grid_z, num_points, num_features, NDim);
    AT_DISPATCH_ALL_TYPES(points.scalar_type(), "dynamic_voxelize_kernel", [&] {
        dynamic_voxelize_kernel<scalar_t, int><<<blocks, threads, 0, stream>>>(
                points.contiguous().data_ptr<scalar_t>(),
                coors.contiguous().data_ptr<int>(), voxel_x, voxel_y, voxel_z,
                coors_x_min, coors_y_min, coors_z_min, coors_x_max, coors_y_max,
                coors_z_max, grid_x, grid_y, grid_z, num_points, num_features, NDim);
    });
    cudaDeviceSynchronize();
    AT_CUDA_CHECK(cudaGetLastError());

    return;
}

template<typename T>
__global__ void
feats_reduce_kernel(const T *feats, const int32_t *coors_map,
                    T *reduced_feats, // shall be 0 at initialization
                    const int num_input, const int num_feats,
                    const reduce_t reduce_type) {
    for (int x = blockIdx.x * blockDim.x + threadIdx.x; x < num_input;
         x += gridDim.x * blockDim.x) {
        int32_t reduce_to = coors_map[x];
        if (reduce_to == -1) continue;

        const T *feats_offset = feats + x * num_feats;
        T *reduced_feats_offset = reduced_feats + reduce_to * num_feats;
        if (reduce_type == reduce_t::MAX) {
            for (int i = 0; i < num_feats; i++) {
                reduceMax(&reduced_feats_offset[i], feats_offset[i]);
            }
        } else {
            for (int i = 0; i < num_feats; i++) {
                reduceAdd(&reduced_feats_offset[i], feats_offset[i]);
            }
        }
    }
}

std::vector<at::Tensor> dynamic_point_to_voxel_forward_gpu(
        const at::Tensor &feats, const at::Tensor &coors,
        const reduce_t reduce_type) {
    CHECK_INPUT(feats);
    CHECK_INPUT(coors);

    const int num_input = feats.size(0);
    const int num_feats = feats.size(1);

    if (num_input == 0)
        return {feats.clone().detach(),
                coors.clone().detach(),
                coors.new_empty({0}, torch::kInt32),
                coors.new_empty({0}, torch::kInt32)};

    at::Tensor out_coors;
    at::Tensor coors_map;
    at::Tensor reduce_count;

    auto coors_clean = coors.masked_fill(coors.lt(0).any(-1, true), -1);

    std::tie(out_coors, coors_map, reduce_count) =
            at::unique_dim(coors_clean, 0, true, true, true);

    if (out_coors.index({0, 0}).lt(0).item<bool>()) {
        // the first element of out_coors (-1,-1,-1) and should be removed
        out_coors = out_coors.slice(0, 1);
        reduce_count = reduce_count.slice(0, 1);
        coors_map = coors_map - 1;
    }

    coors_map = coors_map.to(torch::kInt32);
    reduce_count = reduce_count.to(torch::kInt32);

    auto reduced_feats =
            at::empty({out_coors.size(0), num_feats}, feats.options());

    AT_DISPATCH_FLOATING_TYPES(
            feats.scalar_type(), "feats_reduce_kernel", ([&] {
        if (reduce_type == reduce_t::MAX)
            reduced_feats.fill_(-std::numeric_limits<scalar_t>::infinity());
        else
            reduced_feats.fill_(static_cast<scalar_t>(0));

        dim3 blocks(std::min(ATenCeilDiv(num_input, threadsPerBlock),
                             maxGridDim));
        dim3 threads(threadsPerBlock);
        feats_reduce_kernel<<<blocks, threads>>>(
                feats.data_ptr<scalar_t>(), coors_map.data_ptr<int32_t>(),
                reduced_feats.data_ptr<scalar_t>(), num_input, num_feats, reduce_type);
        if (reduce_type == reduce_t::MEAN)
            reduced_feats /= reduce_count.unsqueeze(-1).to(reduced_feats.dtype());
    }));
    AT_CUDA_CHECK(cudaGetLastError());

    return {reduced_feats, out_coors};
}


torch::Tensor generate_voxel(torch::Tensor &points,
                             const std::vector<float> &voxel_size,
                             const std::vector<float> &coors_range,
                             const int NDim){
    torch::Tensor coors = points.new_zeros({points.size(0), 3},
                                           torch::TensorOptions().dtype(torch::kInt).device(torch::kCUDA, 0));
    dynamic_voxelize_gpu(points, coors, voxel_size, coors_range, 3);
    return coors;
}
std::vector<at::Tensor> DynamicSimpleVFE(torch::Tensor &voxels,
                                         torch::Tensor &coors){
    assert(coors.size(1) == 3 && voxels.size(0) == coors.size(0));
    std::vector<at::Tensor> res = dynamic_point_to_voxel_forward_gpu(voxels, coors, MEAN);
    assert(res.size() == 2);
    return res;
}


