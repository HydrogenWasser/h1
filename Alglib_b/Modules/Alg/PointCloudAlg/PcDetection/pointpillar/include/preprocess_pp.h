

#include "params.h"

class PreProcessCuda {
private:
    ppParams params_;
    unsigned int *mask_;
    float *voxels_;
    int *voxelsList_;
    float *params_cuda_;
    cudaStream_t stream_ = 0;

    //points cloud -> voxels (BEV) -> feature*4 by CPU
    int *coor_to_voxelidx_ = nullptr;

public:
    PreProcessCuda(cudaStream_t stream_ = 0);

    ~PreProcessCuda();

    //points cloud -> voxels (BEV) -> feature*4 
    int generateVoxels(float *points, size_t points_size,
                       unsigned int *pillar_num,
                       float *voxel_features,
                       float *voxel_num_points,
                       float *coords);

    //feature*4 -> feature * 10 
    int generateFeatures(float *voxel_features,
                         float *voxel_num_points,
                         float *coords,
                         unsigned int *params,
                         float *features);

    //points cloud -> voxels (BEV) -> feature*4 by CPU
//    int clearCacheCPU(void);
//
//    void generateVoxels_cpu(float *points, size_t points_size,
//                            unsigned int *pillarCount,
//                            float *voxel_features,
//                            float *voxel_num_points,
//                            float *coords);

};

