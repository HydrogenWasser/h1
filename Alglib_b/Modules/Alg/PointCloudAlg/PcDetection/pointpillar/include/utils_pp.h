//
// Created by root on 4/8/22.
//
#include <iostream>
#include <memory.h>
#include <dirent.h>
#include<algorithm>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cuda_runtime.h>
//#ifndef CUDA_POINTPILLARS_UTILS_H
//#define CUDA_POINTPILLARS_UTILS_H
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

std::vector<std::string> getFilesList(std::string &dirpath);

int loadData(const char *file, void **data, unsigned int *length);

void Getinfo(void);
//#endif //CUDA_POINTPILLARS_UTILS_H
