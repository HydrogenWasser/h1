//
// Created by root on 4/8/22.
//

#include "utils_pp.h"


std::vector<std::string> getFilesList(std::string &dirpath) {
    DIR *dir = opendir(dirpath.c_str());
    if (dir == NULL) {
        std::cout << "open dir error\n";
    }
    std::vector<std::string> all_path;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR) {   // it is a dir
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
                continue;
            }
            std::cout << ">>>> It as a dir path!\n";
            std::string dir_new = entry->d_name;
            std::vector<std::string> temp_path = getFilesList(dir_new);
            all_path.insert(all_path.end(), temp_path.begin(), temp_path.end());
        } else {
            std::string name = entry->d_name;

            all_path.push_back(name);
        }
    }
    closedir(dir);
    std::sort(all_path.begin(), all_path.end());
    return all_path;
}

int loadData(const char *file, void **data, unsigned int *length) {
    std::fstream dataFile(file, std::ifstream::in);

    if (!dataFile.is_open()) {
        std::cout << "Can't open files: " << file << std::endl;
        return -1;
    }

    //get length of file:
    unsigned int len = 0;
    dataFile.seekg(0, dataFile.end);
    len = dataFile.tellg();
    dataFile.seekg(0, dataFile.beg);

    //allocate memory:
    char *buffer = new char[len];
    if (buffer == NULL) {
        std::cout << "Can't malloc buffer." << std::endl;
        dataFile.close();
        exit(-1);
    }

    //read data as a block:
    dataFile.read(buffer, len);
    dataFile.close();

    *data = (void *) buffer;
    *length = len;
    return 0;
}

void Getinfo(void) {
    cudaDeviceProp prop;

    int count = 0;
    cudaGetDeviceCount(&count);
    printf("\nGPU has cuda devices: %d\n", count);
    for (int i = 0; i < count; ++i) {
        cudaGetDeviceProperties(&prop, i);
        printf("----device id: %d info----\n", i);
        printf("  GPU : %s \n", prop.name);
        printf("  Capbility: %d.%d\n", prop.major, prop.minor);
        printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
        printf("  Const memory: %luKB\n", prop.totalConstMem >> 10);
        printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
        printf("  warp size: %d\n", prop.warpSize);
        printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
        printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
        printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
    }
    printf("\n");
}
