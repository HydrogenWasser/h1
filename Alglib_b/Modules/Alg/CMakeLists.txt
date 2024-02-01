project(Alg)
set(BUILD_PC_ALG ON) # 是否编译点云算法
set(BUILD_VIDEO_ALG ON) # 是否编译视频算法
set(BUILD_FUSION_ALG ON) # 是否编译融合算法
set(BUILD_TRIG_ALG ON) # 是否编译触发相机算法
set(BUILD_SCENE_ALG ON) # 是否编译触发相机算法
set(BUILD_EVENT_ALG ON) # 是否编译事件检测算法
add_definitions(-w)

message("CMAKE_SOURCE_DIR= " ${CMAKE_SOURCE_DIR})
add_compile_options(-fvisibility=hidden)
add_compile_options(-fPIC)
add_compile_options(-shared)
add_compile_options(-fext-numeric-literals)

include_directories(${CMAKE_SOURCE_DIR}/Include/Data)
include_directories(${CMAKE_SOURCE_DIR}/Include/Modules/Common)
include_directories(${CMAKE_SOURCE_DIR}/Include/Modules/Alg)
include_directories(${CMAKE_SOURCE_DIR}/Include/Param)
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart)
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/av_opencv)
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/cpp_algorithm)
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/eigen3)
include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/Log)
include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/Include)

# 点云算法头文件包含路径
if(BUILD_PC_ALG)
    enable_language(CUDA) 
    set(CUDNN_DIR /usr/local/)
    find_package(CUDA REQUIRED)
    # include_directories(/usr/local/cuda-11.3/include)
    # link_directories(/usr/local/cuda-11.3/lib64)
    message(STATUS "CUDA version:" ${CUDA_VERSION})
    # message(STATUS "CUDA libs:" ${CUDA_LIBS})
    # message(STATUS "CUDA TOOLKIT_ROOT_DIR:" ${CUDA_TOOLKIT_ROOT_DIR})
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcDetection/second/include)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcDetection/pointpillar/include)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcTrack/include)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcPretreatment)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcPostProcess)
    include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/SSCN)
    include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/SSCN/spconv)
    include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/yaml-cpp)
    include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/libtorch)
    include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/libtorch/torch/csrc/api/include)
    include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart)
    
endif()

# 视频算法头文件包含路径
if(BUILD_VIDEO_ALG)
    enable_language(CUDA) 
    set(CUDNN_DIR /usr/local/)
    find_package(CUDA REQUIRED)
    # include_directories(/usr/local/cuda-11.3/include)
    # link_directories(/usr/local/cuda-11.3/lib64)
    message(STATUS "CUDA version:" ${CUDA_VERSION})
    # message(STATUS "CUDA libs:" ${CUDA_LIBS})
    # message(STATUS "CUDA TOOLKIT_ROOT_DIR:" ${CUDA_TOOLKIT_ROOT_DIR})
    include_directories(${CUDA_TOOLKIT_ROOT_DIR}/include)
    include_directories(${CMAKE_SOURCE_DIR}/Include/ThirdPart/TensorRtInclude)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg/VideoDetection/yolov5/include)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg/VideoDetection/yolov6/include)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg/VideoTrack/include)
endif()

# 事件检测算法头文件包含路径
if(BUILD_EVENT_ALG)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/EventAlg)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/EventAlg/EventDetection/include)
endif()

# 融合算法包含路径
if(BUILD_FUSION_ALG)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/Crossing/include)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/tunnel/include)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/intersection_suyan/include)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/ServiceArea/include)
endif()


# 融合OutPut算法包含路径
# if(BUILD_SCENE_ALG)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/SceneAlg/)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/SceneAlg/include)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/SceneAlg/src HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/SceneAlg HW_SRC_FILES)
    
# endif()




# 触发算法头文件包含路径
if(BUILD_TRIG_ALG)
    include_directories(${CMAKE_SOURCE_DIR}/Modules/Alg/TriggerAlg/)
endif()



# 点云算法源文件
if(BUILD_PC_ALG)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcPretreatment/ HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcDetection/second/src HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcDetection/pointpillar/src HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcTrack/src HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcPostProcess HW_SRC_FILES)  # add by zqj for pc_postprocess
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg HW_SRC_FILES)
endif()



# 视频算法源文件
if(BUILD_VIDEO_ALG)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg/VideoDetection/yolov5/src HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg/VideoDetection/yolov6/src HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg/VideoTrack/src HW_SRC_FILES)
endif()

# 事件检测算法源文件
if(BUILD_EVENT_ALG)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/EventAlg HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/EventAlg/EventDetection/src HW_SRC_FILES)
endif()

# 融合算法源文件
if(BUILD_FUSION_ALG)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/Crossing/src HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/tunnel/src HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/intersection_suyan/src HW_SRC_FILES)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/ServiceArea/src HW_SRC_FILES)
endif()

# 触发算法源文件
if(BUILD_TRIG_ALG)
    aux_source_directory(${CMAKE_SOURCE_DIR}/Modules/Alg/TriggerAlg/ HW_SRC_FILES)
endif()

# 通用链接库路径
link_directories(${CMAKE_SOURCE_DIR}/OutPut/Lib)
link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL)
link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL/Alg)
link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL/av_opencv)

# 点云算法需要链接的库路径

if(BUILD_PC_ALG)
    link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL/Alg/libtorch)
endif()

# 视频算法需要链接的库路径
if(BUILD_VIDEO_ALG)
    link_directories(${CMAKE_SOURCE_DIR}/OutPut/TPL/Alg/TensorRtLib)
endif()

# 直接编译源文件
aux_source_directory(. HW_SRC_FILES)

add_library(${PROJECT_NAME} SHARED ${HW_SRC_FILES} ExportAlgLib.cpp)

# 添加生成动态库的子目录
# add_subdirectory(${CMAKE_SOURCE_DIR}/Modules/Alg/PointCloudAlg/PcDetection/)
# add_subdirectory(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg/VideoDetection/)
# add_subdirectory(${CMAKE_SOURCE_DIR}/Modules/Alg/VideoAlg/VideoTrack/)
# add_subdirectory(${CMAKE_SOURCE_DIR}/Modules/Alg/FusionAlg/CppSrc/)

target_link_libraries(${PROJECT_NAME} PRIVATE opencv_core)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgproc)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_imgcodecs)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_highgui)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_calib3d)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_dnn)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_features2d)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_flann)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_ml)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_objdetect)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_photo)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_stitching)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_video)
target_link_libraries(${PROJECT_NAME} PRIVATE opencv_videoio)

# 点云算法需要链接的库

if(BUILD_PC_ALG)
    target_link_libraries(${PROJECT_NAME} PRIVATE blas)
    target_link_libraries(${PROJECT_NAME} PRIVATE lapack)
    target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE spconv)

    target_link_libraries(${PROJECT_NAME} PRIVATE torch)
    target_link_libraries(${PROJECT_NAME} PRIVATE c10)
    target_link_libraries(${PROJECT_NAME} PRIVATE c10_cuda)
    target_link_libraries(${PROJECT_NAME} PRIVATE caffe2_nvrtc)
    target_link_libraries(${PROJECT_NAME} PRIVATE cudart)
    target_link_libraries(${PROJECT_NAME} PRIVATE fmt)
    target_link_libraries(${PROJECT_NAME} PRIVATE nvToolsExt)
    target_link_libraries(${PROJECT_NAME} PRIVATE nvrtc)
    target_link_libraries(${PROJECT_NAME} PRIVATE nvrtc-builtins)
    target_link_libraries(${PROJECT_NAME} PRIVATE torch_cpu)
    target_link_libraries(${PROJECT_NAME} PRIVATE torch_cuda)
    target_link_libraries(${PROJECT_NAME} PRIVATE torch_global_deps)
endif()

# 视频算法需要链接的库
if(BUILD_VIDEO_ALG)
    target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
    target_link_libraries(${PROJECT_NAME} PRIVATE nvinfer nvinfer_plugin nvparsers nvonnxparser) # TensorRT
endif()