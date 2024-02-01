#pragma once

#include "config.h"

struct YoloKernel {
  int width;
  int height;
  float anchors[kNumAnchor * 2];
};
struct Yolo_Rect
{
    float x;
    float y;
    float width;
    float height;
};
struct alignas(float) yolov5Detection {
  float bbox[4];  // center_x center_y w h
  float conf;  // bbox_conf * cls_conf
  float class_id;
  float mask[32];
  Yolo_Rect rect;// left_top_x, left_top_y, w, h (1920, 1080)
};

