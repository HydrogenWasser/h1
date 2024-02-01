 
#ifndef PARAMS_H_
#define PARAMS_H_
#define MAX_VOXELS 60000
class ppParams
{
  public:
    static const int num_classes = 7;
    const char *class_name [num_classes] = { "car","bicycle","bus","tricycle","pedestrian","semitrailer","truck",};
    const float min_x_range = -99.36;
    const float max_x_range = 99.36;
    const float min_y_range = -99.2;
    const float max_y_range = 99.2;
    const float min_z_range = -7.0;
    const float max_z_range = -1.0;
    // the size of a pillar
    const float pillar_x_size = 0.46;
    const float pillar_y_size = 0.4;
    const float pillar_z_size = 6.0;
    const int max_num_points_per_pillar = 32;
    const int num_point_values = 4;
    // the number of feature maps for pillar scatter
    const int num_feature_scatter = 64;
    const float dir_offset = 0.78539;
    const float dir_limit_offset = 0.0;
    // the num of direction classes(bins)
    const int num_dir_bins = 2;
    // anchors decode by (x, y, z, dir)
    static const int num_anchors = num_classes * 2;
    static const int len_per_anchor = 4;
    const float anchors[num_anchors * len_per_anchor] = {
      4.31,1.8,1.59,0.0,
      4.31,1.8,1.59,1.57,
      1.76,0.68,1.68,0.0,
      1.76,0.68,1.68,1.57,
      10.96,2.95,3.24,0.0,
      10.96,2.95,3.24,1.57,
      2.73,1.43,1.89,0.0,
      2.73,1.43,1.89,1.57,
      0.78,0.64,1.73,0.0,
      0.78,0.64,1.73,1.57,
      14.96,3.01,3.91,0.0,
      14.96,3.01,3.91,1.57,
      7.2,2.53,3.08,0.0,
      7.2,2.53,3.08,1.57,
      };
    const float anchor_bottom_heights[num_classes] = {-1.78,-0.6,-1.78,-0.6,-0.6,-1.78,-1.78,};
    // the score threshold for classification
    const float score_thresh = 0.3;
    const float nms_thresh = 0.1;
    const int max_num_pillars = MAX_VOXELS;
    const int pillarPoints_bev = max_num_points_per_pillar * max_num_pillars;
    // the detected boxes result decode by (x, y, z, w, l, h, yaw)
    const int num_box_values = 7;
    // the input size of the 2D backbone network
    // const int grid_x_size = (max_x_range - min_x_range) / pillar_x_size;
    // const int grid_y_size = (max_y_range - min_y_range) / pillar_y_size;
    // const int grid_z_size = (max_z_range - min_z_range) / pillar_z_size;
    const int grid_x_size = 432;
    const int grid_y_size = 496;
    const int grid_z_size = 1;
    // the output size of the 2D backbone network
    const int feature_x_size = grid_x_size / 2;
    const int feature_y_size = grid_y_size / 2;
    ppParams() {};
};
#endif
