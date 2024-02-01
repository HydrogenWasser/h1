
#ifndef POSTPROCESS_KERNELS_H_
#define POSTPROCESS_KERNELS_H_

cudaError_t postprocess_launch(const float *cls_input,
                               float *box_input,
                               const float *dir_cls_input,
                               float *anchors,
                               float *anchor_bottom_heights,
                               float *bndbox_output,
                               int *object_counter,
                               const float min_x_range,
                               const float max_x_range,
                               const float min_y_range,
                               const float max_y_range,
                               const int feature_x_size,
                               const int feature_y_size,
                               const int num_anchors,
                               const int num_classes,
                               const int num_box_values,
                               const float score_thresh,
                               const float dir_offset,
                               cudaStream_t stream = 0);

#endif
