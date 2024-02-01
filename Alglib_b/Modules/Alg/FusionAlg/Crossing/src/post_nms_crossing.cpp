//
// Created by root on 1/6/22.
//

# include "post_nms_crossing.h"

//tlwh_boxes: xyxy
void post_nms_crossing::postNms(xt::xarray<float> &tlwh_boxes,
                                xt::xarray<float> &scores,
                                xt::xarray<float> &class_idxs) {

    remove_big_box_in_the_images(tlwh_boxes, scores, class_idxs);
    remove_people_in_the_vehicle(tlwh_boxes, scores, class_idxs);
    remove_people_above_bicycle_motor(tlwh_boxes, scores, class_idxs);


}
void post_nms_crossing::remove_big_box_in_the_images(xt::xarray<float> &tlwh_boxes,
                                                    xt::xarray<float> &scores,
                                                    xt::xarray<float> &class_idxs)
{
    std::vector<int> indices(int(scores.size()), 1);//box指示索引默认为1 车中人对应索引为0
    if (tlwh_boxes.shape(0) != 0) {
        for (int i = 0; i < tlwh_boxes.shape(0); i++) {
            xt::xarray<float> box_for_area = xt::view(tlwh_boxes, i);
            float box_area = (box_for_area(2)-box_for_area(0))*(box_for_area(3)-box_for_area(1));
            if (box_area > (640*640/(2.5)))
            {
                indices[i] = 0;
            }
        }
    }
    xt::xarray<float> out_boxes = xt::empty<float>({0, 4});
    xt::xarray<float> out_score = xt::empty<float>({0, 1});
    xt::xarray<float> out_class_idxs = xt::empty<float>({0, 1});
    for (int i = 0; i < tlwh_boxes.shape(0); i++)
    {
        if (indices[i] == 1)
        {
            xt::xarray<float> bbox = xt::view(tlwh_boxes, i);
            xt::xarray<float> sscore = scores(i);
            xt::xarray<float> cclass_idxs = class_idxs(i);

            bbox = bbox.reshape({1, bbox.size()});
            sscore = sscore.reshape({1, 1});
            cclass_idxs = cclass_idxs.reshape({1, 1});

            out_boxes = xt::concatenate(xt::xtuple(out_boxes, bbox), 0);
            out_score = xt::concatenate(xt::xtuple(out_score, sscore), 0);
            out_class_idxs = xt::concatenate(xt::xtuple(out_class_idxs, cclass_idxs), 0);
        }
    }

    out_class_idxs = out_class_idxs.reshape({-1});
    out_score = out_score.reshape({-1});
    tlwh_boxes = out_boxes;
    scores = out_score;
    class_idxs = out_class_idxs;

}

void post_nms_crossing::remove_people_in_the_vehicle(xt::xarray<float> &tlwh_boxes,
                                                    xt::xarray<float> &scores,
                                                    xt::xarray<float> &class_idxs)
{
    std::vector<int> indices(int(scores.shape(0)), 1);//box指示索引默认为1 车中人对应索引为0
    auto not_people_boxes_index = xt::where(class_idxs > 0)[0];//不为人的box索引
    xt::xarray<float> not_people_boxes = xt::zeros<float>({int(not_people_boxes_index.size()), 4});//不为人的box
    for (int i = 0; i < not_people_boxes_index.size(); ++i) {
        xt::view(not_people_boxes, i) = xt::view(tlwh_boxes, not_people_boxes_index[i]);
    }//在tlwh_boxes挑出不为人的box

    if (tlwh_boxes.shape(0) != 0) {
        for (int i = 0; i < tlwh_boxes.shape(0); i++) {
            xt::xarray<float> bbox = xt::view(tlwh_boxes, i);//
            float class_id = class_idxs(i);//
            if (int(class_id) == 0)//people目标
            {
                xt::xarray<float> this_people_box = bbox;
                for (int j = 0; j < not_people_boxes.shape(0); j++) {
                    xt::xarray<float> this_not_people_box = xt::view(not_people_boxes, j);
                    if ((this_not_people_box[0] < this_people_box[0]) &&
                        ((this_not_people_box[1] - 50) < this_people_box[1]) &&//this_not_people_box[1] - 50
                        (this_not_people_box[2] > this_people_box[2]) &&
                        (this_not_people_box[3] > this_people_box[3])) {
                        indices[i] = 0;  //人的框四个坐标小于当前目标框，人目标删除  索引值为0
                    }
                }
            }
        }
    }
    xt::xarray<float> out_boxes = xt::empty<float>({0, 4});
    xt::xarray<float> out_score = xt::empty<float>({0, 1});
    xt::xarray<float> out_class_idxs = xt::empty<float>({0, 1});
    for (int i = 0; i < tlwh_boxes.shape(0); i++) {
        if (indices[i] == 1) {
            xt::xarray<float> bbox = xt::view(tlwh_boxes, i);
            xt::xarray<float> sscore = scores(i);
            xt::xarray<float> cclass_idxs = class_idxs(i);

            bbox = bbox.reshape({1, bbox.size()});
            sscore = sscore.reshape({1, 1});
            cclass_idxs = cclass_idxs.reshape({1, 1});

            out_boxes = xt::concatenate(xt::xtuple(out_boxes, bbox), 0);
            out_score = xt::concatenate(xt::xtuple(out_score, sscore), 0);
            out_class_idxs = xt::concatenate(xt::xtuple(out_class_idxs, cclass_idxs), 0);
        }
    }

    out_class_idxs = out_class_idxs.reshape({out_class_idxs.size()});
    out_score = out_score.reshape({out_score.size()});
    tlwh_boxes = out_boxes;
    scores = out_score;
    class_idxs = out_class_idxs;
}


void post_nms_crossing::remove_people_above_bicycle_motor(xt::xarray<float> &tlwh_boxes,
                                                         xt::xarray<float> &scores,
                                                         xt::xarray<float> &class_idxs) 
{
    std::vector<int> indices(int(scores.size()), 1);//box指示索引默认为1 车中人对应索引为0
    if (tlwh_boxes.shape(0) != 0) 
    {
        for (int i = 0; i < tlwh_boxes.shape(0); i++) 
        {
            float class_id_i = class_idxs(i);//
            if (int(class_id_i) == 0)//people目标
            {
                xt::xarray<float> this_people_box = xt::view(tlwh_boxes, i);//
                for (int j = 0; j < tlwh_boxes.shape(0); j++) 
                {
                    float class_id_j = class_idxs(j);//
                    if ((class_id_j == 1) || (class_id_j == 2)||(class_id_j == 8)||(class_id_j == 9)||(class_id_j == 10)||(class_id_j == 11)||(class_id_j == 12)||(class_id_j == 13)||(class_id_j == 14))//非机动车目标类型
                    {
                        xt::xarray<float> this_not_people_box = xt::view(tlwh_boxes, j);
                        if (((this_not_people_box[1] <= this_people_box[3]) &&
                             (this_people_box[3] <= this_not_people_box[3])) &&
                            (((this_not_people_box[0] <= this_people_box[2]) &&
                              (this_people_box[2] <= this_not_people_box[2])) ||
                             ((this_not_people_box[0] <= this_people_box[0]) &&
                              (this_people_box[0] <= this_not_people_box[2]))) ||
                            (((this_people_box[0] <= this_not_people_box[2]) &&
                              (this_not_people_box[2] <= this_people_box[2])) ||
                             ((this_people_box[0] <= this_not_people_box[0]) &&
                              (this_not_people_box[0] <= this_people_box[2])))) {
                            indices[i] = 0;
                            tlwh_boxes[i, 1] = (tlwh_boxes[i, 1] + tlwh_boxes[j, 1]) / 2;
                            class_idxs[j] = 1;

                        }
                    }
                }
            }
        }
    }
    xt::xarray<float> out_boxes = xt::empty<float>({0, 4});
    xt::xarray<float> out_score = xt::empty<float>({0, 1});
    xt::xarray<float> out_class_idxs = xt::empty<float>({0, 1});
    for (int i = 0; i < tlwh_boxes.shape(0); i++) 
    {
        if (indices[i] == 1) 
        {
            xt::xarray<float> bbox = xt::view(tlwh_boxes, i);
            xt::xarray<float> sscore = scores(i);
            xt::xarray<float> cclass_idxs = class_idxs(i);

            bbox = bbox.reshape({1, bbox.size()});
            sscore = sscore.reshape({1, 1});
            cclass_idxs = cclass_idxs.reshape({1, 1});

            out_boxes = xt::concatenate(xt::xtuple(out_boxes, bbox), 0);
            out_score = xt::concatenate(xt::xtuple(out_score, sscore), 0);
            out_class_idxs = xt::concatenate(xt::xtuple(out_class_idxs, cclass_idxs), 0);
        }
    }
    out_class_idxs = out_class_idxs.reshape({-1});
    out_score = out_score.reshape({-1});
    tlwh_boxes = out_boxes;
    scores = out_score;
    class_idxs = out_class_idxs;
}
