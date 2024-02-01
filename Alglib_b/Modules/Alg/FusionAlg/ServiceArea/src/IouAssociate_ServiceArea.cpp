
#include "IouAssociate_ServiceArea.h"
#include "FunctionHub_ServiceArea.h"


IouAssociate_ServiceArea::IouAssociate_ServiceArea()
{

}


IouAssociate_ServiceArea::~IouAssociate_ServiceArea()
{

}


void IouAssociate_ServiceArea::pre_process_associate_data(xt::xarray<float>     *_dets,
                                                          xt::xarray<float>     *_trks,
                                                          int                   flag,
                                                          nlohmann::json        *parameter)
{
    m_flag = flag;

    xt::xarray<float> &dets = *_dets;
    xt::xarray<float> &trks = *_trks;

    // 是否是雷达IOU匹配
    if (flag == 1) // 雷达距离匹配
    {
        float width_coe = (*parameter)["fusion_param"]["width_coe"];
        xt::view(dets, xt::all(), 2) *= width_coe; // z值乘以缩放因子
        // 北京版本计算iou的rotate_nms_cc_gpu，数据顺序：x,y,w,l,angle
        m_detections = xt::zeros<float>({int(dets.shape(0)), 5}); // x, y, w, l, theta
        m_trackers = xt::zeros<float>({int(trks.shape(0)), 5});   // x, y, w, l, theta

        std::vector<int> cols1 = {0, 1, 2, 3, 4};
        std::vector<int> cols2 = {0, 1, 3, 4, 6};
        for (int i = 0; i < int(cols1.size()); i++)
        {
            xt::view(m_detections, xt::all(), cols1[i]) = xt::view(dets, xt::all(), cols1[i]);
            xt::view(m_trackers, xt::all(), cols1[i]) = xt::view(trks, xt::all(), cols2[i]);
        }
        // 角度转弧度
        adjust_boxes_iou_bev_input(m_detections); // multi-sensor track.py 1905行
        adjust_boxes_iou_bev_input(m_trackers);

        xt::view(dets, xt::all(), 2) /= width_coe; // z值乘缩放因子
    }
    
    // 相机IOU匹配
    if (m_flag == 0)
    {
        // _dets: x,y,w,l,theta(角度),z,h,label,speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel
        // _trks: x, y,z,w,l,h,theta(角度),id
        m_detections = xt::zeros<float>({int(dets.shape(0)), 5}); // x, y, w, l, theta
        m_trackers = xt::zeros<float>({int(trks.shape(0)), 5});   // x, y, w, l, theta
        std::vector<int> cols1 = {0, 1, 2, 3, 4};
        std::vector<int> cols2 = {0, 1, 3, 4, 6};
        for (int i = 0; i < int(cols1.size()); i++)
        {
            xt::view(m_detections, xt::all(), cols1[i]) = xt::view(dets, xt::all(), cols1[i]);
            xt::view(m_trackers, xt::all(), cols1[i]) = xt::view(trks, xt::all(), cols2[i]);
        }

        // 角度转弧度
        adjust_boxes_iou_bev_input(m_detections);
        adjust_boxes_iou_bev_input(m_trackers);
    }
}


void IouAssociate_ServiceArea::cal_cost_matrix(xt::xarray<float> &cost_matrix)
{
    cost_matrix = FunctionHub_ServiceArea::rotate_nms_cc(m_detections, m_trackers).first;
}

void IouAssociate_ServiceArea::execute_match(xt::xarray<float>              _cost_matrix, 
                                             fusion_match_out_ServiceArea   *associate_out, 
                                             nlohmann::json                 *parameter)
{

    xt::xarray<float> cost_matrix;

    associate_out->_cost_matrix = _cost_matrix;
    cost_matrix = -_cost_matrix;
    auto &cost = cost_matrix;
    auto matched_indices = hungarian_alg_SA(cost);
    associate_out->_matched_indices = matched_indices;
}

void IouAssociate_ServiceArea::post_process_associate_data(fusion_match_out_ServiceArea *associate_out, 
                                                           xt::xarray<float>            *_dets, 
                                                           int                          flag, 
                                                           nlohmann::json               *parameter)
{
    // 把匹配成功的检测索引和轨迹索引用vector存储起来
    std::vector<int> matched_det_index;
    std::vector<int> matched_trk_index;
    
    for (int i = 0; i < int(associate_out->_matched_indices.shape(0)); i++)
    {
        

        matched_det_index.push_back(associate_out->_matched_indices(i, 0));
        matched_trk_index.push_back(associate_out->_matched_indices(i, 1));
    }

    // 得到未匹配到的检测索引
    std::vector<int> unmatched_det_index;
    for (int i = 0; i < int(associate_out->_cost_matrix.shape(0)); i++)
    {
        if (std::find(matched_det_index.begin(), matched_det_index.end(), i) == matched_det_index.end())
        {
            unmatched_det_index.push_back(i);
        }
    }

    // 得到未匹配到的轨迹索引
    std::vector<int> unmatched_trk_index;
    for (int i = 0; i < associate_out->_cost_matrix.shape(1); i++)
    {
        if (std::find(matched_trk_index.begin(), matched_trk_index.end(), i) == matched_trk_index.end())
        {
            unmatched_trk_index.push_back(i);
        }
    }

    matched_det_index.clear();
    matched_trk_index.clear();

    // 得到匹配到的轨迹
    for (int i = 0; i < associate_out->_matched_indices.shape(0); i++)
    {
        int det_index = associate_out->_matched_indices(i, 0);
        int trk_index = associate_out->_matched_indices(i, 1);

        float thresh;
        if (m_flag)
        {
            thresh = float((*parameter)["fusion_param"]["lidar_iouThreshold"]);
        }
        else
        {
            thresh = float((*parameter)["fusion_param"]["camera_iouThreshold"]);
        }

        if (associate_out->_cost_matrix(det_index, trk_index) < thresh)
        {
            unmatched_det_index.push_back(det_index);
            unmatched_trk_index.push_back(trk_index);
        }
        else
        {
            matched_det_index.push_back(det_index);
            matched_trk_index.push_back(trk_index);
        }
    }

    // 得到最终的匹配到的索引数组
    if (int(matched_det_index.size()) == 0)
    {
        associate_out->_matched_indices = xt::empty<int>({0, 2});
    }
    else
    {
        assert(matched_det_index.size() == matched_trk_index.size());
        std::vector<std::size_t> shape = {matched_det_index.size(), 1};

        xt::xarray<int> xt_matched_det_index = xt::adapt(matched_det_index, shape);
        xt::xarray<int> xt_matched_trk_index = xt::adapt(matched_trk_index, shape);

        associate_out->_matched_indices = xt::concatenate(xt::xtuple(xt_matched_det_index, xt_matched_trk_index), 1);
    }

    // 把未匹配到的检测索引和未匹配到的轨迹索引，保存到associate_out
    associate_out->_unmatched_detections_indices = unmatched_det_index;
    associate_out->_unmatched_trackers_indices = unmatched_trk_index;

    associate_out->_matched_detections_indices = matched_det_index;
    associate_out->_matched_trackers_indices = matched_trk_index;

    // 把未匹配上的检测数据，保存到associate_out.unmatched_detections中
    xt::xarray<float> dets = *_dets;
    if (unmatched_det_index.size() > 0)
    {
        xt::xarray<float> unmatched_det = xt::zeros<float>({int(unmatched_det_index.size()), int(dets.shape(1))});
        for (int i = 0; i < unmatched_det_index.size(); i++)
        {
            xt::view(unmatched_det, i, xt::all()) = xt::view(dets, unmatched_det_index[i], xt::all());
        }
        associate_out->_unmatched_detections = unmatched_det;
    }
}

void IouAssociate_ServiceArea::adjust_boxes_iou_bev_input(xt::xarray<float> &origin_box)
{
    xt::view(origin_box, xt::all(), 4) *= (PI / 180);
    xt::view(origin_box, xt::all(), 4) = 1.5 * PI - xt::view(origin_box, xt::all(), 4);
}
