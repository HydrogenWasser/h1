//
// Created by root on 10/24/22.
//

#include "fusion_sort_crossing.h"
#include "hungarian.h"
#define None_zsc -10000.1
#define PI 3.1415926
#define Onecircle 360.0
#define Halfcicle 180.0

FSort_crossing::FSort_crossing(TSelfFusionAlgParam *m_stAlgParam){   
    // 读取跟踪算法配置文件
    YAML::Node fsort_cfg = YAML::LoadFile(m_stAlgParam->m_strRootPath + m_stAlgParam->m_strFusionCfgPath);
    _config_path = m_stAlgParam->m_strRootPath + fsort_cfg["CROSSING_FUSION_CONF_PATH"].as<std::string>();
    _iou_threshold = fsort_cfg["CROSSING_FUSION"]["IOU_THRESHOLD"].as<float>();

    _max_age     = fsort_cfg["CROSSING_FUSION"]["MAX_AGE"].as<int>();
    _min_hits    = fsort_cfg["CROSSING_FUSION"]["MIN_HITS"].as<int>();
    _max_age_new = fsort_cfg["CROSSING_FUSION"]["MAX_AGE_NEW"].as<int>();
    _dis_thresh  = fsort_cfg["CROSSING_FUSION"]["DIS_THRESH"].as<float>();
    _angle_judge = fsort_cfg["CROSSING_FUSION"]["ANGLE_JUDGE"].as<float>();
    _nms_min     = fsort_cfg["CROSSING_FUSION"]["NMS_MIN"].as<float>();
    _nms_max     = fsort_cfg["CROSSING_FUSION"]["NMS_MAX"].as<float>();
    _caldis_A    = fsort_cfg["CROSSING_FUSION"]["CALDIS_A"].as<float>();
    _caldis_B    = fsort_cfg["CROSSING_FUSION"]["CALDIS_B"].as<float>();
    _useLanebmp  = fsort_cfg["CROSSING_FUSION"]["USE_LANE_BMP"].as<bool>();

    //读取车道配置图片
    std::string LaneBmpPath = _config_path + "/lane_bmp/chedao.bmp";
    _lane_bmp = cv::imread(LaneBmpPath,cv::IMREAD_COLOR);

    std::string lane_config_path = _config_path + "/laneconfig.csv";
    _laneParam = csv2Xtensor<float>(lane_config_path, 3, 40);
}

//机动车与非机动车分类距离匹配
std::vector<FUKalmanBoxTracker_crossing> FSort_crossing::distanceMatch(xt::xarray<float> dets, xt::xarray<float>trks, 
                                                                       std::vector<int> unmatched_dets, std::vector<int> unmatched_trks)
{
    std::vector <FUKalmanBoxTracker_crossing> new_un_match_trksbig;//作为大列表同样的地位使用,等同于第一轮的_tracker
    std::vector <FUKalmanBoxTracker_crossing> new_un_match_trkssmall;

    xt::xarray<float> un_match_object = xt::zeros<float>({(int)unmatched_dets.size(), (int)dets.shape(1)});//装入内容
    xt::xarray<float> un_match_track = xt::zeros<float>({(int)unmatched_trks.size(), (int)trks.shape(1)});//装入内容

    for (int j = 0; j < unmatched_dets.size(); ++j) 
    {
        xt::view(un_match_object, j, xt::all()) = xt::view(dets, unmatched_dets[j],
                                                           xt::all());//outXtensorItem_bbox-detections
    }//unmatched_dets unmatched_trks 全域索引
    for (int j = 0; j < unmatched_trks.size(); ++j)
    {
        xt::view(un_match_track, j, xt::all()) = xt::view(trks, unmatched_trks[j], xt::all());//outXtensorItem_bbox-detections
    }
    // un_match_track- un_match_object内容
    //存储大类的检测索引-un_match_object
    std::vector<int> detection_indices_big;
    //存储大类的跟踪索引-un_match_track
    std::vector<int> trackers_indices_big;
    //存储xiao类的检测索引-un_match_object
    std::vector<int> detection_indices_small;
    //存储xiao类的跟踪索引-un_match_track
    std::vector<int> trackers_indices_small;

    /*大类检测个数*/
    for (int i = 0; i < un_match_object.shape(0); i++) 
    {
        if (label_crossing::judge_large_target(int(un_match_object(i, 7))) ) 
        {
            detection_indices_big.push_back(i);
        }
    }
    /* 大类轨迹的个数 */
    for (int i = 0; i < un_match_track.shape(0); i++) 
    {
        if (label_crossing::judge_large_target(int(un_match_track(i, 7)))) 
        {
            trackers_indices_big.push_back(i);
        }
    }

    /*小类检测个数，小类的跟踪没用到，就不写 */
    for (int i = 0; i < un_match_object.shape(0); i++) 
    {
        if (!label_crossing::judge_large_target(int(un_match_object(i, 7)))) 
        {
            detection_indices_small.push_back(i);
        }
    }

    // little track
    for (int i = 0; i < un_match_track.shape(0); i++) 
    {
        if (!label_crossing::judge_large_target(int(un_match_track(i, 7)))) 
        {
            trackers_indices_small.push_back(i);
        }
    }

    //开始把距离匹配需要的输入量准备好
    xt::xarray<float> un_match_object_big = xt::zeros<float>({(int)detection_indices_big.size(), (int)dets.shape(1)});//装入内容，作为第二轮的第一个输入
    xt::xarray<float> un_match_object_small = xt::zeros<float>({(int)detection_indices_small.size(), (int)dets.shape(1)});//装入内容
    xt::xarray<float> un_match_track_big = xt::zeros<float>({(int)trackers_indices_big.size(), (int)trks.shape(1)});//装入内容，作为第二轮的第一个输入
    xt::xarray<float> un_match_track_small = xt::zeros<float>({(int)trackers_indices_small.size(),(int)trks.shape(1)});

    for (int i = 0; i < detection_indices_big.size(); i++) 
    {
        xt::view(un_match_object_big, i, xt::all()) = xt::view(un_match_object, detection_indices_big[i], xt::all());//只能一行一行的装填，还不知道有没有集体装填的语句
    }
    for (int i = 0; i < detection_indices_small.size(); i++) 
    {
        xt::view(un_match_object_small, i, xt::all()) = xt::view(un_match_object, detection_indices_small[i], xt::all());//只能一行一行的装填，还不知道有没有集体装填的语句
    }
    for (int i = 0; i < trackers_indices_big.size(); i++) 
    {   
        xt::view(un_match_track_big, i, xt::all()) = xt::view(un_match_track, trackers_indices_big[i], xt::all());//只能一行一行的装填，还不知道有没有集体装填的语句
    }
    for (int i = 0; i < trackers_indices_small.size(); i++) 
    {
        xt::view(un_match_track_small, i, xt::all()) = xt::view(un_match_track, trackers_indices_small[i], xt::all());//只能一行一行的装填，还不知道有没有集体装填的语句
    }

    //un_match_object_big,un_match_track_big,un_match_object_small
    /*把参加的Ｔ重新存 */
    for (int i = 0; i < trackers_indices_big.size(); i++) 
    {
        int index_t = unmatched_trks[trackers_indices_big[i]];//index_t是具体的值，就是全域索引
        new_un_match_trksbig.push_back(_trackers[index_t]); //其实就是找对全局索引
    }
    for (int i = 0; i < trackers_indices_small.size(); i++) 
    {
        int index_t = unmatched_trks[trackers_indices_small[i]];//index_t是具体的值，就是全域索引
        new_un_match_trkssmall.push_back(_trackers[index_t]); //其实就是找对全局索引
    }

    //big target
    auto match_res1 = FSort_crossing::associate_detections_to_trackers(un_match_object_big, un_match_track_big, 2);
    auto matched_bak1 = match_res1.matches;//都是基于输入内容的索引
    auto unmatched_dets_bak1 = match_res1.unmatched_detections_vector;
    auto unmatched_trks_bak1 = match_res1.unmatched_trackers_vector;
    auto cost_matrix1 = match_res1.cost_matrix;
    //small target
    auto match_res2 = FSort_crossing::associate_detections_to_trackers(un_match_object_small, un_match_track_small, 3);
    auto matched_bak2 = match_res2.matches;//都是基于输入内容的索引
    auto unmatched_dets_bak2 = match_res2.unmatched_detections_vector;
    auto unmatched_trks_bak2 = match_res2.unmatched_trackers_vector;
    auto cost_matrix2 = match_res2.cost_matrix;

    //unmatchtrk_for_video
    //    xt::xarray<float> unmatchTrkAfterkm=xt::zeros<float>({int(unmatched_trks_bak1.size()+unmatched_trks_bak2.size()), 11});
    std::vector<FUKalmanBoxTracker_crossing> unmatchTrkAfterkm;
    int count_untrk = 0;
    for (int i = 0; i < new_un_match_trksbig.size(); i++) 
    {
        if (std::find(unmatched_trks_bak1.begin(), unmatched_trks_bak1.end(), i) == unmatched_trks_bak1.end())//找到最后，也没找到i在ut中,说明是匹配的t
        {
            auto index = xt::where(xt::col(matched_bak1, 1) > i - 1 && xt::col(matched_bak1, 1) < i + 1)[0];//index==行,
            new_un_match_trksbig[i].update(xt::view(un_match_object_big, matched_bak1(index[0],0)));// xt::view(dets, matched(index[0], 0))--整个是一个输入量
        }
        else
        {
            unmatchTrkAfterkm.push_back(new_un_match_trksbig[i]);
        }
    }

    /* 大类中没有匹配的检测*/
    for (int i = 0; i < unmatched_dets_bak1.size(); i++) 
    {
        xt::xarray<float> bbox_for_kalma1 = xt::view(un_match_object_big, unmatched_dets_bak1[i], xt::all());
        _trackers.push_back(FUKalmanBoxTracker_crossing(bbox_for_kalma1));
    }
    //small label

    for (int i = 0; i < new_un_match_trkssmall.size(); i++) 
    {
        if (std::find(unmatched_trks_bak2.begin(), unmatched_trks_bak2.end(), i) == unmatched_trks_bak2.end())//找到最后，也没找到i在ut中,说明是匹配的t
        {
            auto index = xt::where(xt::col(matched_bak2, 1) > i - 1 && xt::col(matched_bak2, 1) < i + 1)[0];//index==行,
            new_un_match_trkssmall[i].update(xt::view(un_match_object_small, matched_bak2(index[0],0)));// xt::view(dets, matched(index[0], 0))--整个是一个输入量
        }else
        {
            unmatchTrkAfterkm.push_back(new_un_match_trkssmall[i]);
        }
    }

    for (int i = 0; i < unmatched_dets_bak2.size(); i++) 
    {
        xt::xarray<float> bbox_for_kalma2 = xt::view(un_match_object_small, unmatched_dets_bak2[i], xt::all());
        _trackers.push_back(FUKalmanBoxTracker_crossing(bbox_for_kalma2));//KalmanBoxTracker(bbox_for_kalma)-初始化之后目标，放到Sort 的大列表中
    }
    return unmatchTrkAfterkm;
}


//点云检测与预测目标关联:IOU / distance
fumatch_out_crossing FSort_crossing::associate_detections_to_trackers(xt::xarray<float> &detections, xt::xarray<float> &trackers, int flag) 
{
    fumatch_out_crossing associate_out;
    //track empty
    // flag 0:iou 1:dis_car 2:dis_bicycle
    if (!trackers.size()) 
    {
        associate_out.matches = xt::empty<int>({0, 2});
        associate_out.unmatched_detections = xt::arange<int>(0, detections.shape(0));
        associate_out.unmatched_trackers = xt::empty<int>({0, 7});
        associate_out.cost_matrix = xt::xarray<float>({0});
        for (int i = 0; i < detections.shape(0); ++i) 
        {
            associate_out.unmatched_detections_vector.push_back(i);
        }
        return associate_out;
    } else if (!detections.size()) 
    {
        associate_out.matches = xt::empty<int>({0, 2});
        associate_out.unmatched_detections = xt::empty<int>({0, 9});
        associate_out.unmatched_trackers = xt::empty<int>({0, 7});
        associate_out.cost_matrix = xt::xarray<float>({0});
        for (int i = 0; i < trackers.shape(0); ++i) 
        {
            associate_out.unmatched_trackers_vector.push_back(i);
        }
        return associate_out;
    }

    xt::xarray<float> cost_matrix;
    xt::xarray<float> iou_matrix;
    xt::xarray<float> iou_matrix_new;
    xt::xarray<float> L_distance = xt::zeros<float> ({(int)detections.shape(0),(int)trackers.shape(0)});
    xt::xarray<float> N_distance = xt::zeros<float> ({(int)detections.shape(0),(int)trackers.shape(0)});
    xt::xarray<float> O_distance = xt::zeros<float> ({(int)detections.shape(0),(int)trackers.shape(0)});
    //计算IOU
    if (flag == 1)
    {
        auto iou_res = box_ops_crossing::rotate_nms_cc(detections, trackers);

        iou_matrix = iou_res.first;
        iou_matrix_new = iou_res.second;
        cost_matrix = -iou_matrix;
    }
    else
    {
        cal_distance(detections, trackers, flag, L_distance,N_distance,O_distance);
        cost_matrix = -O_distance;
    }
    /** ================== KM alg ======================*/
    //        xt::dump_npy("./pc_cost_matrix.npy",cost_matrix);
    // xt::xarray<float> temp_cost = cost_matrix;

    // auto hungarian7 = std::chrono::high_resolution_clock::now();
    auto matched_indices = run_hungarian_match1(cost_matrix);

    // auto hungarian8 = std::chrono::high_resolution_clock::now();

    // auto hungarian5 = std::chrono::high_resolution_clock::now();
    // auto matched_indices = run_hungarian_match(cost_matrix);
    // auto hungarian6 = std::chrono::high_resolution_clock::now();

    // auto run_hungarian3 = std::chrono::duration<double, std::milli>(hungarian6 - hungarian5).count();
    // auto run_hungarian4 = std::chrono::duration<double, std::milli>(hungarian8 - hungarian7).count();
    // std::cout << "run_hungarian3_time:" << run_hungarian3 << " ms. cost_matrix:" << cost_matrix.shape(0) <<","<< cost_matrix.shape(1) <<std::endl;
    // std::cout << "run_hungarian4_time:" << run_hungarian4 << " ms. cost_matrix:" << temp_cost.shape(0) <<","<< temp_cost.shape(1) <<std::endl;

    std::vector<int> match_0, match_1;
    for (int i = 0; i < matched_indices.shape(0); ++i) 
    {
        match_0.push_back(matched_indices(i, 0));
        match_1.push_back(matched_indices(i, 1));
    }

    std::vector<int> unmatched_detections;
    for (int j = 0; j < detections.shape(0); ++j) 
    {
        if (std::find(match_0.begin(), match_0.end(), j) == match_0.end())
            unmatched_detections.push_back(j);
    }
    std::vector<int> unmatched_trackers;
    for (int k = 0; k < trackers.shape(0); ++k) 
    {
        if (std::find(match_1.begin(), match_1.end(), k) == match_1.end())
            unmatched_trackers.push_back(k);
    }

    std::vector<int> matches_0, matches_1;
    for (int m = 0; m < matched_indices.shape(0); ++m) 
    {
        int a = matched_indices(m, 0), b = matched_indices(m, 1);
        //iou阈值和低速度高阈值
        if (flag == 1)
        {
            if ((iou_matrix(a, b) < _iou_threshold) || (iou_matrix_new(a, b) < 0.2 && trackers(b, trackers.shape(1) - 1) < 3)) 
            {
                unmatched_detections.push_back(a);
                unmatched_trackers.push_back(b);
            } else 
            {
                matches_0.push_back(matched_indices(m, 0));
                matches_1.push_back(matched_indices(m, 1));
            }
        }else
        {
            if (flag == 2)//机动车
            {
                //机动车距离匹配阈值
                if (fabsf(L_distance(a,b)) >= 7 || fabsf(N_distance(a,b)) >= 2 || fabsf(O_distance(a,b))>=10 )
                {
                    unmatched_detections.push_back(a);
                    unmatched_trackers.push_back(b);
                } else 
                {
                    matches_0.push_back(matched_indices(m, 0));
                    matches_1.push_back(matched_indices(m, 1));
                }
            }
            else//非机动车
            {
                //非机动车距离匹配阈值
                if (fabsf(O_distance(a,b)) >= 6)
                {
                    unmatched_detections.push_back(a);
                    unmatched_trackers.push_back(b);
                }else
                {
                    matches_0.push_back(matched_indices(m, 0));
                    matches_1.push_back(matched_indices(m, 1));
                }
            }
        }
    }
    if (!matches_0.size())
        associate_out.matches = xt::empty<int>({0, 2});
    else 
    {
        assert(matches_0.size() == matches_1.size());
        std::vector <std::size_t> shape = {matches_0.size(), 1};
        xt::xarray<int> xt_match0 = xt::adapt(matches_0, shape);
        xt::xarray<int> xt_match1 = xt::adapt(matches_1, shape);
        associate_out.matches = xt::concatenate(xt::xtuple(xt_match0, xt_match1), 1);
    }
    associate_out.unmatched_detections_vector = unmatched_detections;
    associate_out.unmatched_trackers_vector = unmatched_trackers;
    std::vector <std::size_t> shape_d = {unmatched_detections.size()};
    std::vector <std::size_t> shape_t = {unmatched_trackers.size()};
    associate_out.unmatched_detections = xt::adapt(unmatched_detections, shape_d);
    associate_out.unmatched_trackers = xt::adapt(unmatched_trackers, shape_t);
    associate_out.cost_matrix = iou_matrix;

    return associate_out;
}

//车道线加载，车道划分与轨迹航向角
void FSort_crossing::trackAngleHandle(xt::xarray<float> x_temp,xt::xarray<float> d,int i ,bool useLanebmp)
{
    auto angel_fmod = std::fmod(x_temp(4), Onecircle);
    angel_fmod = angel_fmod >= 0 ? angel_fmod : Onecircle + angel_fmod;
    _trackers[i]._angle_list.push_back(angel_fmod);

    int l_tracker = _trackers[i]._angle_list.size();
    if (l_tracker > 10) 
    {
        for (int j = 0; j < l_tracker - 10; ++j) 
        {
            _trackers[i]._angle_list.erase(_trackers[i]._angle_list.begin());
        }
    }

    if (_trackers[i]._angle_list.size() > 1) 
    {
        auto angle_diff = std::abs(_trackers[i]._angle_list[_trackers[i]._angle_list.size() - 1] - _trackers[i]._angle_list[_trackers[i]._angle_list.size() - 2]);
        if (angle_diff < _angle_judge || angle_diff > (Onecircle - _angle_judge)) 
        {

        } else if (angle_diff > (Halfcicle - _angle_judge) && angle_diff < (Halfcicle + _angle_judge)) 
        {
            auto angel_inf = std::fmod((_trackers[i]._angle_list[_trackers[i]._angle_list.size() - 1] + Halfcicle), Onecircle);
            angel_inf = angel_inf >= 0 ? angel_inf : Onecircle + angel_inf;
            _trackers[i]._angle_list[_trackers[i]._angle_list.size() - 1] = angel_inf;
        } else
        {
            _trackers[i]._angle_list[_trackers[i]._angle_list.size() - 1] = _trackers[i]._angle_list[_trackers[i]._angle_list.size() - 2];
        }
    }
    _trackers[i]._detec_angle = _trackers[i]._angle_list[_trackers[i]._angle_list.size() - 1];

    if(useLanebmp)
    {
        auto d0 = d(0);
        auto d1 = d(1);
        //车道bmp
        int bmpWidth = _laneParam(2,0);
        int bmpLenth = _laneParam(2,1);
        int tempx = (d0 + _laneParam(2,2) / _laneParam(2,4)) * _laneParam(2,4);
        int tempy = -((d1 - _laneParam(2,3) / _laneParam(2,4)) * _laneParam(2,4)) - 1;
        if (tempx >= bmpWidth)
            tempx = bmpWidth-1;
        if (tempy >= bmpLenth)
            tempy = bmpLenth-1;
        if (tempx <= -1)
            tempx = 0;
        if (tempy <= -1)
            tempy = 0;

        int lane_color = _lane_bmp.at<cv::Vec3b>(tempy, tempx)[0];
        _trackers[i]._ilaneIndexGlobal = lane_color/_laneParam(2,5);
        _trackers[i]._lane_angle = _laneParam(1,_trackers[i]._ilaneIndexGlobal);
    }else
    {
        _trackers[i]._lane_angle = None_zsc;
    }

    if (_trackers[i]._state.size() > 1) 
    {
        _trackers[i]._track_angle = box_ops_crossing::cal_angle(_trackers[i]._state, _dis_thresh);
    }
    float head_angle;
    if (_trackers[i]._high_speed) 
    {
        if (std::abs(_trackers[i]._track_angle - None_zsc) > 0.2) 
        {
            head_angle = _trackers[i]._track_angle;
            _trackers[i]._final_angle = head_angle;
        } else if (std::abs(_trackers[i]._lane_angle - None_zsc) > 0.2) 
        {
            head_angle = _trackers[i]._lane_angle;
            _trackers[i]._final_angle = head_angle;
        } else 
        {
            head_angle = _trackers[i]._detec_angle;
        }
    } else 
    {
        if (std::abs(_trackers[i]._lane_angle - None_zsc) > 0.2) 
        {
            head_angle = _trackers[i]._lane_angle;
            _trackers[i]._final_angle = head_angle;
        } else if (std::abs(_trackers[i]._track_angle - None_zsc) > 0.2) 
        {
            head_angle = _trackers[i]._track_angle;
            _trackers[i]._final_angle = head_angle;
        } else 
        {
            head_angle = _trackers[i]._detec_angle;
        }
    }

    if (std::abs(_trackers[i]._final_angle - None_zsc) > 0.2) 
    {
        float angle_diff = std::fmod(std::abs(_trackers[i]._final_angle - head_angle), Onecircle);
        angle_diff = angle_diff >= 0 ? angle_diff : Onecircle + angle_diff;
        if (angle_diff < _angle_judge || angle_diff > (Onecircle - _angle_judge)) 
        {

        } else if (angle_diff > (Halfcicle - _angle_judge) && angle_diff < (Halfcicle + _angle_judge)) 
        {
            head_angle = std::fmod(head_angle + Halfcicle, Onecircle);
            head_angle = head_angle >= 0 ? head_angle : Onecircle + head_angle;
        } else 
        {
            head_angle = _trackers[i]._final_angle;
        }
    }
    _trackers[i]._head_angle.push_back(head_angle);

}

//结果nms
void FSort_crossing::result_nms(xt::xarray<float> &pc_ret_nms, std::vector<int> to_del3)
{
    if (pc_ret_nms.shape(0) > 0)
    {
        auto iou_res2 = box_ops_crossing::rotate_nms_cc(pc_ret_nms, pc_ret_nms);
        xt::xarray<float> iou_matrix2 = iou_res2.first;
        for (int i = 0; i < iou_matrix2.shape(0); ++i)
        {
            for (int j = 0; j < iou_matrix2.shape(0); ++j)
            {
                if (i>j)
                {
                    if (iou_matrix2(i,j) >= _nms_min && iou_matrix2(i,j) <= _nms_max)
                    {
                        if (pc_ret_nms(i,10) > pc_ret_nms(j,10))
                            to_del3.push_back(j);
                        else
                            to_del3.push_back(i);
                    }
                }

            }
        }

        for (int i = 0; i < pc_ret_nms.shape(0); ++i)
        {
            int flag_new = 0;
            for (int j = 0; j < to_del3.size(); ++j)
            {
                if (i == to_del3[j])
                    flag_new =1;
            }
            if (flag_new == 0)
            {
                xt::xarray<float> temp1 = xt::view(pc_ret_nms, i, xt::all());
                _pc_ret = xt::concatenate(xt::xtuple(_pc_ret, temp1.reshape({1,temp1.size()})), 0);
            }
        }
    }
    to_del3.clear();
}

//距离匹配计算，机动车与非机动车采用不用的模型,flag:2 机动车 flag:1 非机动车
void FSort_crossing::cal_distance(xt::xarray<float> &detections,
                                    xt::xarray<float> &trackers,
                                    int flag,
                                    xt::xarray<float> &L_distance,
                                    xt::xarray<float> &N_distance,
                                    xt::xarray<float> &O_distance)
{
    for (int i = 0; i < detections.shape(0); ++i)
    {
        for (int j = 0; j < trackers.shape(0); ++j)
        {
            float theta = trackers(j,4);
            float O_dis = sqrt(pow(detections(i,0)-trackers(j,0),2)+pow(detections(i,1)-trackers(j,1),2));
            float L_dis = O_dis * sin((- theta - 90) / 180 * PI);
            float N_dis = O_dis * cos((- theta - 90) / 180 * PI);

            //椭圆门限
            if (flag == 2)//car_dis
                O_dis = sqrt(pow(L_dis,2) * _caldis_A + pow(N_dis,2) * _caldis_B) ;
            if (O_dis >= 12)
                O_dis = 1000;
            L_distance(i, j) = L_dis;
            N_distance(i ,j) = N_dis;
            O_distance(i ,j) = -fabsf(O_dis);
        }
    }
}





void FSort_crossing::sort(xt::xarray<float> &data, std::vector<xt::xarray<float>> &video_ret, int frame_count)
{
    _frame_count = frame_count;
    xt::xarray<float> dets = xt::empty<float>({int(data.shape(0)), 15});
    _pc_ret = xt::empty<float>({0, 21});
    for (int i = 0; i < data.shape(0); ++i) 
    {
        dets(i, 0) = data(i, 0); // x
        dets(i, 1) = data(i, 1); // y
        dets(i, 2) = data(i, 3); // w
        dets(i, 3) = data(i, 4); // l
        dets(i, 4) = data(i, 6); // angle
        dets(i, 5) = data(i, 2); // z
        dets(i, 6) = data(i, 5); // h
        //size restict class
        float lidar_class;
        xt::xarray<float> data_temp = xt::view(data, i, xt::all());
        data_temp = data_temp.reshape({data_temp.size()});
        label_crossing::lidar_class_judge(data_temp,lidar_class); // TODO
        dets(i, 7)  = lidar_class; // label
        dets(i, 8)  = data(i, 10); // score
        dets(i, 9)  = data(i, 35); // leftx
        dets(i, 10) = data(i, 36); // lefty
        dets(i, 11) = data(i, 37); // rightx
        dets(i, 12) = data(i, 38); // righty
        dets(i, 13) = data(i, 39); // camID
        dets(i, 14) = data(i, 40); // video_boxid
    }

    int l_num = _trackers.size();
    xt::xarray<float> trks = xt::zeros<float>({l_num, 8});
    xt::xarray<float> ret = xt::empty<float>({0, 21});
    std::vector<int> to_del,to_del2,to_del3;
    //last frame predict
    for (int i = 0; i < l_num; ++i) 
    {
        auto trk_pre = _trackers[i].predict();
        xt::xarray<float> pos = trk_pre.first;
        xt::xarray<float> bbox = trk_pre.second;
        pos = xt::view(pos, 0);
        xt::view(trks, i) = xt::xarray<float>({pos(0), pos(1), bbox(2), bbox(3), bbox(4),
                                               float(_trackers[i]._time_since_update),
                                               float(_trackers[i]._speed),
                                               float(_trackers[i]._label)});
        if (xt::any(xt::isnan(pos)))
            to_del.push_back(i);
    }

    //跟踪目标去除空数据
    std::vector<int> invalid;
    for (int j = 0; j < trks.shape(0); ++j) 
    {
        if (xt::any(xt::isinf(xt::view(trks, j))) ||
        xt::any(xt::isnan(xt::view(trks, j))))
            invalid.push_back(j);
    }//isnan
    trks = xt::view(trks, xt::drop(invalid));

    std::reverse(to_del.begin(), to_del.end());
    for (int i = 0; i < to_del.size(); ++i) 
    {
        _trackers.erase(_trackers.begin() + to_del[i]);
    }

    //first match detection with trackers:IOU
    auto match_res = FSort_crossing::associate_detections_to_trackers(dets, trks, 1);
    auto matched = match_res.matches;
    auto unmatched_dets = match_res.unmatched_detections_vector;
    auto unmatched_trks = match_res.unmatched_trackers_vector;
    auto cost_matrix = match_res.cost_matrix;
    //matched trk update
    for (int i = 0; i < _trackers.size(); ++i) 
    {
        if (std::find(unmatched_trks.begin(), unmatched_trks.end(), i) == unmatched_trks.end()) 
        {
            auto index = xt::where(xt::col(matched, 1) > i - 1 &&
                    xt::col(matched, 1) < i + 1)[0];
            _trackers[i].update(xt::view(dets, matched(index[0], 0)));
        }
    }


    //***********************    dis_km     *******************************

    //第二轮匹配:距离匹配
    std::vector<FUKalmanBoxTracker_crossing> unmatchTrkAfterkm = distanceMatch(dets, trks, unmatched_dets, unmatched_trks);

    //跟踪结果输出
    int num_tra = _trackers.size();
    for (int i = _trackers.size() - 1; i >= 0; --i) 
    {
        fustate_output_crossing res = _trackers[i].get_state();
        xt::xarray<float> d = res.x;
        xt::xarray<float> x_temp = res.bbox;
        float trk_speed = res.speed;
        std::vector <xt::xarray<float>> angle_box = res.angle_box;
        float lefttopx_ = res.lefttopx;
        float lefttopy_ = res.lefttopy;
        float rightbottomx_ = res.rightbottomx;
        float rightbottomy_ = res.rightbottomy;
        float camid_ = res.camid;
        float VideoBoxid_ = res.VideoBoxid;

        d = d.reshape({d.size()});

        //处理航向角和车道线
        trackAngleHandle(x_temp, d, i, _useLanebmp);

        //change video_id
        if (camid_ != 0 && VideoBoxid_ != -1)
        {
            video_ret[int(camid_-1)]((int)VideoBoxid_, 7) = int(_trackers[i]._id + 1);
        }
        //跟踪结果输出
        if ((_trackers[i]._time_since_update < _max_age &&(_trackers[i]._hits >= _min_hits || _frame_count <= _min_hits)) ||
        ((_trackers[i]._time_since_update < _max_age_new && _trackers[i]._hits >= 10))) 
        {
            float head_final = _trackers[i]._head_angle[_trackers[i]._head_angle.size() - 1];
            head_final = head_final >= 0 ? head_final : Onecircle + head_final;
            if (_trackers[i]._time_since_update < 6 )
            {
                if (x_temp(3)>9)
                {
                    _trackers[i]._label = 4;
                    std::vector<int> temp_list =_trackers[i].Get_label();
                    _trackers[i].setLabel(temp_list.size()-1, 5);
                }
                xt::xarray<float> d_conv = {d(0),
                                            d(1),
                                            x_temp(2),
                                            x_temp(3),
                                            head_final,
                                            x_temp(5),
                                            x_temp(6),
                                            static_cast<float>(_trackers[i]._label),
                                            static_cast<float>(_trackers[i]._speed),
                                            static_cast<float>(_trackers[i]._id + 1),
                                            x_temp(8),
                                            static_cast<float>(_trackers[i]._hits),
                                            static_cast<float>(_trackers[i]._ilaneIndexGlobal),
                                            static_cast<float>(_trackers[i]._time_since_update),
                                            1.0,
                                            static_cast<float>(_frame_count),
                                            lefttopx_,
                                            lefttopy_,
                                            rightbottomx_,
                                            rightbottomy_,
                                            camid_};


                d_conv.reshape({1, d_conv.size()});
                ret = xt::concatenate(xt::xtuple(ret, d_conv), 0);
            }
        }

        num_tra -= 1;
        if (_trackers[i]._time_since_update > _max_age && _trackers[i]._hits < 10) 
        {
            to_del2.push_back(num_tra);

        } else if (_trackers[i]._time_since_update > _max_age_new) 
        {
            to_del2.push_back(num_tra);
        }
    }

    std::sort(to_del2.rbegin(), to_del2.rend());
    for (auto num : to_del2) {
        _trackers.erase(_trackers.begin() + num);
    }

    //图像孪生结果与点云跟踪结果去重（图像孪生没做跟踪，也可以将该函数提前将图像结果加入跟踪）
//    xt::xarray<float> video_coordinate_temp = littleTargetMap(ret);
//    pc_ret_nms = xt::concatenate(xt::xtuple(ret, video_coordinate_temp), 0);

    //NMS FOR RESULT
    result_nms(ret,to_del3);
    to_del2.clear();

}