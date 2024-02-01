//
// Created by root on 10/24/22.
//
#include "FUKalmanBoxTracker_crossing.h"
#include "label_crossing.h"
#define None_zsc -10000.1

//卡尔曼滤波器
FUKalmanBoxTracker_crossing::FUKalmanBoxTracker_crossing(xt::xarray<float> bbox) : _kf{} 
{
    _use_acc_model = 0;
    _track_angle = None_zsc;
    _final_angle = None_zsc;

    //对机动车采用加速CA模型，对于非机动车采用匀速CV模型
    if (label_crossing::judge_large_target(int(bbox(7)))) {
        _use_acc_model = 1;
        _kf = FUKalmanFilter_crossing(8, 4);
        _kf._F = xt::xarray<float>({{1, 0, 0, 0, 0.1, 0,   0.005, 0},
                                    {0, 1, 0, 0, 0,   0.1, 0,     0.005},
                                    {0, 0, 1, 0, 0,   0,   0,     0},
                                    {0, 0, 0, 1, 0,   0,   0,     0},
                                    {0, 0, 0, 0, 1,   0,   0.1,   0},
                                    {0, 0, 0, 0, 0,   1,   0,     0.1},
                                    {0, 0, 0, 0, 0,   0,   1,     0},
                                    {0, 0, 0, 0, 0,   0,   0,     1}});

        _kf._H = xt::xarray<float>({{1, 0, 0, 0, 0, 0, 0, 0},
                                    {0, 1, 0, 0, 0, 0, 0, 0},
                                    {0, 0, 1, 0, 0, 0, 0, 0},
                                    {0, 0, 0, 1, 0, 0, 0, 0}});
        _kf._Q *= 0.1; // init value
        _kf._R *= 1;
        _kf._P *= 10;
        xt::view(_kf._P, xt::range(4, 6), xt::range(4, 6)) *= 1000;
    } else 
    {
        _use_acc_model = 0;
        _kf = FUKalmanFilter_crossing(7, 4);
        _kf._F = xt::xarray<float>({{1, 0, 0, 0, 1, 0, 0},
                                    {0, 1, 0, 0, 0, 1, 0},
                                    {0, 0, 1, 0, 0, 0, 1},
                                    {0, 0, 0, 1, 0, 0, 0},
                                    {0, 0, 0, 0, 1, 0, 0},
                                    {0, 0, 0, 0, 0, 1, 0},
                                    {0, 0, 0, 0, 0, 0, 1}});

        _kf._H = xt::xarray<float>({{1, 0, 0, 0, 0, 0, 0},
                                    {0, 1, 0, 0, 0, 0, 0},
                                    {0, 0, 1, 0, 0, 0, 0},
                                    {0, 0, 0, 1, 0, 0, 0}});
        _kf._Q(_kf._Q.shape(0) - 1, _kf._Q.shape(1) - 1) *= 0.01; // init value
        xt::view(_kf._Q, xt::range(4, _kf._Q.shape(0)), xt::range(4, _kf._Q.shape(1))) *= 0.01;
        _kf._R *= 10;
        _kf._P *= 10;
        xt::view(_kf._P, xt::range(4, 6), xt::range(4, 6)) *= 1000;
    }
    xt::xarray<float> bbox_ = xt::view(bbox, xt::range(0, 4));

    //使用box_lidar,里面包含中心点，面积和航向角度
    xt::view(_kf._x, xt::range(0, 4)) = bbox_.reshape({bbox_.size(), 1});
    _bbox = bbox;
    _time_since_update = 0;

    //减小和增大卡尔曼Q矩阵的标志位
    _min_q = true;
    _max_q = false;

    //存储检测目标的稳定航向角，以车道角和轨迹角为标准
    _final_angle = None_zsc;
    //表示检测目标的ID
    _id = _count;  //
    _count += 1;

    //    _history = xt::zeros<float>({1,4});
    //存储相应类别的对应的目标的anchor信息
    //0:car 1:bicycle 2:bus 3:tricycle 4:pedestrian 5:semi 6:truck
    //5 model size has a problem, pay attention
    _label_dict = {{0, xt::xarray<float>({1.95017717, 4.60718145, 1.72270761})},
                   {1, xt::xarray<float>({0.60058911, 1.68452161, 1.27192197})},
                   {2, xt::xarray<float>({0.60058911, 1.68452161, 1.27192197})},
                   {3, xt::xarray<float>({0.76279481, 2.09973778, 1.44403034})},
                   {4, xt::xarray<float>({0.66344886, 0.7256437, 1.75748069})},
                   {5, xt::xarray<float>({2.94046906, 11.1885991, 3.47030982})},
                   {6, xt::xarray<float>({2.4560939, 6.73778078, 2.73004906})}};
    //高速目标  低速目标
    _high_speed = false;
    _low_speed = false;
    //动态目标  静态目标
    _dynamic = false;
    _static = false;

    //更新的次数
    _hits = 0;
    //追踪目标的速度
    _speed = 0;
    //高速和低速的阈值,10km/h
    _speed_thresh = 3; //动静态阈值
}

FUKalmanBoxTracker_crossing::FUKalmanBoxTracker_crossing() {}

//拷贝构造函数
FUKalmanBoxTracker_crossing::FUKalmanBoxTracker_crossing(const FUKalmanBoxTracker_crossing &rhs) 
{
    _use_acc_model = rhs._use_acc_model;
    _time_since_update = rhs._time_since_update;

    _speed        = rhs._speed;
    _label        = rhs._label;
    _speed_thresh = rhs._speed_thresh;
    _angle_box    = rhs._angle_box;
    _angle_list   = rhs._angle_list;
    _detec_angle  = rhs._detec_angle;
    _lane_angle   = rhs._lane_angle;
    _state        = rhs._state;
    _track_angle  = rhs._track_angle;
    _high_speed   = rhs._high_speed;
    _low_speed    = rhs._low_speed;
    _final_angle  = rhs._final_angle;
    _head_angle   = rhs._head_angle;
    _hits         = rhs._hits;
    _id           = rhs._id;
    _kf           = rhs._kf;
    //    _ilaneIndexGlobal = rhs._ilaneIndexGlobal;
    _use_acc_model = rhs._use_acc_model;
    _bbox          = rhs._bbox;
    _min_q         = rhs._min_q;
    _max_q         = rhs._max_q;
    _lefttopx      = rhs._lefttopx;
    _lefttopy      = rhs._lefttopy;
    _rightbottomx  = rhs._rightbottomx;
    _rightbottomy  = rhs._rightbottomy;
    _camid         = rhs._camid;
    _VideoBoxid    = rhs._VideoBoxid;
    _history       = rhs._history;
    _label_dict    = rhs._label_dict;
    _dynamic       = rhs._dynamic;
    _static        = rhs._static;
    _state_judge   = rhs._state_judge;
    _label_box     = rhs._label_box;
    _label_box_central_area = rhs._label_box_central_area;
}

FUKalmanBoxTracker_crossing::~FUKalmanBoxTracker_crossing() {}

//检测更新跟踪器，同时跟踪的后处理在这里完成
void FUKalmanBoxTracker_crossing::update(xt::xarray<float> bbox) 
{
    //表示距离上一次更新后没有再匹配更新的次数
    _time_since_update = 0;
    std::vector <xt::xarray<float>>::iterator it;
    std::vector <xt::xarray<float>> ().swap(_history);

    //匹配上的次数
    _hits += 1;
    xt::xarray<float> bbox_ = xt::view(bbox, xt::range(0, 4));
    bbox_ = bbox_.reshape({int(bbox_.size() / 4), 4});
    //使用匹配到的box信息对当前的卡尔曼状态量进行更新
    _kf.update(bbox_);
    //更新存储的目标信息
    _bbox = bbox;
    _lefttopx = bbox(9);
    _lefttopy = bbox(10);
    _rightbottomx = bbox(11);
    _rightbottomy = bbox(12);
    _camid = bbox(13);
    _VideoBoxid = bbox(14);
    //最新50帧目标分类结果
    float d0 = _bbox(0);
    float d1 = _bbox(1);
    //部分核心区域类别统计,用来统计非机动车细分类问题
    //    if(d0 < 0 && d0 > -20 && d1 < 5 && d1 > -10)
    //    {
    //        _label_box_central_area.push_back(int(_bbox(7)));
    //    }
    //全局类别统计
    _label_box.push_back(int(_bbox(7)));

    //保留历史50帧
    int label_storage_time = 50;
    if (_label_box.size() > label_storage_time) {
        for (int i = 0; i < _label_box.size() - label_storage_time; ++i) {
            _label_box.erase(_label_box.begin()); // delete first N
        }
    }
//    if (_label_box_central_area.size() > label_storage_time) {
//        for (int i = 0; i < _label_box_central_area.size() - label_storage_time; ++i) {
//            _label_box_central_area.erase(_label_box_central_area.begin()); // delete first N
//        }
//    }

    //取分类类别最大,(是不是考虑放弃远端的类别可信度)
    if (_label_box.size() > 0) {
        int more_label = 0;
        int temp_max_count = 0;
        xt::xarray<int> label_count_list = xt::zeros<int> ({12});//class number
        for (int i = 0; i < 12; ++i)
        {
            label_count_list(i) =std::count(_label_box.begin(), _label_box.end(), i);
            if (label_count_list(i) > temp_max_count)
            {
                temp_max_count = label_count_list(i);
                more_label = i;
            }
        }
        //众数类别
        _label = more_label;
        // xt::xarray<int> label_count_list2 = xt::zeros<int> ({25});//class number
//        if (_label_box_central_area.size() > 0)
//        {
//
//            for (int j = 0; j < 10; ++j)
//            {
//                label_count_list2(j) =std::count(_label_box_central_area.begin(), _label_box_central_area.end(), j);
//            }
//        }

        //非机动车细分类类别判断，可以根据实际需求进行调整
        // if ((float(label_count_list(8)+label_count_list(9)+label_count_list(10)+label_count_list(11)+label_count_list(12)+label_count_list(13))/float(_label_box.size()))>0.7) {
        //     //            if (_label_box_central_area.size() > 0)
        //     if (false)
        //     {
        //         //在指定区域内出现几次即认为是该类型
        //         if (float(label_count_list2(8)) > 3)
        //         {
        //             _bbox(7) = 8;
        //             _label_box[_label_box.size() - 1] = 8;
        //         }
        //         else if (float(label_count_list2(7)) > 3)
        //         {
        //             _bbox(7) = 7;
        //             _label_box[_label_box.size() - 1] = 7;
        //         }
        //         else if (float(label_count_list2(9)) > 3)
        //         {
        //             _bbox(7) = 9;
        //             _label_box[_label_box.size() - 1] = 9;
        //         }
        //         else if (float(label_count_list2(2)) > 4)
        //         {
        //             _bbox(7) = 2;
        //             _label_box[_label_box.size() - 1] = 2;
        //         }
        //         else {
        //             _bbox(7) = 1;
        //             _label_box[_label_box.size() - 1] = 1;
        //         }
        //         _label = int(_bbox(7));
        //     }
        //     else
        //     {
        //         _bbox(7) = 8;
        //         _label_box[_label_box.size() - 1] = 8;
        //         _label = int(_bbox(7));
        //     }
        // }else if ((float(temp_max_count) / float(_label_box.size())) > 0.5)
        // {
        //     _bbox(7) = _label;
        //     //把历史类别的最新类别也改成多数类别
        //     _label_box[_label_box.size() - 1] = more_label;
        // }
    }else
    {
        _label = _bbox(7);
    }

    //速度计算，是不是可以考虑不计算横向位移
    if (_use_acc_model)
    {
        _speed = std::sqrt(std::pow(_kf._x(4, 0), 2) + std::pow(_kf._x(5, 0), 2));
    }else
    {
        _speed = 10 * std::sqrt(std::pow(_kf._x(4, 0), 2) + std::pow(_kf._x(5, 0), 2));
    }
    if (_speed > _speed_thresh)
    {
        _high_speed = true;
        _low_speed = false;
        if (_label == 0) //not person
        {
            _label = 5;
            if (_label_box.size() > 0)
            {
                for (int i = 0; i < _label_box.size(); ++i)
                {
                    _label_box[i] = 5;
                }
            }
        }
    } else 
    {
        _high_speed = false;
        _low_speed = true;
    }
    //个人认为应该只考虑沿前进方向的位移
    if (_state_judge.size() > 9) 
    {
        float diff_x = _state_judge[_state_judge.size() - 1](0) - _state_judge[0](0);
        float diff_y = _state_judge[_state_judge.size() - 1](1) - _state_judge[0](1);
        float diff_dis = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        _static = _speed < 3 && diff_dis < 1.6;
        _dynamic = !(_speed < 3 && diff_dis < 1.6);
    }
    //低速的时候，将卡尔曼的Q矩阵变小，让轨迹更平滑，即受离谱检测目标的影响会小些，待测试的功能
    if (label_crossing::judge_large_target(_label)) 
    {
        if (!_high_speed and _min_q) 
        {
            _kf._Q *= 0.1;
            _min_q = false;
            _max_q = true;
        }
        if (_high_speed and _max_q) 
        {
            _kf._Q *= 10;
            _min_q = true;
            _max_q = false;
        }
    }
    //历史状态存储x,y
    if (_speed > 0.8) 
    {
        xt::xarray<float> x_ = xt::view(_kf._x, xt::range(0, 2));
        _state.push_back(x_.reshape({1, x_.size()}));
    }
    int l_state = _state.size();
    if (_high_speed) 
    {
        if (l_state > 10) {
            for (int i = 0; i < l_state - 10; ++i) 
            {
                _state.erase(_state.begin() + 0); // delete first N
            }
        }
    } else 
    {
        if (l_state > 30) 
        {
            for (int i = 0; i < l_state - 30; ++i) 
            {
                _state.erase(_state.begin() + 0); // delete first N
            }
        }
    }
    //_angle_box存储检测状态量10帧
    _angle_box.push_back(_bbox);
    int l_box = _angle_box.size();
    if (l_box > 10) 
    {
        for (int i = 0; i < l_box - 10; ++i)
        {
            _angle_box.erase(_angle_box.begin()); // delete first N
        }
    }
}

//后处理，将一些现场条件修正的结果放入历史结果中
void FUKalmanBoxTracker_crossing::setLabel(int index, int label)
{
    if(index > _label_box.size()) 
    {
        return;
    }
    _label_box[index]=label;
}

//跟踪预测，速度大于2或小于连续3帧没有关联的目标进行预测
std::pair <xt::xarray<float>, xt::xarray<float>> FUKalmanBoxTracker_crossing::predict() 
{
    //对于速度较小，且连续>=3帧没有更新的轨迹不进行预测
    if (_speed > 2 or _time_since_update < 3) 
    {
        _kf.predict();
    }
    _time_since_update += 1;
    //存储历史位置长宽
    xt::xarray<float> output_history = xt::view(_kf._x, xt::range(0, 4));
    output_history.reshape({1, 4});
    _history.push_back(output_history);
    //历史10帧状态
    _state_judge.push_back(_kf._x);
    int l_state_judge = _state_judge.size();
    if (l_state_judge > 10) 
    {
        for (int i = 0; i < l_state_judge - 10; ++i)
            _state_judge.erase(_state_judge.begin()); // delete the first N
    }
    std::pair <xt::xarray<float>, xt::xarray<float>> p1{_history[_history.size() - 1], _bbox};
    return p1;
}

//获取跟踪目标状态量
fustate_output_crossing FUKalmanBoxTracker_crossing::get_state() {
    xt::xarray<float> output_x = xt::view(_kf._x, xt::range(0, 4));
    output_x = output_x.reshape({1, 4});
    if (_speed < 0.5 and _angle_box.size() > 1) 
    {
        xt::xarray<float> x_mean = xt::zeros<float>({1, int(_angle_box[0].size())});
        for (int i = 0; i < _angle_box.size(); ++i) 
        {
            x_mean = xt::concatenate(xt::xtuple(x_mean, _angle_box[i].reshape({1, _angle_box[i].size()})), 0);
        }
        //历史10帧位置取平均
        output_x = xt::mean(xt::view(x_mean, xt::range(1, _angle_box.size() + 1), xt::range(0, 4)), 0);
        output_x.reshape({1, 4});
    }

    //非预测目标，可以查到对应图像检测框
    if (_time_since_update!=0)
    {
        _lefttopx = 0;
        _lefttopy = 0;
        _rightbottomx = 0;
        _rightbottomy = 0;
        _camid = -1;
        _VideoBoxid = -1;
    }
    fustate_output_crossing res;
    res.x = output_x;
    res.bbox = _bbox;
    res.speed = _speed;
    res.angle_box = _angle_box;
    res.lefttopx = _lefttopx;
    res.lefttopy = _lefttopy;
    res.rightbottomx = _rightbottomx;
    res.rightbottomy = _rightbottomy;
    res.camid = _camid;
    res.VideoBoxid = _VideoBoxid;
    return res;
}

