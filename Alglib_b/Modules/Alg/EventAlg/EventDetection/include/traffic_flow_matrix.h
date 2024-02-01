//
// Created by root on 7/1/22.
//

#ifndef EVENT_ALG_TRAFFIC_FLOW_MATRIX_H
#define EVENT_ALG_TRAFFIC_FLOW_MATRIX_H

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <unistd.h>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <exception>
#include <malloc.h>
#include <json.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "xtensor/xnpy.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xshape.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor-blas/xlinalg.hpp"
#include "xtensor/xview.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xsort.hpp"
#include "xtensor/xio.hpp"

//#include "opencv2/core.hpp"
//#include "opencv2/calib3d.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc.hpp"

//#include "spdlog/spdlog.h"

#include "yaml-cpp/yaml.h"
#include "KDTree.h"

using json = nlohmann::json;

/* xcb note
#define __A 6378137
#define __B 6356725
#define __IterativeValue 10
*/
#define R_a 6378137.00
#define R_b 6356752.3142


using namespace std;

// 多层map模板类
template<typename T1, typename T2=int>
class map_event {
public:
    typedef std::map<int, std::map<int, T1>> triple_map;
    typedef std::map<int, std::map<int, std::map<int, std::vector<T1>>>> quadra_map;
    typedef std::map<int, T1> Param_map;
    typedef std::map<int, std::map<std::string, T1>> triple_map_str;
    typedef std::map<int, std::map<std::string, std::map<int, T1>>> triple_map_multi_type;
    typedef std::map<T1, T2> noarm_map;
};


enum Lane_Class {
    highway = 0,  //# 快速路
    main_branch = 1  // 主干路、支路
};
enum Config_Matrix_Index {
    LANE_NO = 0,
    TURN_LEFT_1 = 1,  // 左转辅助道1
    TURN_LEFT_2 = 2,  // 左转辅助道2
    TURN_LEFT_3 = 3,  // 左转辅助道3
    EMERGENCY_AREA = 4,  // emergency 4
    CALC_FLOW = 5,  // 车流量统计区
    FORBID_CROSS_LINE = 6,  // 禁止压线区
    NO_PARK = 7,  // 不能停车
    CENTER_AREA = 8,  // 十字中心,包括车道头
    SIDEWALK = 9,  // 人行道
    SECTION_ID = 10,  //section id
    PEDSTRIAN_WAITING_AREA = 11 //行人等待区id
};
enum Congestion_Level {
    unblocked = 1,              // 畅通
    slightly_congested = 2,     // 轻度拥堵
    moderately_congested = 3,   // 中度拥堵
    seriously_congested = 4     // 严重拥堵
};
// 异常事件编码
enum Abnormal_Class_Index {
    illegal_stop = 1, // 违停
    retrograde_for_non_motor = 2, // 非机动车逆行
    retrograde_for_motor = 3, // 机动车逆行
    occupy_dedicated_lane = 28, // 占用专用车道
    people_occupy_motor_lane = 5, // 行人占用机动车道

    speeding_for_motor = 6, // 机动车超速
    speeding_for_non_motor = 7, // 非机动车超速
    stroll = 8, // 慢行
    cross_line = 9, // 机动车压线
    cross_lane_for_non_motor = 10, // 非机动车横穿
    cross_lane_for_people = 11, // 行人横穿
    cross_lane_for_motor = 12, // 机动车横穿
    run_the_red_light_for_motor = 13, // 机动车闯红灯
    run_the_red_light_for_people = 14, // 行人闯红灯
    occupy_bus_lane = 15, // 占用公交车道
    change_lanes = 16, // 变道
    spills = 17, // 抛洒物
    accident = 18, // 事故
    occupy_emergency_lane = 19, // 占用应急车道
    hanging_in_tunnel = 20, // 隧道悬挂物
    person_in_tunnel = 21, // 行人闯入隧道
    non_motor_in_tunnel = 22, // 非机动车闯入隧道
    stop_over_terminate_line = 23, //非机动车越线停车
    non_motor_occupy_motor_lane = 24, //zly add 非机动车占用机动车道
    run_the_red_light_for_non_motor = 25,   //非机动车闯红灯，TODO：未拆分
    
    reversing_for_motor = 99,           //机动车倒车

    not_following_lane_guide = 27,   //未按导向车道行驶  e
    motor_occupy_non_motor_lane = 28,    //机动车占用非机动车道，TODO：目前为机动车占用专用车道
    stop_over_terminate_line_for_motor = 29, //机动车越线停车
    roadwork = 30, // 道路施工 
    car_in_zebra_stripe = 31,            //不理让行人 车在斑马线
    person_not_walk_zebra_stripe = 32,   //行人未走斑马线（在路口核心区）

    congestion = 33, // 拥堵
    traverse_the_diversion_zone = 34, // 穿越导流区
    big_car_in_dedicated_lane = 35, // 大车占用小车道
};

class History_Different_Lane_Info{
public:
    History_Different_Lane_Info(int lane);
    int lane;
};



enum Pc_Info_Index {
    x = 0,      //  中心点x坐标0
    y = 1,      // 中心点y坐标1
    z = 2,      // 中心点z坐标2
    w = 3,      // 宽3
    l = 4,      // 长4
    h = 5,      // 高5
    a = 6,      // 航向角6
    c = 7,      // 类别7
    v = 8,      // 速度8
    all_area_id = 9,        //全域id9
    s = 10,     //置信度10    1
    src = 11,       //来源11
    origin_id = 12,     //原id12
    pc_id = 13,     //基站id13
    color = 14,     //颜色14
    longitude = 15,     //经度15
    latitude = 16,      //纬度16
    pc_x = 17,      // 单基站x17
    pc_y = 18,      // 单基站y18
    cam_id = 19,        //目标在所在基站的哪个相机下 0代表不在相机下19
//    八角点三维坐标
    bottom_front_right_x = 20,      //
    bottom_front_right_y = 21,      //
    bottom_front_right_z = 22,      //
    top_front_right_x = 23,     //
    top_front_right_y = 24,     //
    top_front_right_z = 25,     //
    top_behind_right_x = 26,        //
    top_behind_right_y = 27,        //
    top_behind_right_z = 28,        //
    bottom_behind_right_x = 29,     //
    bottom_behind_right_y = 30,     //
    bottom_behind_right_z = 31,     //
    bottom_front_left_x = 32,       //
    bottom_front_left_y = 33,       //
    bottom_front_left_z = 34,       //
    top_front_left_x = 35,      //
    top_front_left_y = 36,      //
    top_front_left_z = 37,      //
    top_behind_left_x = 38,     //
    top_behind_left_y = 39,     //
    top_behind_left_z = 40,     //
    bottom_behind_left_x = 41,      //
    bottom_behind_left_y = 42,      //
    bottom_behind_left_z = 43,      //

    cur_frame_time = 44,        // 当前帧时间戳
    cur_global_frameid = 45,        // 当前帧id
    left_top_px = 46,       //目标的相机检测角点-左上x
    left_top_py = 47,       //目标的相机检测角点-左上y
    right_bottom_px = 48,       //目标的相机检测角点-右下x
    right_bottom_py = 49,       //目标的相机检测角点-右下y
    head_camera_time = 50,      //头相机视频时间戳
    body_camera_time = 51,      //中间相机视频时间戳
    tail_camera_time = 52       //尾相机视频时间戳
//    lane_no = 53,     //
//    pre_frame = 54        //
};

//储存目标的历史信息
struct History_Info {
public:
    History_Info() {};
    std::vector<double> t; //历史时间戳
    std::vector<double> x; //历史位置x
    std::vector<double> y; //历史位置y
    std::vector<double> v; //历史速度v
    std::vector<double> cal_speed; //目标的计算速度
    std::vector<double> moving_average_speed;//目标的平均速度
    std::vector<double> acceleration;//目标的加速度
    std::vector<double> label;//目标類別
    int speeding_count = 0;//超速累计帧数
    int unspeeding_count = 0;//未超速累计帧数
    int stroll_count = 0;//慢行累计帧数
    double speeding_rate = 0;//超速比率
    double stroll_rate = 0;//慢行比率
    std::vector<double> x_diff;//前后帧x差值
    std::vector<double> y_diff;//前后帧y差值
    std::vector<double> car_l;//车辆长度
    double x_min = 0; //保存位置的x最小
    double x_max = 0; //保存位置的x最大
    double y_max = 0; //保存位置的y最大
    double y_min = 0; //保存位置的y最小
    double normal_count = 0; //正常行驶帧数
    double normal_rate = 0; //正常行驶比率
    std::vector<double> px;
    std::vector<double> py;
};


//根据协议确定类别
//    0         1          2       3           4             5             6             7      8        9         10      11     12     13
//"person", "twocycle", "elme","meituan", "takeoutcar", "tricycle", "expresstricycle", "car", "bus", "tractors", "truck", "cl","block", "box"}
class Detect_Class_Type {
public:
    Detect_Class_Type() {};
    const std::vector<int> non_motor_type = {1, 3};
    const std::vector<int> motor_type = {0, 2, 5, 6, 7, 8};
    const std::vector<int> all = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 54};
    const std::vector<int> target_type = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 54};
    const std::vector<int> big_car = { 2, 5, 6, 7};
    const std::vector<int> car = {0};
    const std::vector<int> bicycle = {1};
    const std::vector<int> bus = {2, 8};
    const std::vector<int> motorcycle = {54};
    const std::vector<int> people = {4};
    const std::vector<int> largetruck = {5, 7};
    const std::vector<int> truck = {5, 6, 7};
    const std::vector<int> none_type = {54};
    const std::vector<int> minibus = {8};
    const std::vector<int> littletruck = {54};
    const std::vector<int> non_people = {0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 54};
    const std::vector<int> spills = {11};
    const std::vector<int> roadwork = {9, 10};
};

class Lane_Direction{
public:
    Lane_Direction(){};
    const int STRAIGHT = 0;
    const int LEFT = 1;
    const int RIGHT = 2;
    const int STRAIGHT_LEFT = 3;
    const int STRAIGHT_RIGHT = 4;
    const int STRAIGHT_LEFT_RIGHT = 5;
    const int UNKOWN = 6;
};
//对当前帧每个目标进行各种信息赋值
class Detect_Info
{
public:
    Detect_Info(){};

    void fill_data(const xt::xarray<double> &origin_data, xt::xarray<int> &px_data,
                   const xt::xarray<double> &boxdata_info);

    double src = -1;
    double cam_id = -1;
    double x = -1;
    double y = -1;
    double z = -1;
    double l = -1;
    double w = -1;
    double h = -1;
    double angle = -1;
    double class_id = -1;
    double coincidence = -1;
    double v = -1;
    double v_director = -1;//daiding
    double origin_id = -1;

    int center_px = -1;
    int center_py = -1;
    int front_left_px = -1;
    int front_left_py = -1;
    int front_right_px = -1;
    int front_right_py = -1;
    int behind_left_px = -1;
    int behind_left_py = -1;
    int behind_right_px = -1;
    int behind_right_py = -1;

    double longitude = -1;
    double latitude = -1;
    double event_mid_camera_id = -1;
    double cur_id_time = -1;
    double cur_global_frameid = -1;
    double left_top_px = -1;
    double left_top_py = -1;
    double right_bottom_px = -1;
    double right_bottom_py = -1;
    double head_camera_time = -1;
    double body_camera_time = -1;
    double tail_camera_time = -1;
    double pc_id = -1;
    double pos_coincidence = -1;
    double pc_x = -1;
    double pc_y = -1;
    // double single_info = -1;//
    double pre_frame = 0;//
    double lane_no = 0;//
    map_event<int, double>::noarm_map cam_id_time_dict;
};

class History_Continue_Lane_Info{
public:
    History_Continue_Lane_Info(int lane, std::shared_ptr<Detect_Info> detectInfo, int v_px, int v_py, double v_px_by_angle,
                               double v_py_by_angle, double time_now);

    int lane;
    std::shared_ptr<Detect_Info> detect_info;
    int v_px;
    int v_py;
    double v_px_by_angle;
    double v_py_by_angle;
    double time;
};

//对当前帧每个车道上每个目标速度进行累计并更新车道平均速度和进行拥堵计算
class Mean_V_Info {
public:
    static int default_mean_v;

    Mean_V_Info() {};

    Mean_V_Info(const double &max_v) : max_v(max_v), mean_v(max_v) {
        statistice_congestion_count[1] = 0;
        statistice_congestion_count[2] = 0;
        statistice_congestion_count[3] = 0;
        statistice_congestion_count[4] = 0;
    };

    void update(const double &v);
    void get_mean_v();
    void clear();

    int cur_num = 0;
    double cur_target_length = 0;
    double cur_target_v = 0;
    double cur_mean_v = 0;
    double cur_occupancy = 0;
    int congestion = Congestion_Level::unblocked;
    std::map<int, int> statistice_congestion_count;
    double mean_v;

    int statistice_congestion = Congestion_Level::unblocked;
private:
    double max_v;
    double sum_v = 0;
    double sum_num = 0;

};

//对当前帧每个车道上目标之间车头间距进行计算, 输出平均车头间距
class Mean_Space_Headway_Info {
public:
    Mean_Space_Headway_Info() {};

    Mean_Space_Headway_Info(const float &max_sh) : max_sh(max_sh), mean_sh(max_sh) {};

    void update(std::vector<float> &sh_list);
    float mean_sh;

    void get_mean_sh();
    void clear();
private:
    float max_sh;
    float sum_sh = 0;
    int sum_num = 0;

    float SumVector(std::vector<float> &vector);

};

//车道信息: 通过车道名查找车道输出编号
class Lane_Fix_Info {
public:
    Lane_Fix_Info() {};

    Lane_Fix_Info(const float &angle, const float &lane_output_name) : angle(angle),
                                                                       lane_output_name(lane_output_name) {};
    float lane_output_name;
private:
    float angle;

};

class Judge_Camera
{
public:
    Judge_Camera(int p1_x, int p1_y, int p2_x, int p2_y);
private:
    int c_px;
    int c_py;

    xt::xarray<int>points_for_camera_boundary;
    bool is_in_this_camera;
    void active();
    float get_angle(std::vector<int>v1, std::vector<int>v2);
    bool donot_contain_0(int px, int py);
    bool contain_0(int px, int py);
};

//计算时间戳\文件读取等工具函数
namespace ops {
    std::string GetLocalTimeWithMs();

    std::string GetLocalTimeWithMs(const int &seconds);

    std::string TimeStampToString_sec(long timestamp); // 时间戳转字符串，秒


    inline long getTimeStamp()
    {
        long t_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); //ms cjm
        return t_now;
    }

    int file_len(const std::string &filename);


    inline void Trim(std::string &str) {
        str.erase(0, str.find_first_not_of(" \t\r\n"));
        str.erase(str.find_last_not_of(" \t\r\n") + 1);
    }

    template<class Type>
    Type stringToNum(const std::string &str) {
        std::istringstream iss(str);
        Type num;
        iss >> num;
        return num;
    }

    template<class Type>
    std::vector<Type> vStrSplit(const std::string &strSur, const char &cConChar) {
        std::vector<Type> output;
        std::string::size_type pos1, pos2;
        pos1 = 0;
        pos2 = strSur.find(cConChar, 0);
        int i = 0;
        while (std::string::npos != pos2) {
            Type key = stringToNum<Type>(strSur.substr(pos1, pos2 - pos1));
            output.push_back(key);
            pos1 = pos2 + 1;
            pos2 = strSur.find(cConChar, pos1);
        }
        output.push_back(stringToNum<Type>(strSur.substr(pos1, strSur.size())));
        return output;
    }

    template<typename T>
    xt::xarray<T> csvReader(const std::string &csv_path) {
        std::ifstream csv_reader;
        csv_reader.open(csv_path);
        std::string line;
        std::size_t k = 0;
        std::vector<T> out;
        while (getline(csv_reader, line)) {
            k++;
            ops::Trim(line);
            std::vector<T> key = ops::vStrSplit<T>(line, ',');
            out.insert(out.end(), key.begin(), key.end());
        }
        if (k == 0)
        {
            xt::xarray<T> res = {0};
            return res;
//            throw("k===0");
        }

        std::vector<std::size_t> shape = {k, std::size_t(out.size() / k)};
        xt::xarray<T> res = xt::adapt(out, shape);
        return res;
    }
}

// 填充车道信息
class Lane_Info
{
public:
    Lane_Info() {};//gouzao hanshu

    void fill_data(const std::vector<double> &origin_data, const xt::xarray<int> &Congestion_Level_highway_table,
                   const xt::xarray<int> &Congestion_Level_main_branch_table);

    void fill_plus_data_for_non_converge(const std::vector<double> &origin_data,
                                         const xt::xarray<int> &negative_y_axis_xy,
                                         const double &theta);
    void fill_plus_data_for_converge(std::shared_ptr<Lane_Info>,
                                     std::shared_ptr<Lane_Info>);

    int out_in = -1;
    int start_px = 0;
    int start_py = 0;
    int end_px = 0;
    int end_py = 0;
    double length = 0.0;
    double width = 0.0;
    double min_v = -1;
    double max_v = -1;
    bool is_emergency_lane = 0;
    bool is_non_motorway = 0;
    bool is_bus_lane = 0;
    double angle_to_north = -1;
    int radar_direct;
    double lane_direction = 0.0;
    double v_px = -1;
    double v_py = -1;


    int unblocked_min_v = -1;
    int slightly_congested_min_v = -1;
    int slightly_congested_max_v = -1;
    int moderately_congested_min_v = -1;
    int moderately_congested_max_v = -1;
    int seriously_congested_min_v = -1;
    int seriously_congested_max_v = -1;
private:

    void get_v_for_congestion_level(const xt::xarray<int> &Congestion_Level_highway_table,
                                    const xt::xarray<int> &Congestion_Level_main_branch_table);

    bool is_angle_in_this_turn_left_lane_normal(float angle);
    bool is_angle_in_this_turn_left_lane_near_0(float angle);
    
    int lane_class = -1;


    
    float angle_in = -1;
    float angle_out = -1;
    float small_angle = -1;
    float big_angle = -1;
    bool is_angle_in_this_turn_left_lane = 0;



};

//对连续帧进行事件检测, 记录连续的疑似事件帧目标信息, 时间\位置\视频检测像素角点等等
class Event_Continue_Info {
public:
    Event_Continue_Info(const double &start_forward_time, const double &time_now, double &coincidence) : //yk:wrong time class should be float
            start_time(time_now - start_forward_time),
            coincidence(coincidence) {
        end_time = start_time;
        global_frame_time = start_time;
    };

    void Assignment(std::shared_ptr<Detect_Info> &detect_info) {
        this->longitude = detect_info->longitude;
        this->latitude = detect_info->latitude;
        this->event_mid_Hcamera_time = detect_info->head_camera_time;
        this->event_mid_Bcamera_time = detect_info->body_camera_time;
        this->event_mid_Tcamera_time = detect_info->tail_camera_time;
        this->event_mid_lon = detect_info->longitude;
        this->event_mid_lat = detect_info->latitude;
        this->camera_id = int(detect_info->cam_id);
        this->pc_id = int(detect_info->pc_id);
        this->global_frame_id = long(detect_info->cur_global_frameid);
        this->left_top_px = detect_info->left_top_px;
        this->left_top_py = detect_info->left_top_py;
        this->right_bottom_px = detect_info->right_bottom_px;
        this->right_bottom_py = detect_info->right_bottom_py;
        this->x = detect_info->x;
        this->y = detect_info->y;

        // cjm 1209
        this->v = int(detect_info->v * 100); // cm/s
        this->lane = int(detect_info->lane_no);


    };
    double start_time;//yk:wrong long->float
    double end_time;
    int hit_count = 1;
    int event_no = -1;
    double longitude = 0;
    double latitude = 0;
    double coincidence;
    double event_mid_Hcamera_time = 0;
    double event_mid_Bcamera_time = 0;
    double event_mid_Tcamera_time = 0;
    double event_mid_lon = 0;
    double event_mid_lat = 0;
    int camera_id = 0;
    int pc_id = 0;
    double global_frame_time;//yk:wrong long to float
    int global_frame_id = 0;
    double left_top_px = 0;
    double left_top_py = 0;
    double right_bottom_px = 0;
    double right_bottom_py = 0;
    double x = 0;
    double y = 0;

    // cjm 1209
    int v = 0;
    int lane = 0;
    std::vector<std::vector<double>> eventInfoList;

};



struct Cur_Time {
public:
    Cur_Time(const int &t1, const int &t2) : times(t1), cur_time(t2) {};
    int times;
    long cur_time;
};

// 对连续帧目标进行轨迹评价, 保存对应的评价结果, 判断速度\位置\类别\尺寸\预测帧数等异常情况
struct History_Continue_Trajectory_Eval {
public:
    History_Continue_Trajectory_Eval(const int &lane,
//                                     const long &time_now,  //yk:wrong long can not save the dot
                                     const double &time_now,  //yk:wrong long can not save the dot
                                     const double &x,
                                     const double &y,
                                     const double &pos_anno_coin = 0,
                                     const double &pos_anno_vel_coin = 0,
                                     const double &vel_std = 0,
                                     const int &length_std = 0,
                                     const double &pre_cnt_rate = 0,
                                     const double &pose_x_std = 0,
                                     const double &pose_x_diff_mean = 0) :
            lane(lane),
            time(time_now),
            x(x),
            y(y),
            pos_anno_coin(pos_anno_coin),
            pos_anno_vel_coin(pos_anno_vel_coin),
            vel_std(vel_std),
            length_std(length_std),
            pose_x_std(pose_x_std),
            pre_cnt_rate(pre_cnt_rate),
            pose_x_diff_mean(pose_x_diff_mean) {
        for (int i = 1; i < 23; ++i) {
            coincidence[i] = -1;
            final_coincidence[i] = -1;
        }
    };
    int lane;
//    long time; //yk:wrong
    double time; //yk:wrong
    double x;
    double y;
    double pos_anno_coin = 0;
    double pos_anno_vel_coin = 0;
    double vel_std = 0;
    int length_std = 0;
    double pre_cnt_rate = 0;
    double pose_x_std = 0;
    double pose_x_diff_mean = 0;

    std::map<int, double> coincidence;
    std::map<int, double> final_coincidence;
};

//
struct EventList{
public:
    EventList(){};
    double laneNo;
    int laneType;
    double laneDirection;
    double laneSpeedLimit;
    double stationNum;
    std::vector<std::vector<int>> stationCongestionList;
    double followPercent;
    double timeOccupancy;
    double headSpaceAvg;
    double carFlow;
    double truckFlow;
    double busFLow;
    double mediumBusFlow;
    double largeTruckFlow;
    double carSpeedAvg;
    double truckSpeedAvg;
    double busSpeedAvg;
    double mediumBusSpeedAvg;
};

struct B4_res{
public:
    B4_res(){};
    std::string startTimeStamp;
    std::string endTimeStamp;
    int laneNum;
    std::vector<std::shared_ptr<EventList>> eventList;
};

// cjm 0927
struct B4_context_info
{
public:
    B4_context_info(const int & item0, const double & item1, const double & item2, const double & item3, const double & item4, 
                    const int & item5, const std::vector<std::vector<double>> & item6, const double & item7, const double & item8, 
                    const double & item9, const double & item10, const double & item11, const double & item12, const double & item13, 
                    const int & item14, const double & item15, const double & item16, const double & item17, const double & item18, 
                    const double & item19, const double & item20, const double & item21, const double & item22, const double & item23, 
                    const double & item24)
            :item0(item0),
             item1(item1),
             item2(item2),
             item3(item3),
             item4(item4),
             item5(item5),
             item6(item6),
             item7(item7),
             item8(item8),
             item9(item9),
             item10(item10),
             item11(item11),
             item12(item12),
             item13(item13),
             item14(item14),
             item15(item15),
             item16(item16),
             item17(item17),
             item18(item18),
             item19(item19),
             item20(item20),
             item21(item21),
             item22(item22),
             item23(item23),
             item24(item24){};
    int item0;
    double item1;
    double item2; 
    double item3; 
    double item4; 
    int item5;
    std::vector<std::vector<double>> item6;
    double item7; 
    double item8; 
    double item9; 
    double item10;
    double item11; 
    double item12; 
    double item13; 
    int item14;
    double item15; 
    double item16; 
    double item17; 
    double item18; 
    double item19; 
    double item20; 
    double item21; 
    double item22; 
    double item23; 
    double item24;
};


struct B5_event_info
{
public:
//    B5_event_info(const int &item0, const int & item1, const double & item2, const float & item3, const int & item4,
//                  const std::vector<int> & item5
//                  ){};
    B5_event_info(const int &item0, const int & item1, const double & item2, const float & item3, const float & item4,
                  const float & item5
    ){};
//    B5_event_info(const int & item0, const int & item1, const double & item2, const float & item3, const int & item4,
//                const std::vector<std::vector<int>> & item5, const double & item6, const double & item7, const long & item8,
//                const int & item9, const int & item10, const double & item11, const long & item12,
//                const std::vector<std::vector<double>> & eventInfoList)
    B5_event_info(const int & item0, const int & item1, const double & item2, const float & item3, const float & item4, const float & item5, const int & item6, const int & item7,
                  const int & item8, const std::vector<std::vector<double>> & item9, const double & item10, const double & item11,
                  const double & item12, const double & item13,
                  const std::vector<std::vector<double>> & eventInfoList)
//        :item0(item0),
//         item1(item1),
//         item2(item2),
//         item3(item3),
//         item4(item4),
//         item5(item5),
//         item6(item6),
//         item7(item7),
//         item8(item8),
//         item9(item9),
//         item10(item10),
//         item11(item11),
//         item12(item12),
//         eventInfoList(eventInfoList){};
            :item0(item0),
             item1(item1),
             item2(item2),
             item3(item3),
             item4(item4),
             item5(item5),
             item6(item6),
             item7(item7),
             item8(item8),
             item9(item9),
             item10(item10),
             item11(item11),
             item12(item12),
             item13(item13),
             eventInfoList(eventInfoList){};
//    int item0;
//    int item1;
//    double item2;
//    float item3;
//    int item4;
//    std::vector<std::vector<int>> item5;
//    double item6;
//    double item7;
//    long item8;
//    int item9;
//    int item10;
//    double item11;
//    long item12;
//    std::vector<std::vector<double>> eventInfoList;
    int item0;
    int item1;
    double item2;
    float item3;
    float item4;
    float item5;
    int item6;
    int item7;
    int item8;
    std::vector<std::vector<double>> item9;
    double item10;
    double item11;
    double item12;
    double item13;
    std::vector<std::vector<double>> eventInfoList;
};

class Event_Output {
public:
//    int m_input4;
    int out_normal_lane_num;
    int out_input4;
    int out_B3_cnt;
    int out_B4_cnt;
    std::vector<std::vector<double>> out_content;
    std::vector<std::shared_ptr<B4_context_info>> out_context;
    // std::vector<std::vector<double>> out_context; // cjm 0927

    std::vector<std::shared_ptr<B5_event_info>> out_all_event_output_for_B5;

// duoge chushihua hanshu
public:
    Event_Output();
    Event_Output(const int & input0,  //Event_Output for b3
                 const int & input1,
                 const int & input2,
                 const int & input3,
                 const int & B3_cnt,
                 const int & normal_lane_num,
                 const int & input4,
                 std::vector<std::vector<double>> &content,
                 const int & input5)
    {
//                    m_input4 = input4;
        out_normal_lane_num = normal_lane_num;
        out_content = content;
        out_B3_cnt = B3_cnt;
    };

    Event_Output(const int & input0,//Event_Output for b4
                 const int & input1,
                 const int & input2,
                 const int & input3,
                 const int & input4,
                 const int & input5,
                 const int & B4_cnt,
                 const int & normal_lane_num,
                 const int & input6,
                 const std::vector<std::shared_ptr<B4_context_info>> & context)
    {
        // const std::vector<std::vector<double>> & context  // cjm 0927
        // const std::vector<std::vector<std::vector<double>>> & context2
        out_context = context;
        out_B4_cnt = B4_cnt;
    };
    Event_Output(const int & input0,
                 const int & input1,
                 const int & input2,
                 const int & input3,

                 const int & input4,
                 const int & input5,
                 const std::vector<std::shared_ptr<B5_event_info>> & all_event_output_for_B5,
                 const int & input6)
    {
        out_all_event_output_for_B5 = all_event_output_for_B5;
        out_input4 = input4;
    };
private:
    std::vector<std::vector<double>> B3_content;
};

struct Point_Now
{
public:
    Point_Now(const int & event_id, const int & target_id, const int & lane, const long & time, const std::string & str_time,
              const double & x, const double & y, const int & pos_anno_coin, const int & pos_anno_vel_coin, const double & vel_std,
              const int & length_std,const double & pose_x_std,const double & pose_x_diff_mean,const double & pre_cnt_rate,
              const std::vector<std::vector<double>> &eventInfoList):
            event_id(event_id),
            target_id(target_id),
            lane(lane),


            time(time),
            str_time(str_time),
            x(x),
            y(y),
            pos_anno_coin(pos_anno_coin),
            pos_anno_vel_coin(pos_anno_vel_coin),
            vel_std(vel_std),
            length_std(length_std),
            pose_x_std(pose_x_std),
            pose_x_diff_mean(pose_x_diff_mean),
            pre_cnt_rate(pre_cnt_rate),
            eventInfoList(eventInfoList) {};
    int event_id;
    int target_id;
    int lane;
    long time;
    std::string str_time;
    double x;
    double y;
    int pos_anno_coin;
    int pos_anno_vel_coin;
    double vel_std;
    int length_std;
    double pose_x_std;
    double pose_x_diff_mean;
    double pre_cnt_rate;
    std::vector<std::vector<double>> eventInfoList;
};

struct history_all_event_output_for_B5_info{
public:
    history_all_event_output_for_B5_info():end_time(-1),m_list({}) {};
    history_all_event_output_for_B5_info(const long & end_time, const std::vector<double> & m_list):end_time(end_time), m_list(m_list){};
    long end_time;
    std::vector<double> m_list;
    std::shared_ptr<B5_event_info> event;
};

class Traffic_Flow {
public:
    Traffic_Flow() {};

    Traffic_Flow(const std::string &lane_config_path, const int &pc_num);

    ~Traffic_Flow();

    void use(xt::xarray<double> &input_info, const bool &statistics_flag, xt::xarray<double> &boxes_final, int stationId);

    void get_B3(std::shared_ptr<Event_Output> & event_res);

    void get_B4(std::shared_ptr<Event_Output> & event_res);

    std::shared_ptr<Event_Output> get_B5();
    std::string m_config_path;
    std::string B5_all;
    std::map<int, double> not_follow_lane_status;

//    Lon_Lat_Meter_Transform *lon_lat_meter_transform;
    xt::xarray<int8_t> config_matrix;
    std::map<int, int> lane_no_lane_name;
    std::vector<std::vector<double>> lane_origin_info;
    std::map<int, std::vector<int>> lane_match_pair;
    std::map<std::string, double> param;
    std::map<int, std::vector<float>> lane_xy; // 初始化车道停止线中心点坐标
    std::vector<float> forbid_over_lane; //x_min,x_max,y_min,y_max
    // std::map<int, std::vector<float>> lane_xy; // cjm 0901 chushi

    map_event<int, std::shared_ptr<Detect_Info>>::noarm_map cur_id_info;
    map_event<int, int>::noarm_map cur_id_lane;

    std::map<int, std::shared_ptr<KDTree>> lane_name_centerline_pcd_tree;
    std::map<int, xt::xarray<float>> lane_name_centerline_mediumpoint;
    std::map<int, std::vector<int>> lane_branches;

    std::map<std::string, std::vector<int>> pedstrain_waiting_area_for_crosswalk; // cjm 0921
    std::vector<int> pedstrain_waiting_area; // cjm 0921

    json map_param;

private:
    std::ofstream use_time_traffic_fd;

    std::vector<std::vector<double>> points;

//    std::vector<std::vector<double>> worinima;

    std::shared_ptr<KDTree> pcd_tree;

    int lane_num_switch(int old_lane_num);

    int dir_num_switch(int old_dir_num);

    void lonlat_to_xyz_batch(xt::xarray<double> &box, double &lon0, double &lat0, double &angle_north);

    xt::xarray<double> cal_trans(double x, double y, double z);

    xt::xarray<float> load_info(const std::string &config_path);

    void get_map_param(const std::string &lane_config_path);

    void load_lane_num(const std::string &config_path);

    void generate_matrix_data(const std::string &lane_config_path); 

    void generate_lane_origin_info(const std::string &lane_config_path);

    void get_traffic_param(const std::string &config_path);

    void save_config_matrix_info(const std::string &config_path, xt::xarray<int8_t> matrix); 

    void save_lane_no_lane_name_info(const std::string &config_path, std::map<int, int>lane_no_name); 

    int init_event_number(const std::string &lane_config_path);

    void get_negative_y_axis_xy();

    void format_lane_info(const std::string &lane_config_path);

    void init_total_info();

    void init_abnormal_event_sign(const std::string &lane_config_path);

    void init_abnormal_event_continue_info();

    void init_history_abnormal_event();

    void init_lane_fix_info(const std::string &lane_config_path);

    void init_history_signal_light();

    void init_judge_camera();

    void init_for_nearest_out_lane();

    void init_pc_bmp_xy();

    void get_pre_info(xt::xarray<double> &input_info, xt::xarray<double> &boxes_final);

    void init_cur_info();

    void get_cur_id_info(xt::xarray<double> &input_info, xt::xarray<double> &boxes_final);

    void get_original_box_lane(xt::xarray<double> &boxes_final);

    std::map<int, int> match_org_track_res(xt::xarray<double> &input_info, xt::xarray<double> &boxes_final);

    void get_current_lane_id();

    void get_id_count();

    void save_history_id_speed();

    void get_instantaneous_info();

    void get_sorted_car_info_by_dis_to_out();

    void get_space_headway();

    void get_queue_length_by_space_headway();

    void get_lane_occupancy();

    void get_cur_lane_classes_num();

    void get_cur_lane_mean_v();

    void get_queue_waitingtime(); // cjm 0921

    void get_cur_crosswalk(); // cjm 0921

    /**3 计算统计量*/
    void get_statistics_info(const bool &statistics_flag, xt::xarray<double> &boxes_final);

    void get_history_lane_id_info_of_section();

    void get_time_headway_section(const bool &statistics_flag);

    void get_follow_car_percent(const bool &statistics_flag);

    void get_time_occupancy_by_id_time(const bool &statistics_flag);

    void get_lane_flow(const bool &statistics_flag);

    void get_lane_mean_v_info_by_section(const bool &statistics_flag);

    void get_lane_mean_v_info_by_all_v(const bool &statistics_flag, xt::xarray<double> &boxes_final);

    void get_local_static_congestion_state(const bool &statistics_flag);

    void get_lane_mean_space_headway_info(const bool &statistics_flag);

    void clear_history_lane_id_info_of_section(const bool &statistics_flag);

    int judge_cur_congestion(double &cur_mean_v, const double &cur_occupancy, const int &cur_num);

    /**""" 4 异常事件检测"""*/
    void detect_abnormal_event(const bool &statistics_flag);

    void init_abnormal_event_info();

    void clear_id_info();

    void get_cur_pix_v();

    void get_cur_pix_v_by_angle();

    void trajectory_evalute(int &id, std::shared_ptr<Detect_Info> &info);

    void get_id_continue_speeding_info(int &id, std::shared_ptr<Detect_Info> &info,
                                       std::set<int> &all_unhit_id_continue_speeding_for_motor,
                                       std::set<int> &all_unhit_id_continue_speeding_for_non_motor);

    void update_now_and_history(int &idx, double &coincidence,
                                std::map<int, std::shared_ptr<Event_Continue_Info>> &idx_continue_info,
                                std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> &history_event,
                                std::set<int> &unhit_idxs,
                                const float &start_forward_time = 0.0); //yk:wrong long -> float

    void get_id_continue_stroll_info(int &id, std::shared_ptr<Detect_Info> &info,
                                     std::set<int> &all_unhit_id_continue_stroll);

    void get_id_continue_occupy_dedicated_lane_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                    std::set<int> &all_unhit_id_continue_occupy_dedicated_lane,
                                                    std::set<int> &all_unhit_id_continue_people_occupy_motor_lane,
                                                    std::set<int> &all_unhit_id_continue_non_motor_occupy_motor_lane);

    void get_id_continue_occupy_emergency_lane_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                    std::set<int> &all_unhit_id_continue_occupy_emergency_lane);

    // cjm add 0902
    void get_id_continue_occupy_bus_lane_info(std::set<int> &all_unhit_id_continue_occupy_bus_lane);


    
    void get_id_continue_not_following_lane_guide(int &id, std::shared_ptr<Detect_Info> &info,
                                                  std::set<int> &all_unhit_id_continue_not_following_lane_guide);

    // void get_id_continue_stop_info(int &id, std::shared_ptr<Detect_Info> &info, std::set<int> &all_unhit_id_continue_stop);

    void get_id_continue_stop_info(std::set<int> &all_unhit_id_continue_stop);

    // void get_id_continue_retrograde_info(int &id, std::shared_ptr<Detect_Info> &info,
    //                                      std::set<int> &all_unhit_id_continue_retrograde_for_motor,
    //                                      std::set<int> &all_unhit_id_continue_retrograde_for_non_motor);

    void get_id_continue_retrograde_info(std::set<int> &all_unhit_id_continue_retrograde_for_motor,
                                         std::set<int> &all_unhit_id_continue_retrograde_for_non_motor);
    //xcb note
    // void get_id_continue_change_lanes_info(int &id, std::shared_ptr<Detect_Info> &info,
    //                                        std::set<int> &all_unhit_id_continue_change_lanes);

    void get_id_continue_change_lanes_info(std::set<int> &all_unhit_id_continue_change_lanes);

    void get_id_continue_stop_over_terminateline(std::set<int> &all_unhit_id_continue_stop_over_terminateline,
                                                std::set<int> &all_unhit_id_continue_stop_over_terminateline_for_motor);

    void get_spills_coordinate_continue_info(int &id, std::shared_ptr<Detect_Info> &info,
                                             std::set<int> &all_unhit_spills_coordinate_continue);

    void get_construction_coordinate_continue_info(int &id, std::shared_ptr<Detect_Info> &info,
                                             std::set<int> &all_unhit_construction);

    void get_congestion_continue_info(int &id, std::shared_ptr<Detect_Info> &info,
                                      std::set<int> &all_unhit_construction_sign_continue);

    void get_traverse_continue_info(int &id, std::shared_ptr<Detect_Info> &info,
                                    std::set<int> &all_unhit_traverse_continue);

    void get_big_car_in_dedicated_lane_continue_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                     std::set<int> &all_unhit_big_car_continue);

    std::map<int, int> input_land_box_process();

    void judge_speeding(std::set<int> &all_unhit_id_continue_speeding_for_motor,
                        std::set<int> &all_unhit_id_continue_speeding_for_non_motor);

    void judge_stroll(std::set<int> &all_unhit_id_continue_stroll, std::map<int, int> &lane_box_sorted);

    void detect_occupy_dedicated_lane(std::set<int> &all_unhit_id_continue_occupy_dedicated_lane,
                                      std::set<int> &all_unhit_id_continue_people_occupy_motor_lane,
                                      std::set<int> &all_unhit_id_continue_non_motor_occupy_motor_lane);

    void detect_car_not_following_lane_guide(std::set<int> &all_unhit_id_continue_not_following_lane_guide);

    void detect_occupy_emergency_lane(std::set<int> &all_unhit_id_continue_occupy_emergency_lane);

    void detect_occupy_bus_lane(std::set<int> &all_unhit_id_continue_occupy_bus_lane); // cjm 0902 add 

    void detect_stop_over_terminateline(std::set<int> &all_unhit_id_continue_stop_over_terminateline,
                                        std::set<int> &all_unhit_id_continue_stop_over_terminateline_for_motor);

    void judge_illegal_stop(std::set<int> &all_unhit_id_continue_stop, std::map<int, int> &lane_box_sorted);

    void judge_retrograde(std::set<int> &all_unhit_id_continue_retrograde_for_motor,
                          std::set<int> &all_unhit_id_continue_retrograde_for_non_motor,
                          std::map<int, int> &lane_box_sorted);

    void judge_change_lanes(std::set<int> &all_unhit_id_continue_change_lanes, std::map<int, int> &lane_box_sorted);

    void judge_congestion_detect(std::set<int> &all_unhit_congestion_continue);

    void judge_big_car_in_dedicated_lane(std::set<int> &all_unhit_big_car_in_dedicated_lane_continue);

    void judge_traverse(std::set<int> &all_unhit_traverse_continue);

    void spills_detect(std::set<int> &all_unhit_spills_coordinate_continue);

    void judge_construction(std::set<int> &all_unhit_construction);

    void accident_detect();

    void save_id_info();

    void get_accident_coordinate_continue_info();


    template<typename T>
    xt::xarray<T> lidar_to_bmp(const xt::xarray<T> &lidar_coordinate_xy, const int &x_index, const int &y_index) {
        int sx = param["sx"] * param["meter_to_pixel"];
        int sy = param["sy"] * param["meter_to_pixel"];
        int ex = param["ex"] * param["meter_to_pixel"];
        int ey = param["ey"] * param["meter_to_pixel"];

        xt::xarray<T> bmp_xy = lidar_coordinate_xy * param["meter_to_pixel"];
        xt::view(bmp_xy, xt::all(), y_index) *= -1;
        xt::view(bmp_xy, xt::all(), x_index) += (50 - sx);
        xt::view(bmp_xy, xt::all(), y_index) += (50 + sy);
        bmp_xy *= param["bmp_ratio"];
        std::cout << "liluo-------------------> bmp_xy: " << bmp_xy << std::endl;
        return bmp_xy;
    };

    template<typename T1, typename T2>
    xt::xarray<T2>
    lidar_to_bmp(xt::xarray<T1> &lidar_coordinate_xy, std::vector<int> &x_index, std::vector<int> &y_index) {
        int sx = param["sx"] * param["meter_to_pixel"];
        int sy = param["sy"] * param["meter_to_pixel"];
        int ex = param["ex"] * param["meter_to_pixel"];
        int ey = param["ey"] * param["meter_to_pixel"];
//        xt::xarray<T1> temp =

        lidar_coordinate_xy *= param["meter_to_pixel"];
       for (const auto &i: x_index) {     //yk: python code has no this part
           xt::view(lidar_coordinate_xy, xt::all(), i) += (50 - sx);
       }
       for (const auto &i: y_index) {
           xt::view(lidar_coordinate_xy, xt::all(), i) *= -1.0;
           xt::view(lidar_coordinate_xy, xt::all(), i) += (50 + sy);
       }

        // for (const auto &i: y_index) {
        //     xt::view(lidar_coordinate_xy, xt::all(), i) *= -1.0;  //yk:fanzhuan
        //     xt::view(lidar_coordinate_xy, xt::all(), i) += (-1 + sy);//yk:pingyi
        // }
        // for (const auto &i: x_index) {
        //     xt::view(lidar_coordinate_xy, xt::all(), i) += (-1 - sx);
        // }
        lidar_coordinate_xy *= param["bmp_ratio"];
        xt::xarray<T2> bmp_xy = xt::cast<T2>(lidar_coordinate_xy);
        return bmp_xy;
    };

    xt::xarray<int>
    lidar_to_bmp(xt::xarray<double> &lidar_coordinate_xy, std::vector<int> &x_index, std::vector<int> &y_index);

    xt::xarray<int> bmp_to_congestion_map(const xt::xarray<int> &bmp_xy);

    xt::xarray<int> bmp_to_spills_coordinate(const xt::xarray<int> &bmp_xy);

    xt::xarray<int> bmp_to_accident_coordinate(const xt::xarray<int> &bmp_xy);

    xt::xarray<uint8_t> congestion_matrix;
    xt::xarray<uint8_t> spills_matrix;
    xt::xarray<uint8_t> accident_matrix;
    xt::xarray<double> rotate_station;

    std::vector<std::shared_ptr<Judge_Camera>> judge_camera_lists;


    int B3_cnt = 0;
    int B4_cnt = 0;
    int event_cnt=0;

    std::map<int, double> history_events;
    int pc_num;
    int normal_lane_str_len;
    xt::xarray<int> negative_y_axis_xy;  //  { 0, 10}
    std::map<int, std::shared_ptr<Lane_Info>> lane_info;
    std::map<int, std::shared_ptr<Lane_Info>> turn_left_info;
//    map_event<std::map<int, int>>::triple_map history_lane_id_info_of_section;
    map_event<std::map<int, int>>::triple_map_str total_statistics;
    std::map<int,std::map<std::string ,std::vector<double>>> total_statistics1;
    map_event<std::shared_ptr<Mean_V_Info>>::triple_map_multi_type lane_mean_v_info_by_all_v;
    map_event<int>::triple_map_multi_type lane_local_static_congestion_state;
    map_event<std::shared_ptr<Mean_V_Info>>::triple_map_str lane_mean_v_info_by_section;
    map_event<int, std::shared_ptr<Mean_Space_Headway_Info>>::noarm_map lane_mean_space_headway_info;
    map_event<std::string, bool>::noarm_map event_sign;
    map_event<int, std::shared_ptr<Lane_Fix_Info>>::noarm_map lane_fix_info;
    map_event<int, std::vector<double>>::noarm_map base_station_pos;
//    map_event<int, std::shared_ptr<Detect_Info>>::noarm_map cur_id_info;
    map_event<int, std::vector<int>>::noarm_map cur_lane_id;
    map_event<int, std::vector<int>>::noarm_map cur_lane_id_for_flow;
    map_event<int, std::shared_ptr<Cur_Time>>::noarm_map id_count;
    map_event<int, std::vector<std::shared_ptr<History_Different_Lane_Info>>>::noarm_map id_history_lane;
    map_event<int, std::shared_ptr<History_Info>>::noarm_map history_speed_info;

    map_event<int, std::map<int, int>>::noarm_map cur_lane_classes_num;
    map_event<int, std::map<std::string, int>>::noarm_map cur_lane_classes_num2;   //python中途换类型，没办法只能新建一个了

    map_event<int, std::map<int, std::vector<int>>>::noarm_map cur_lane_pc_car_num;
    map_event<std::vector<float>>::triple_map_str cur_statistics;
    map_event<int, double>::noarm_map cur_lane_mean_v;
    map_event<int, std::vector<int>>::noarm_map cur_lane_sorted_id;
    map_event<double>::quadra_map history_lane_id_info_of_section;   // {lane: {class_id: {car_id: {v, cur_time, cur_frame_id}}}}
    map_event<std::set<int>>::triple_map history_lane_motor_id_with_class_id;
    map_event<std::vector<double>>::triple_map history_lane_id_time_info;
    map_event<int, std::vector<std::vector<double>>>::noarm_map history_lane_id_order_of_section;   // {lane: {{car_id, class_id}}}

    map_event<std::vector<double>>::triple_map history_pedstrian_waiting_area;
    map_event<std::vector<double>>::triple_map history_car_waiting_area;
    map_event<int, std::vector<int>>::noarm_map cur_id_pix_v;
    map_event<int, std::vector<double>>::noarm_map cur_id_pix_v_by_angle; //# 航向角方向的速度向量

    std::shared_ptr<Detect_Class_Type> classType = std::make_shared<Detect_Class_Type>();
    std::shared_ptr<Lane_Direction> laneDirection = std::make_shared<Lane_Direction>();
    int cur_frame_id;
    double cur_time;//yk:python is float, but this is long int
    bool del_flag;

    std::vector<int> cur_occupy_dedicated_lane;
    std::vector<int> cur_people_occupy_motor_lane;
    std::vector<int> cur_non_motor_occupy_motor_lane;
    // std::vector<int> cur_id_pix_v;
    // std::vector<int> cur_id_pix_v_by_angle;
    std::vector<int> cur_retrograde_for_motor;
    std::vector<int> cur_retrograde_for_non_motor;
    std::vector<int> cur_speeding_for_motor;
    std::vector<int> cur_speeding_for_non_motor;
    std::vector<int> cur_stroll;
    std::vector<int> cur_cross_line;
    std::vector<int> cur_illegal_stop;
    std::vector<int> cur_cross_lane_for_non_motor;
    std::vector<int> cur_cross_lane_for_people;
    std::vector<int> cur_cross_lane_for_motor;
    std::vector<int> cur_run_the_red_light_for_motor;
    std::vector<int> cur_run_the_red_light_for_people;
    std::vector<int> cur_occupy_bus_lane;
    std::vector<int> cur_change_lanes_illegal;
    std::vector<int> cur_construction_sign;
    std::vector<int> cur_spills;
    std::vector<int> cur_construction;
    std::vector<int> cur_accident;
    std::vector<int> cur_roadwork;
    std::vector<int> cur_person_not_walk_zebra_stripe;
    std::vector<int> cur_occupy_zebra_stripe;
    std::vector<int> cur_not_following_lane_guide;
    std::vector<int> cur_stop_over_terminateline;
    std::vector<int> cur_stop_over_terminateline_for_motor;
    std::vector<int> cur_occupy_emergency_lane;
    std::vector<int> cur_congestion;
    std::vector<int> cur_traverse;
    std::vector<int> cur_big_car_in_dedicated_lane;

    std::vector<int> out_lane;
    xt::xarray<int> pxy_of_out_lane;

    map_event<int, std::vector<std::vector<double>>>::noarm_map id_dist_list;
    std::map<int, int> lane2zone;
    std::map<int, int> zone2bus;
    // map_event<std::shared_ptr<History_Continue_Trajectory_Eval>>::triple_map id_trajectory_eval;
    std::map<int, std::vector<std::shared_ptr<History_Continue_Trajectory_Eval>>> id_trajectory_eval;
    std::map<int, std::vector<std::shared_ptr<History_Continue_Lane_Info>>> id_last_lane;
    std::map<int, double>id_states_of_red_light_for_people;
    std::map<int, int> cur_lane_box_info;       //

    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_occupy_dedicated_lane_info;  // # 记录当前id连续占用其他车道的信息,包括机动车占用非机动车道,非机动车占用机动车道
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_people_occupy_motor_lane_info;  // # 记录当前行人连续占用机动车道的信息
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_non_motor_occupy_motor_lane_info; //非机动车在机动车道逗留
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_occupy_emergency_lane_info;  // # 记录目标连续占用紧急车道的信息，占用紧急车道
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_retrograde_info_for_motor;  // # 记录目标连续逆行的信息,机动车逆行
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_retrograde_info_for_non_motor; //  # 记录目标连续逆行的信息,非机动车逆行
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_speeding_info_for_motor;  //# 记录当前id连续超速的信息,机动车超速
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_speeding_info_for_non_motor;  //# 记录当前id连续超速的信息,非机动车超速
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_stroll_info;  // # 记录当前id连续慢行的信息,机动车慢行
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_cross_line_info;  // # 记录当前id连续压线的信息,机动车压线
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_stop_info;  // # 记录当前机动车id连续停的信息,机动车违停
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_cross_lane_info_for_people; // # 记录当前id连续横穿马路的信息,行人横穿马路
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_cross_lane_info_for_motor; // # 记录当前id连续横穿马路的信息,机动车横穿马路
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_cross_lane_info_for_non_motor; // # 记录当前id连续横穿马路的信息,非机动车横穿马路
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_run_the_red_light_info_for_motor; // # 记录当前id连续闯红灯的信息,包括机动车闯红灯,行人闯红灯
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_run_the_red_light_info_for_people; // # 记录当前id连续闯红灯的信息,包括机动车闯红灯,行人闯红灯
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_occupy_bus_lane_info; // # 记录当前id连续占用公交车道的信息
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_change_lanes_info; // # 记录当前id连续变道的信息
    std::map<int, std::shared_ptr<Event_Continue_Info>> spills_coordinate_continue_info; // # 记录当前位置连续检测出抛洒物的信息
    std::map<int, std::shared_ptr<Event_Continue_Info>> construction_continue_info;  // 记录当前位置连续检测出施工牌的信息
    std::map<int, std::shared_ptr<Event_Continue_Info>> accident_coordinate_continue_info; // # 记录当前位置连续检测出交通事故的信息
    std::map<int, std::shared_ptr<Event_Continue_Info>> roadwork_coordinate_continue_info;  // 记录当前位置连续检测出道路施工的信息
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_person_not_walk_zebra_stripe;  // 记录当前id的行人连续未走斑马线
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_stop_in_zebra_stripe;  //记录当前车辆占用斑马线
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_not_following_lane_guide;  //记录当前车辆不按车道导向行驶
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_stop_over_terminateline;   //记录当前越过停止线听停车
    std::map<int, std::shared_ptr<Event_Continue_Info>> id_continue_stop_over_terminateline_for_motor;   //记录当前越过停止线听停车
    std::map<int, std::shared_ptr<Event_Continue_Info>> construction_sign_continue_info;
    std::map<int, std::shared_ptr<Event_Continue_Info>> congestion_continue_info;
    std::map<int, std::shared_ptr<Event_Continue_Info>> big_car_in_dedicated_lane_continue_info;
    std::map<int, std::shared_ptr<Event_Continue_Info>> traverse_continue_info;


    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> confirm_run_the_red_light_info;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> confirm_not_following_info;
    std::map<int, double>confirm_not_following_info_cur_time;//xcb add
    std::map<int, int>confirm_not_following_info_lane;//xcb add
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_speeding_for_motor;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_speeding_for_non_motor;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_stroll;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_occupy_dedicated_lane;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_people_occupy_motor_lane;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_non_motor_occupy_motor_lane;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_retrograde_for_motor;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_retrograde_for_non_motor;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_cross_line;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_illegal_stop;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_congestion;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_traverse;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_big_car_in_dedicated_lane;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_cross_lane_for_non_motor;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_cross_lane_for_people;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_cross_lane_for_motor;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_run_the_red_light_for_motor;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_run_the_red_light_for_people;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_occupy_bus_lane;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_change_lanes;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_spills;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_construction;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_accident;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_roadwork;  // 道路施工
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_person_not_walk_zebra_stripe;  // 行人未走斑马线
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_occupy_emergency_lane;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_not_following_lane_guide; //不按车道线行驶
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_car_in_zebra_stripe;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_stop_over_terminateline;
    std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> history_stop_over_terminateline_for_motor;


    std::vector<std::map<int, std::shared_ptr<Detect_Info>>> last_id_info_0;  //此处python的容器存放了多种类型，C++难以实现，因此一分为二
    std::vector<double> last_id_info_1;
    
    std::map<int, std::shared_ptr<B4_res>> save_b4;
    std::vector<std::shared_ptr<B4_context_info>> B4_context; // cjm 0927

    // std::map<int, std::map<std::string, std::shared_ptr<B4_res>>> save_b4;
    std::map<int, std::shared_ptr<history_all_event_output_for_B5_info>> history_all_event_output_for_B5;
    std::map<int, std::shared_ptr<history_all_event_output_for_B5_info>> history_all_event_output_for_B5_all; // cjm 0824

    bool has_new_event;
    std::vector<std::shared_ptr<B5_event_info>> B5_event;
    std::vector<std::shared_ptr<B5_event_info>> all_event_output_for_B5;
    void get_event_output_for_B5();
    void judge_new_event(std::vector<int> & cur_event, std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> &history_infos);
    void del_long_time_miss_history(std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> &history_event);
    void one_event_info_for_B5(std::vector<int> & cur_event, std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> &history_infos, const int & abnormal_class_index);
    std::vector<double> find_right_video_time(const int & flag, const int & frame_id, const int & id, const int & pc_id);
    std::map<int, std::map<int, std::vector<std::shared_ptr<Point_Now>>>> event_save_by_my_own;
};


void run_event_alg();

#endif //EVENT_ALG_TRAFFIC_FLOW_MATRIX_H
