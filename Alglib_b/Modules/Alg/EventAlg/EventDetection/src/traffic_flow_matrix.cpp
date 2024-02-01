//
// Created by root on 7/1/22.
//

#include "traffic_flow_matrix.h"
#include <algorithm>
#include <typeinfo>
#include "Log/glog/logging.h"
using namespace std;
using namespace xt::placeholders;

namespace {
    int None = -1000000;
//    std::vector<int> id_temp = {
//            5002620, 5002644, 5002696, 5002717, 5002718, 5002784, 5002928, 5002941, 5003057, 5003056, 5003200, 5003360,
//            5003369, 5003436, 5003481, 5003486, 5003520, 5003596, 5003590, 5003611, 5003731, 5003821, 5003851, 5003908,
//            5003973, 5003952, 5004003, 5004010, 5004084, 5004106, 5004132, 5004135, 5004149, 5004447, 5004683, 5004753,
//            5004851, 5004949, 5004990, 5005428, 5005545, 5005724, 5005775, 5005766, 5005811, 5005821, 5005829, 5005847,
//            5005887, 5005905, 5006124, 5006188, 5006321, 5006341, 5006457, 5006484, 5006539, 5006597, 5006605, 5007344,
//            5007513, 5007538, 5007543, 5007615, 5007639, 5007642, 5007742, 5007806, 5007917, 5008033, 5008101, 5008133,
//            5008131, 5008326, 5008318, 5008462, 5008616, 5008625, 5008837, 5008906, 5008889, 5008946, 5009001, 5009051,
//            5009076, 5009554, 5009689, 5009738, 5009808, 5009897, 5010163, 5010318, 5010324, 5010460, 5010537, 5010617,
//            5010728, 5010817, 5011078, 5011117, 5011160, 5011163, 5011422, 5011442, 5011507, 5011651, 5011670, 5011762,
//            5011811, 5011971, 5012099, 5012130, 5012165
//    };

    int logical_calculation(const bool & a, const bool & b)
    {
        if (a)
        {
            return 3;
        }
        else
        {
            if (b)
            {
                return 2;
            }
            else
            {
                return 1;
            }
        }
    }
}

int Mean_V_Info::default_mean_v = 255;

static int frame = 0;

void Mean_V_Info::update(const double &v) {
    sum_v += v;
    sum_num += 1;
}
void Mean_V_Info::get_mean_v()
{
    if (sum_num ==0)
    {
        mean_v = default_mean_v;
    }
    else
    {
        mean_v = sum_v*3.6/sum_num;  //why?
    }
}
void Mean_V_Info::clear()
{
    sum_v = 0;
    sum_num = 0;
    statistice_congestion_count[1] = 0;
    statistice_congestion_count[2] = 0;
    statistice_congestion_count[3] = 0;
    statistice_congestion_count[4] = 0;
}

//template<typename T>
//T SumVector(std::vector<T>& vec)
//{
//    T res = 0;
//    for (size_t i=0; i<vec.size(); i++)
//    {
//        res += vec[i];
//    }
//    return res;
//}


void Mean_Space_Headway_Info::update(std::vector<float>& sh_list) {
//    sum_sh += std::accumulate(sh_list.begin(), sh_list.end(), 0); //yk: accumulate should change
    sum_sh += Mean_Space_Headway_Info::SumVector(sh_list); //yk: accumulate should change
    sum_num += sh_list.size();
}

float Mean_Space_Headway_Info::SumVector(std::vector<float> &vector) {
    float res = 0;
    for (size_t i=0; i<vector.size(); i++)
    {
        res += vector[i];
    }
    return res;
}

//xcb add
Judge_Camera::Judge_Camera(int p1_x, int p1_y, int p2_x, int p2_y)
{
    std::cout<<"Judge_Camera::Judge_Camera dai shi xian"<<std::endl;
}

float Judge_Camera::get_angle(std::vector<int>v1, std::vector<int>v2)
{
    std::cout<<"Judge_Camera::get_angle dai shi xian"<<std::endl;
}
void Judge_Camera::active()
{
    std::cout<<" Judge_Camera::active dai shi xian"<<std::endl;
}
 
bool Judge_Camera::donot_contain_0(int px, int py)
{
    std::cout<<" Judge_Camera::donot_contain_0 dai shi xian"<<std::endl;
}

bool Judge_Camera::contain_0(int px, int py)
{
    std::cout<<" Judge_Camera::contain_0 dai shi xian"<<std::endl;
}

std::string ops::GetLocalTimeWithMs() {
    std::string defaultTime = "19700101000000000";
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    int milli = curTime.tv_usec / 1000;
    char buffer[80] = {0};
    struct tm nowTime;
    localtime_r(&curTime.tv_sec, &nowTime);

    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &nowTime);
    char currentTime[92] = {0};
    snprintf(currentTime, sizeof(currentTime), "%s.%06d", buffer, milli);
    return currentTime;
}
std::string ops::GetLocalTimeWithMs(const int &seconds) {
    std::string defaultTime = "19700101000000000";
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    curTime.tv_sec -= seconds;
    int milli = curTime.tv_usec / 1000;
    char buffer[80] = {0};
    struct tm nowTime;
    localtime_r(&curTime.tv_sec, &nowTime);

    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &nowTime);
    char currentTime[92] = {0};
    snprintf(currentTime, sizeof(currentTime), "%s.%06d", buffer, milli);
    return currentTime;
}

std::string ops::TimeStampToString_sec(long timestamp) {
    long t_sec = timestamp / 1000;
    auto now = static_cast<time_t>(t_sec);
    struct tm *info = localtime(&now);
    std::string year = std::to_string(info->tm_year + 1900);
    std::string month = info->tm_mon >= 9 ? std::to_string(info->tm_mon + 1) : "0" + std::to_string(info->tm_mon + 1);
    std::string day = info->tm_mday >= 10 ? std::to_string(info->tm_mday) : "0" + std::to_string(info->tm_mday);
    std::string hour = info->tm_hour >= 10 ? std::to_string(info->tm_hour) : "0" + std::to_string(info->tm_hour);
    std::string minute = info->tm_min >= 10 ? std::to_string(info->tm_min) : "0" + std::to_string(info->tm_min);
    std::string secs = info->tm_sec >= 10 ? std::to_string(info->tm_sec) : "0" + std::to_string(info->tm_sec);
    std::string res = year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + secs;
    return res;
}
int ops::file_len(const std::string &filename) {
    int len;
    struct stat fileStat;
    stat(filename.c_str(), &fileStat);
    len = fileStat.st_size;
    return len;
}

History_Different_Lane_Info::History_Different_Lane_Info(int lane) : lane(lane){}

History_Continue_Lane_Info::History_Continue_Lane_Info(int lane, std::shared_ptr<Detect_Info> detectInfo, int v_px, int v_py, double v_px_by_angle,
                                                       double v_py_by_angle, double time_now) : lane(lane), detect_info(detectInfo), v_px(v_px),
                                                       v_px_by_angle(v_px_by_angle), v_py_by_angle(v_py_by_angle), time(time_now){}



//xcb modify
Traffic_Flow::Traffic_Flow(const std::string &lane_config_path, const int &pc_num) :
        m_config_path(lane_config_path),
        pc_num(pc_num),
        B3_cnt(0),
        B4_cnt(0),
        normal_lane_str_len(4) {
    LOG(INFO)<<">>> ----------traffic_1";
    get_map_param(lane_config_path);
    LOG(INFO)<<">>> ----------traffic_2";
    std::vector<float> xy;
    forbid_over_lane = {0,0,0,0}; // px_min, px_max, py_min,py_max
    for (auto item : map_param["map_param"]["lane_terminate_xy"].items())
    {
        int i = 1;
        for (auto v : item.value().items())
        {   
            if(i%2)
            {
                xy.push_back(float(v.value()) - 5.0);    
                if (forbid_over_lane[0] == 0){
                    forbid_over_lane[0] = float(v.value()) - 5.0; //px_min
                    forbid_over_lane[1] = float(v.value()) - 5.0; //px_max
                }
                else{
                    if (float(v.value()) - 5.0 < forbid_over_lane[0]){
                        forbid_over_lane[0] = float(v.value()) - 5.0;
                    }
                    else if(float(v.value()) - 5.0 > forbid_over_lane[1]){
                        forbid_over_lane[1] = float(v.value()) - 5.0;
                    }
                }
            }
            else
            {
                xy.push_back(float(v.value()) + 5.0); 
                if (forbid_over_lane[2] == 0){
                    forbid_over_lane[2] = float(v.value()) + 5.0; //py_min
                    forbid_over_lane[3] = float(v.value()) + 5.0; //py_max
                }
                else{
                    if (float(v.value()) + 5.0 < forbid_over_lane[2]){
                        forbid_over_lane[2] = float(v.value()) + 5.0;
                    }
                    else if(float(v.value()) + 5.0 > forbid_over_lane[3]){
                        forbid_over_lane[3] = float(v.value()) + 5.0;
                    }
                }
                
            }
            ++i;
        }
        lane_xy[atoi(item.key().c_str())] = xy;
        // std::cout<<"lane_xy value:"<< xy <<std::endl;
        xy.clear();      
    }
    forbid_over_lane[0] -= 5.0;
    forbid_over_lane[1] += 5.0;
    forbid_over_lane[2] -= 5.0;
    forbid_over_lane[3] += 5.0;
    // std::map<int, std::vector<float>>::iterator pos;
    // for (pos = lane_xy.begin(); pos != lane_xy.end(); pos++)
    // {
    //     for(size_t sz = 0; sz < pos->second.size(); ++sz)
    //     {
    //         std::cout<<"map["<<pos->first<<"]:"<<"vector["<<sz<<"]:"<<pos->second[sz]<<std::endl;
    //     }
    // }

    B5_all = ops::TimeStampToString_sec(long (ops::getTimeStamp())).substr(0, 19);

    LOG(INFO)<<">>> ----------traffic_3";
    generate_lane_origin_info(lane_config_path);
    LOG(INFO)<<">>> ----------traffic_4";
    std::ifstream fin(lane_config_path + "/config_matrix.npy");
    std::ifstream flane(lane_config_path + "/lane_no_lane_name.csv");
    if (fin and flane)
    {
        config_matrix = load_info(lane_config_path + "/config_matrix.npy");
        load_lane_num(lane_config_path + "/lane_no_lane_name.csv");
    } 
    else
    {
        generate_matrix_data(lane_config_path);
    }
    LOG(INFO)<<">>> ----------traffic_5";
    get_traffic_param(lane_config_path);
    LOG(INFO)<<">>> ----------traffic_6";
    //经纬度与m坐标系相互转换
    LOG(INFO)<<"经纬度与m坐标系相互转换dai shi xian"<<std::endl;
    // lon_lat_meter_transform = Lon_Lat_Meter_Transform(param['lon1_lastest'],
    //                                                   param['lat1_lastest'],
    //                                                   param['angle1_lastest'])

    // 拥堵热力图矩阵,1m为一个格子
    congestion_matrix = xt::zeros<uint8_t>(bmp_to_congestion_map({
                                                                    static_cast<int>(config_matrix.shape(0)),
                                                                    static_cast<int>(config_matrix.shape(1))}));
    //抛洒物热力图矩阵
    spills_matrix = xt::zeros<uint8_t>(bmp_to_spills_coordinate({
                                                                    static_cast<int>(config_matrix.shape(0)),
                                                                    static_cast<int>(config_matrix.shape(1))}));

    //交通事故热力图矩阵
    accident_matrix = xt::zeros<uint8_t>(bmp_to_accident_coordinate({
                                                                    static_cast<int>(config_matrix.shape(0)),
                                                                    static_cast<int>(config_matrix.shape(1))}));
    LOG(INFO)<<">>> ----------traffic_7";
    //初始化抛洒物坐标系每个格子下的经纬度
    LOG(INFO)<<"初始化抛洒物坐标系每个格子下的经纬度 dai shi xian"<<std::endl;
    // init_spills_lon_lat(self, lane_config_path)

    //初始化交通事故坐标系每个格子下的经纬度
    LOG(INFO)<<"初始化交通事故坐标系每个格子下的经纬度 dai shi xian"<<std::endl;
    // init_accident_lon_lat(lane_config_path)

    event_cnt = init_event_number(lane_config_path);  //yk:config 文件
    LOG(INFO)<<">>> ----------traffic_8";
    std::cout <<">>> init     event_cnt.\n";
    //yk:wrong in event_cnt=15700  python is 0

    history_events = {};        // 事件保存字典

    get_negative_y_axis_xy();
    format_lane_info(lane_config_path);
    LOG(INFO)<<">>> ----------traffic_9";

    init_total_info();//yi dui qi
    std::cout << ">>> init     total_info\n";
    init_abnormal_event_sign(lane_config_path);
    init_abnormal_event_continue_info();
    
    init_history_abnormal_event();
    // init_lane_fix_info(lane_config_path);
    LOG(INFO)<<">>> ----------traffic_10";


    init_history_signal_light();
    init_judge_camera();

    LOG(INFO)<<">>> cjm----------traffic_11";
    init_for_nearest_out_lane();
    LOG(INFO)<<">>> ----------traffic_11";
    
   
    init_pc_bmp_xy();               // TODO:: fix error.

    /*xcb note
    if (int(param["use_lastest_lonlat"]) == 1){
        rotate_station = cal_trans(0,0,(param["angle1_lastest"] - 360) * M_PI / 180);
    }
    */

    // std::string path = "OutPut/Configs/Alg/CppConfigs/event/use_time_traffic/";
    // if (access(path.c_str(), 0) == -1) {
    //     mkdir(path.c_str(), S_IRWXU);
    // }
    // path += "use_time_traffic.txt";
    // if (access(path.c_str(), 0) == 0) {
    //     if (std::remove(path.c_str()) == 0) {
    //         std::cout << ">>> Delete use_time_traffic.txt. --> Sucess.\n";
    //     } else {
    //         std::cout<< ">>> Delete use_time_traffic.txt. --> Failed.\n";
    //     }
    // }

    // use_time_traffic_fd.open(path, std::ios::out | std::ios::app);
    LOG(INFO)<<">>> ----------traffic_12";
//     xt::xarray<double> mediumpoint = xt::load_npy<double>(lane_config_path + "/centerline_medium.npy");

//     xt::view(mediumpoint, xt::all(), 2) *= 0;
//     for (int i = 0; i < mediumpoint.shape(0); ++i) {
//         points.push_back({mediumpoint(i, 0), mediumpoint(i, 1)});
// //        worinima.push_back({mediumpoint(i, 0), mediumpoint(i, 1)});
//     }
//     pcd_tree = std::make_shared<KDTree>(points);    //change points into the can-use std::complex<double> medium_points[mediumpoint.shape(0)]; //weak refs
    // pcd_tree->nearest_point();
}

/*xcb note 
Traffic_Flow::Traffic_Flow(const std::string &lane_config_path, const int &) :
        m_config_path(lane_config_path),
        (),
        B3_cnt(0),
        B4_cnt(0),
        normal_lane_str_len(4) {
    
    std::cout<<"xcb----get_map_param"<<std::endl;
    get_map_param(lane_config_path);

    config_matrix = load_info(lane_config_path + "/config_matrix.npy");
    std::cout <<">>> init     config_matrix.\n";

    load_lane_num(lane_config_path + "/lane_no_lane_name.csv");
    std::cout <<">>> init     lane_no_lane_name.\n";

    generate_lane_origin_info(lane_config_path);
    std::cout <<">>> init     lane_origin_info.\n";

    get_traffic_param(lane_config_path);
    std::cout <<">>> init     param.\n";



    congestion_matrix = xt::zeros<uint8_t>(bmp_to_congestion_map({
                                                                         static_cast<int>(config_matrix.shape(0)),
                                                                         static_cast<int>(config_matrix.shape(1))}));

    spills_matrix = xt::zeros<uint8_t>(bmp_to_spills_coordinate({
                                                                        static_cast<int>(config_matrix.shape(0)),
                                                                        static_cast<int>(config_matrix.shape(1))}));

    accident_matrix = xt::zeros<uint8_t>(bmp_to_accident_coordinate({
                                                                            static_cast<int>(config_matrix.shape(0)),
                                                                            static_cast<int>(config_matrix.shape(1))}));
    std::cout <<">>> init     congestion_matrix spills_matrix accident_matrix\n";

    B3_cnt = 0;  // 发送给平台的计数器
    B4_cnt = 0;  // 发送给平台的计数器
    event_cnt = init_event_number(lane_config_path);  //yk:config 文件
    std::cout <<">>> init     event_cnt.\n";
    //yk:wrong in event_cnt=15700  python is 0
    history_events = {};        // 事件保存字典
    normal_lane_str_len = 6;    // 1(默认)  3(非左转)  6(左转)

    get_negative_y_axis_xy();
    format_lane_info(lane_config_path);
    init_total_info();
    std::cout << ">>> init     total_info\n";
    init_abnormal_event_sign(lane_config_path);
    init_abnormal_event_continue_info();

    std::cout << ">>> traffic_8\n";
    init_history_abnormal_event();
    init_lane_fix_info(lane_config_path);
    std::cout << ">>> traffic_9\n";
    init_pc_bmp_xy();               // TODO:: fix error.
    std::cout << ">>> traffic_10\n";

    if (int(param["use_lastest_lonlat"]) == 1){
        rotate_station = cal_trans(0,0,(param["angle1_lastest"] - 360) * M_PI / 180);
    }

//    std::string path = "/data/event-alg/use_time_traffic/";
//    if (access(path.c_str(), 0) == -1) {
//        mkdir(path.c_str(), S_IRWXU);
//    }
//    path += "use_time_traffic.txt";
//    if (access(path.c_str(), 0) == 0) {
//        if (std::remove(path.c_str()) == 0) {
//            std::cout << ">>> Delete use_time_traffic.txt. --> Sucess.\n";
//        } else {
//            std::cout<< ">>> Delete use_time_traffic.txt. --> Failed.\n";
//        }
//    }
//
//    use_time_traffic_fd.open(path, std::ios::out | std::ios::app);
    xt::xarray<double> mediumpoint = xt::load_npy<double>(lane_config_path + "/centerline_medium.npy");

    xt::view(mediumpoint, xt::all(), 2) *= 0;
    for (int i = 0; i < mediumpoint.shape(0); ++i) {
        points.push_back({mediumpoint(i, 0), mediumpoint(i, 1)});
//        worinima.push_back({mediumpoint(i, 0), mediumpoint(i, 1)});
    }
    pcd_tree = std::make_shared<KDTree>(points);    //change points into the can-use std::complex<double> medium_points[mediumpoint.shape(0)]; //weak refs
//    pcd_tree->nearest_point
}
*/

int Traffic_Flow::lane_num_switch(int old_lane_num)
{
    int new_lane_num;
    try
    {
        std::map<int, int>lane_num_switch = {};
        for (auto item : map_param["map_param"]["lane_num_switch"].items())
        {
            lane_num_switch[atoi(item.key().c_str())] = item.value();
        }

        if (lane_num_switch.count(old_lane_num) != 0)
        {
            new_lane_num = lane_num_switch[old_lane_num];
        }
        else
        {
            new_lane_num = 0;
        }
        return new_lane_num;
    }
    catch(const std::exception& e)
    {
        new_lane_num = old_lane_num;
        return new_lane_num;
    }

}

int Traffic_Flow::dir_num_switch(int old_dir_num)
{
    int new_dir_num;
    try
    {
        std::map<int, int>dir_num_switch = {};
        for (auto item : map_param["map_param"]["dir_num_switch"].items())
        {
            dir_num_switch[atoi(item.key().c_str())] = item.value();
        }

        if (dir_num_switch.count(old_dir_num) != 0)
        {
            new_dir_num = dir_num_switch[old_dir_num];
        }
        else
        {
            new_dir_num = 0;
        }
        return new_dir_num;
    }
    catch(const std::exception& e)
    {
        new_dir_num = old_dir_num;
        return new_dir_num;
    }
}

xt::xarray<double> Traffic_Flow::cal_trans(double x, double y, double z) {
    xt::xarray<double> R_x = {{1.0, 0.0,         0.0},
                              {0.0, std::cos(x), -1 * std::sin(x)},
                              {0.0, std::sin(x), std::cos(x)}};
    xt::xarray<double> R_y = {{std::cos(y),      0.0, std::sin(y)},
                              {0.0,              1.0, 0.0},
                              {-1 * std::sin(y), 0.0, std::cos(y)}};
    xt::xarray<double> R_z = {{std::cos(z), -1 * std::sin(z), 0.0},
                              {std::sin(z), std::cos(z),      0.0},
                              {0.0,         0.0,              1.0}};
    xt::xarray<double> rotate = xt::linalg::dot(R_z, R_y);
    rotate = xt::linalg::dot(rotate, R_x);
    return rotate;
}

void Traffic_Flow::lonlat_to_xyz_batch(xt::xarray<double> &box, double &lon0, double &lat0, double &angle_north) {
    for (int i = 0; i < box.shape(0); ++i) {
//        if(i == 2541){
//            printf("error");
//        }
        double lon = box(i, 15);
        double lat = box(i, 16);
        // todo:: diff connot be remove!!!
//        std::cout<<std::floor((lon - lon0) * M_PI / 180 * 1e7 + 0.5) / 1e7<<"\n";
        double x =
                std::floor((lon - lon0) * M_PI / 180 * 1e7 + 0.5) / 1e7 * double(R_a) * std::cos(lat0 * M_PI / 180.0);
        double y = std::floor((lat - lat0) * M_PI / 180 * 1e7 + 0.5) / 1e7 * R_b;
        xt::xarray<double> xyz = {{x, y, -4.2}};
//        xt::xarray<double> R_bmp = cal_trans(0, 0, angle_north * M_PI / 180);
        xt::xarray<double> xyz_temp = xt::view(xyz, xt::all(), xt::range(0, 3));
        xyz_temp = xt::transpose(xyz_temp);
        xt::xarray<double> A = xt::linalg::dot(rotate_station, xyz_temp);
        xt::view(xyz, xt::all(), xt::range(0, 3)) = xt::transpose(A);
        box(i, 0) = xyz(0, 0);
        box(i, 1) = xyz(0, 1);
//        std::cout << box(i, 0) << ", " << box(i, 1) << std::endl;
    }
}



xt::xarray<float> Traffic_Flow::load_info(const std::string &config_path) {
    xt::xarray<int8_t> matrix = xt::load_npy<int8_t>(config_path);
    return matrix;
}

void Traffic_Flow::get_map_param(const std::string &lane_config_path)
{
    std::string json_file_path = lane_config_path  + "/map_param.json";  
    std::string name = "map_param";
    std::ifstream ifs(json_file_path, std::ios::in);
    if (!ifs.good())
    {
        std::cout << json_file_path << " No such Json File\n"
                  << std::endl;
    }
    if (!(ifs >> (map_param[name])))
    {
        std::cout << json_file_path << " Error reading Json File\n"
                  << std::endl;
    }
    // std::cout<<"map_param:"<<map_param<<std::endl;
}

void Traffic_Flow::load_lane_num(const std::string &config_path) {
    std::ifstream csv_reader;
    csv_reader.open(config_path);

    std::string line;
    while (getline(csv_reader, line)) {
        ops::Trim(line);
        char s = ',';
        std::string::size_type pos1, pos2;
        pos1 = 0;
        pos2 = line.find(s, 0);
        int key = ops::stringToNum<int>(line.substr(pos1, pos2 - pos1));
        int val = ops::stringToNum<int>(line.substr(pos2 + 1, line.size() - pos2 + 1));
        lane_no_lane_name[key] = val;
    }
    csv_reader.close();
}

//xcb add
void Traffic_Flow::generate_matrix_data(const std::string &lane_config_path)
{
    std::string bmp_path = map_param["map_param"]["bmp_path"];
    std::size_t pos0,pos1;
    pos0= bmp_path.find_first_not_of("\"");
    pos1 = bmp_path.find_last_not_of("\"");
    cv::Mat img = cv::imread(lane_config_path + "/" + bmp_path.substr(pos0, pos1+1));
    int w = img.cols;
    int h = img.rows;
    int c = img.channels();
    xt::xarray<int8_t> matrix = xt::zeros<int8_t>({w, h, 12})-1;
    std::cout<<"img size:"<<img.size()<<std::endl;
    std::ifstream csv_reader;
    std::string traffic_csv_path = map_param["map_param"]["Traffic_csv_path"];
    pos0 = traffic_csv_path.find_first_not_of("\"");
    pos1 = traffic_csv_path.find_last_not_of("\"");
    std::string line;
    csv_reader.open(lane_config_path + "/" + traffic_csv_path.substr(pos0, pos1+1));
    while (getline(csv_reader, line))
    {
        ops::Trim(line);
        std::vector<float> key = ops::vStrSplit<float>(line, ',');
        // std::cout<<"key:"<<key<<std:endl;
        int x = int(key[0]);
        // std::cout<<"x:"<<x<<std::endl;
        int y = int(key[1]);
        // std::cout<<"y:"<<x<<std::endl;
        
        int lane_name = int(key[2]);
        std::string str_lane_name = std::to_string(lane_name);
        if(str_lane_name.length() == 5)
        {
            lane_name = atoi(str_lane_name.substr(1).c_str());
        }

        if(lane_fix_info.find(lane_name) != lane_fix_info.end())
        {
            matrix(x,y,Config_Matrix_Index::LANE_NO) = int(lane_fix_info[lane_name]->lane_output_name);
        }
        matrix(x, y, Config_Matrix_Index::CALC_FLOW) = int(key[3]);
        matrix(x, y, Config_Matrix_Index::FORBID_CROSS_LINE) = int(key[4]);
        matrix(x, y, Config_Matrix_Index::EMERGENCY_AREA) = int(key[5]);
        matrix(x, y, Config_Matrix_Index::NO_PARK) = int(key[6]);
        matrix(x, y, Config_Matrix_Index::CENTER_AREA) = int(key[7]);
        matrix(x, y, Config_Matrix_Index::SIDEWALK) = int(key[8]);
        matrix(x, y, Config_Matrix_Index::TURN_LEFT_1) = int(key[5]);
        matrix(x, y, Config_Matrix_Index::TURN_LEFT_2) = int(key[5]);
        matrix(x, y, Config_Matrix_Index::TURN_LEFT_3) = int(key[5]);
        matrix(x, y, Config_Matrix_Index::SECTION_ID) = int(key[5]);
    }
    csv_reader.close();

    std::ifstream forbid(lane_config_path + "/Forbid_Cross_Line_Area_Traffic.csv");
    if(forbid)
    {
        csv_reader.open(lane_config_path + "/Forbid_Cross_Line_Area_Traffic.csv");
        while (getline(csv_reader, line))
        {
            ops::Trim(line);
            std::vector<float> key = ops::vStrSplit<float>(line, ',');
            int x = int(key[0]);
            int y = int(key[1]);
            matrix(x, y, Config_Matrix_Index::FORBID_CROSS_LINE) = int(key[4]);
        }
        
    }
    csv_reader.close();

    csv_reader.open(lane_config_path + "/Crosswalk_Traffic.csv");
    while (getline(csv_reader, line))
    {
        ops::Trim(line);
        std::vector<float> key = ops::vStrSplit<float>(line, ',');
        int x = int(key[0]);
        int y = int(key[1]);
        matrix(x, y, Config_Matrix_Index::CENTER_AREA) = int(key[7]);
        matrix(x, y, Config_Matrix_Index::SIDEWALK) = int(key[8]);
    }
    csv_reader.close();

    std::ifstream pedestrian(lane_config_path + "/Pedestrian_Waiting_Area_Traffic.csv");
    if(pedestrian)
    {
        csv_reader.open(lane_config_path + "/Pedestrian_Waiting_Area_Traffic.csv");
        while (getline(csv_reader, line))
        {
            ops::Trim(line);
            std::vector<float> key = ops::vStrSplit<float>(line, ',');
            int x = int(key[0]);
            int y = int(key[1]);
            matrix(x, y, Config_Matrix_Index::EMERGENCY_AREA) = int(key[8]);
        }  
    }
    csv_reader.close();

    std::ifstream left_turn_waiting(lane_config_path + "/Left_Turn_Waiting_Area_Traffic.csv");
    if(left_turn_waiting)
    {
        csv_reader.open(lane_config_path + "/Left_Turn_Waiting_Area_Traffic.csv");
        while (getline(csv_reader, line))
        {
            ops::Trim(line);
            std::vector<float> key = ops::vStrSplit<float>(line, ',');
            int x = int(key[0]);
            int y = int(key[1]);
            matrix(x, y, Config_Matrix_Index::CENTER_AREA) = int(key[7]) + 10;
        }  
    }
    config_matrix = matrix;
    csv_reader.close();

    //保存模拟得到的配置文件
    save_config_matrix_info(lane_config_path + "/config_matrix.npy", config_matrix);
    save_lane_no_lane_name_info(lane_config_path + "/lane_no_lane_name.csv", lane_no_lane_name);
}

//xcb add
void Traffic_Flow::generate_lane_origin_info(const std::string &lane_config_path) {
    lane_origin_info.clear();
    lane_no_lane_name.clear();
    lane_fix_info.clear();
    lane_name_centerline_mediumpoint.clear();
    lane_name_centerline_pcd_tree.clear();
    lane_match_pair.clear();

    std::vector<int> branch;
    for(auto item : map_param["map_param"]["lane_branches"].items())
    {
        for(auto br : item.value().items())
        {
            branch.push_back(int(br.value()));
        }
        lane_branches[atoi(item.key().c_str())] = branch;
        branch.clear();
    }
    // std::map<int, std::vector<int>>::iterator pos;
    // for (pos = lane_branches.begin(); pos != lane_branches.end(); pos++)
    // {
    //     for(size_t sz = 0; sz < pos->second.size(); ++sz)
    //     {
    //         std::cout<<"map["<<pos->first<<"]:"<<"vector["<<sz<<"]:"<<pos->second[sz]<<std::endl;
    //     }
    // }

    std::ifstream csv_reader;
    std::string line_csv_path;
    line_csv_path = map_param["map_param"]["Line_csv_path"];
    std::size_t pos0,pos1;
    pos0=line_csv_path.find_first_not_of("\"");
    pos1 = line_csv_path.find_last_not_of('\"');
    // std::cout<<"map_param[map_param][Line_csv_path]:"<<line_csv_path.substr(pos0, pos1)<<std::endl;
    csv_reader.open(lane_config_path + "/" + line_csv_path.substr(pos0, pos1+1));
    std::cout<<"xcb---line_csv_path:"<<lane_config_path + "/" + line_csv_path.substr(pos0, pos1+1)<<std::endl;

    std::string line;
    while (getline(csv_reader, line)) 
    {
        // std::cout<<"xcb----line:"<<line<<std::endl;
        ops::Trim(line);
        std::vector<double> key = ops::vStrSplit<double>(line, ',');

        int lane_name = std::to_string(int(key[0])).length() == 5 ? atoi(std::to_string(int(key[0])).substr(1).c_str()) : int(key[0]);
        int lane_no = int(key[1]);
        int lane_direction = int(key[2]);
        int lane_class = int(key[3]);
        int is_non_motorway = int(key[4]);
        int is_bus_lane = int(key[5]);
        int is_emergency_lane = int(key[6]);
        double min_V = double(key[7]);
        double max_v = double(key[8]);
        double length = double(key[9]);
        double width = double(key[10]);
        int start_px = int(key[11]);
        int start_py = int(key[12]);
        int end_px = int(key[13]);
        int end_py = int(key[14]);

        lane_no_lane_name[lane_no] = lane_name;
        lane_fix_info[lane_name] = std::make_shared<Lane_Fix_Info>(double(lane_direction), double(lane_no));
        key.erase(key.begin()+1);
        lane_origin_info.push_back(key);

        if (std::to_string(lane_name).length() == normal_lane_str_len)
        {
            try
            {
                std::string strname = std::to_string(key[0]);
                std::string title_lane = strname.substr(0, 1);

                xt::xarray<double> mediumpoint = xt::load_npy<double>(lane_config_path + "/" + strname.substr(0,2) + ".npy");
                int title_lane_int = atoi(title_lane.c_str());
                int in_or_out = atoi(strname.substr(1,1).c_str());
                if (in_or_out == 1)
                {
                    lane_branches[title_lane_int][3] += 1;
                }
                else
                {
                    lane_branches[title_lane_int][4] += 1;
                }
                xt::view(mediumpoint, xt::all(), 2) *= 0;
                for (int i = 0; i < mediumpoint.shape(0); ++i) {
                    points.push_back({mediumpoint(i, 0), mediumpoint(i, 1)});
                }
                pcd_tree = std::make_shared<KDTree>(points);    //change points into the can-use std::complex<double> medium_points[mediumpoint.shape(0)]; //weak refs
                lane_name_centerline_pcd_tree[lane_name] = pcd_tree;
                lane_name_centerline_mediumpoint[lane_name] = mediumpoint;
                points.clear();
            }
            catch(const std::exception& e)
            {
                std::cerr << "find lane"<< lane_name <<"centerline error:"<<e.what() << '\n';
            }
            
        }

        // open3d dai shixian
    } 
    // LOG(INFO)<<"open3d dai shi xian"<<std::endl;
    LOG(INFO)<<"generate_lane_origin_info success !"<<std::endl;
    csv_reader.close();
}

void Traffic_Flow::get_traffic_param(const std::string &config_path) {
    std::ifstream csv_reader;
    csv_reader.open(config_path + "/traffic_param.csv");
    int k = 0;
    std::string line;
    char s = ',';
    while (getline(csv_reader, line)) {
        if (!k) {
            k++;
            continue;
        } else {}
        ops::Trim(line);

        std::string::size_type pos1, pos2;
        pos1 = 0;
        pos2 = line.find(s, pos1);
        std::string key = line.substr(pos1, pos2 - pos1);
        pos1 = pos2 + 1;
        pos2 = line.find(s, pos1);
        double val = ops::stringToNum<double>(line.substr(pos1, pos2 - pos1));
        param[key] = val;
    }
    csv_reader.close();
}
//xcb add
void Traffic_Flow::save_config_matrix_info(const std::string &config_path, xt::xarray<int8_t> matrix)
{
    xt::dump_npy(config_path, matrix);
}

//xcb add
void Traffic_Flow::save_lane_no_lane_name_info(const std::string &config_path, std::map<int, int>lane_no_name)
{
    ofstream csvfile;
    csvfile.open(config_path, ios::out);
    for (auto it : lane_no_name)
    {
        csvfile << it.first<<","<<it.second<<endl;
    }
    csvfile.close();
}

xt::xarray<int> Traffic_Flow::bmp_to_congestion_map(const xt::xarray<int> &bmp_xy) {
    return bmp_xy / int(param["bmp_ratio"] * param["meter_to_pixel"] * param["congestion_meter"]);
}

xt::xarray<int> Traffic_Flow::bmp_to_spills_coordinate(const xt::xarray<int> &bmp_xy) {
    return bmp_xy / int(param["bmp_ratio"] * param["meter_to_pixel"] / param["spills_meter"]);
}

xt::xarray<int> Traffic_Flow::bmp_to_accident_coordinate(const xt::xarray<int> &bmp_xy) {
    return bmp_xy / int(param["bmp_ratio"] * param["meter_to_pixel"] / param["accident_meter"]);
}

int Traffic_Flow::init_event_number(const std::string &lane_config_path) {
    int event_no;

    std::string event_num_file = lane_config_path + "/event_number.npy";
    if (access(event_num_file.c_str(), 0) == -1) {
        event_no = 0;
    } else {
        event_no = xt::load_npy<int>(event_num_file)(0) + 1000;
    }
    return event_no;
}

void Traffic_Flow::get_negative_y_axis_xy() {
    xt::xarray<int> origin_xy = lidar_to_bmp<int>({{0, 0}}, 0, 1);
    std::cout << "xcb-------------------> negative_y_axis_xy origin_xy: " << origin_xy << std::endl;
    xt::xarray<int> negative_xy = lidar_to_bmp<int>({{0, -1}}, 0, 1);
    std::cout << "xcb-------------------> negative_y_axis_xy negative_xy: " << negative_xy << std::endl;
    negative_y_axis_xy = negative_xy - origin_xy;
    std::cout << "xcb-------------------> negative_y_axis_xy 00: " << negative_y_axis_xy << std::endl;
    negative_y_axis_xy = negative_y_axis_xy.reshape({negative_y_axis_xy.size()});
    std::cout << "xcb-------------------> negative_y_axis_xy  11: " << negative_y_axis_xy << std::endl;
}
//xcb add 配置文件中读取的信息获取更多有用的信息
void Traffic_Flow::format_lane_info(const std::string &lane_config_path) {
    xt::xarray<int> Congestion_Level_highway_table = ops::csvReader<int>(
            lane_config_path + "/Congestion_Level_highway.csv");

    xt::xarray<int> Congestion_Level_main_branch_table = ops::csvReader<int>(
            lane_config_path + "/Congestion_Level_main_branch.csv");


    for (auto &origin_data: lane_origin_info) {
        int lane_name = int(origin_data[0]);
        std::string str_lane_name = std::to_string(lane_name);
        if (str_lane_name.size() == normal_lane_str_len) {   // 每一个车道信息，包括限速等
            lane_info[lane_name] = std::make_shared<Lane_Info>();
            lane_info[lane_name]->fill_data(origin_data, Congestion_Level_highway_table,
                                            Congestion_Level_main_branch_table);
            // lane_info[lane_name]->fill_plus_data_for_non_converge(origin_data, negative_y_axis_xy,
            //                                                       int(param["use_lastest_lonlat"]) == 0
            //                                                       ? param["angle1_bmp"] : param["angle1_lastest"]);

            lane_info[lane_name]->fill_plus_data_for_non_converge(origin_data, negative_y_axis_xy,
                                                                  param["angle1_lastest"]);
        } else if (str_lane_name.size() == 2 * normal_lane_str_len) {
            lane_info[lane_name] = std::make_shared<Lane_Info>();
            lane_info[lane_name]->fill_data(origin_data, Congestion_Level_highway_table,
                                            Congestion_Level_main_branch_table);
            int start_lane_name;
            int end_lane_name;
            std::size_t pos0 = 0;
            std::size_t pos1 = normal_lane_str_len;
            start_lane_name = atoi(str_lane_name.substr(pos0, normal_lane_str_len).c_str());
            end_lane_name = atoi(str_lane_name.substr(pos1).c_str());
            lane_info[lane_name]->fill_plus_data_for_converge(lane_info[start_lane_name],
                                                              lane_info[end_lane_name]);
            turn_left_info[lane_name] = lane_info[lane_name];
        }
    }
    /*
    Lane_Direction:
    STRAIGHT = 0
    LEFT = 1
    RIGHT = 2
    STRAIGHT_LEFT = 3
    STRAIGHT_RIGHT = 4
    STRAIGHT_LEFT_RIGHT = 5
    UNKOWN = 6
    */
    std::vector<int> lane_info_keys;
    for (decltype(lane_info)::const_iterator it = lane_info.cbegin(); it != lane_info.cend(); ++it)
    {
        lane_info_keys.push_back(it->first);
    }

    for (decltype(lane_info)::const_iterator it = lane_info.cbegin(); it != lane_info.cend(); ++it)
    {
        lane_match_pair[it->first] = {};
        std::string lane_name = std::to_string(it->first);
        std::string in_or_out = lane_name.substr(1, 1);
        if (in_or_out.compare("1") == 0)
        {
            int title_lane = atoi(lane_name.substr(0,1).c_str());
            std::vector<int>match_info = lane_branches[title_lane];
            int aim_lane_straight = match_info[0];
            int aim_lane_left = match_info[1];
            int aim_lane_right = match_info[2];
            if (int(it->second->lane_direction) == laneDirection->STRAIGHT)
            {
                for(decltype(lane_info_keys)::const_iterator lane = lane_info_keys.cbegin(); lane != lane_info_keys.cend(); ++lane)
                {
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_straight and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                }
            }
            else if (int(it->second->lane_direction) == laneDirection->LEFT)
            {
                for(decltype(lane_info_keys)::const_iterator lane = lane_info_keys.cbegin(); lane != lane_info_keys.cend(); ++lane)
                {
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_left and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                }
            }
            else if(int(it->second->lane_direction) == laneDirection->RIGHT)
            {
                for(decltype(lane_info_keys)::const_iterator lane = lane_info_keys.cbegin(); lane != lane_info_keys.cend(); ++lane)
                {
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_right and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                }
            }
            else if(int(it->second->lane_direction) == laneDirection->STRAIGHT_LEFT)
            {
                for(decltype(lane_info_keys)::const_iterator lane = lane_info_keys.cbegin(); lane != lane_info_keys.cend(); ++lane)
                {
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_straight and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_left and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                }
            }
            else if(int(it->second->lane_direction) == laneDirection->STRAIGHT_RIGHT)
            {
                for(decltype(lane_info_keys)::const_iterator lane = lane_info_keys.cbegin(); lane != lane_info_keys.cend(); ++lane)
                {
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_straight and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_right and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                }
            }
            else if(int(it->second->lane_direction) == laneDirection->STRAIGHT_LEFT_RIGHT)
            {
                for(decltype(lane_info_keys)::const_iterator lane = lane_info_keys.cbegin(); lane != lane_info_keys.cend(); ++lane)
                {
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_straight and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_left and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                    if (atoi(std::to_string(*lane).substr(0,1).c_str()) == aim_lane_right and 
                        atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                    {
                        lane_match_pair[it->first].push_back(*lane);
                    }
                }
            }
            else
            {
                continue;
            }
        }
    }
    for (auto item : map_param["map_param"]["lane_u_turn"].items())
    {
        if (int(item.value()) == 1)
        {
            std::vector<int>out_lane = {};
            // int direction = atoi(item.key().c_str());
            int direction = static_cast<int>(item.key()[0] - '0');
            for(decltype(lane_info_keys)::const_iterator lane = lane_info_keys.cbegin(); lane != lane_info_keys.cend(); ++lane)
            {
                if (int((*lane) / 1000) == direction and atoi(std::to_string(*lane).substr(1,1).c_str()) == 0)
                {
                    out_lane.push_back(*lane);
                }
            }
            lane_match_pair[atoi(item.key().c_str())].insert(lane_match_pair[atoi(item.key().c_str())].end(), out_lane.begin(), out_lane.end());
        }
    }
}
//初始化时间段里的统计量
void Traffic_Flow::init_total_info() {
//    Detect_Class_Type classType;
    total_statistics.clear();  //# 当前时间段的统计信息:车流量、时间占有率
    history_lane_id_time_info.clear();  //# 当前车道累计的车辆出现时间信息
    history_lane_id_info_of_section.clear(); //当前车道统计区累计出现的id
    history_lane_motor_id_with_class_id.clear(); //# hhz 1110 当前车道统计区车辆id第一次出现的类型
    history_lane_id_order_of_section.clear();  //# 目标进入车道统计区的先后顺序
    last_id_info_0.clear();  //# 保存上一阵的id对应信息，用于计算bmp下的速度
    last_id_info_1.clear();  //# 保存上一阵的id对应信息，用于计算bmp下的速度
    id_count.clear();  //# 记录当前id出现次数
    id_history_lane.clear(); //#记录id对应的历史所属不同车道
    id_trajectory_eval.clear();  //# 记录id对应的之前X时刻车道  {id:[最新x帧车道]}
//    self.id_states_of_red_light_for_motor = {}  # 记录id对应的刚进交汇区的位置 和 当前位置 与刚进十字位置的角度和方向
    id_states_of_red_light_for_people.clear();  //记录id对应的行人状态  -1:不在人行道  其他:人行道id号
    history_speed_info.clear();  //# 记录id的历史速度信息
    for (const auto &lane: lane_info) {
        history_lane_id_time_info[lane.first] = {};
        history_lane_id_info_of_section[lane.first] = {};
        history_lane_motor_id_with_class_id[lane.first] = {};
        history_lane_id_order_of_section[lane.first] = {};
        for (auto class_id: classType->all) {
            history_lane_id_info_of_section[lane.first][class_id] = {};
        }
        total_statistics[lane.first] = {};
        total_statistics[lane.first]["lane_flow"] = {};
        for (auto class_id: classType->all) {
            total_statistics[lane.first]["lane_flow"][class_id] = 0;
        }
        total_statistics[lane.first]["time_occupancy"][0] = 0;      // 此处有别于python，因为python程序在倒数第二层非法变换类型
        total_statistics[lane.first]["follow_car_percent"][0] = 0;  // 此处有别于python，因为python程序在倒数第二层非法变换类型
    }

    lane_mean_v_info_by_all_v.clear(); //每条车道的mean_v_info
    lane_mean_v_info_by_section.clear(); //通过断面求每条车道上的mean_v_info
    lane_local_static_congestion_state.clear(); //每条车道的  拥堵状态
    lane_mean_space_headway_info.clear(); //每条车道的sh信息

    for (const auto &lane: lane_info) {
        lane_mean_v_info_by_all_v[lane.first] = {};
        for (auto class_id: classType->all) {
            lane_mean_v_info_by_all_v[lane.first]["null"][class_id] = std::make_shared<Mean_V_Info>(
                    Mean_V_Info::default_mean_v);
        }
        lane_mean_v_info_by_all_v[lane.first]["all"][0] = std::make_shared<Mean_V_Info>(Mean_V_Info::default_mean_v);
        lane_mean_v_info_by_all_v[lane.first]["pc_id"] = {};
        for (int pc_id = 1; pc_id <  pc_num + 1; ++pc_id) {
            lane_mean_v_info_by_all_v[lane.first]["pc_id"][pc_id] = std::make_shared<Mean_V_Info>(
                    Mean_V_Info::default_mean_v);
        }
        for (int i = 0; i < congestion_matrix.shape(0); ++i) {
            lane_mean_v_info_by_all_v[lane.first]["meter"][i] = std::make_shared<Mean_V_Info>(Mean_V_Info::default_mean_v);
        }

        lane_mean_v_info_by_section[lane.first] = {};
        for (const auto &class_id: classType->all) {
            lane_mean_v_info_by_section[lane.first][std::to_string(class_id)] = std::make_shared<Mean_V_Info>(
                    Mean_V_Info::default_mean_v);
        }
        lane_mean_v_info_by_section[lane.first]["all"] = std::make_shared<Mean_V_Info>(Mean_V_Info::default_mean_v);

        lane_local_static_congestion_state[lane.first] = {};
        lane_local_static_congestion_state[lane.first]["all"][0] = Congestion_Level::unblocked;
        for (int pc_id = 1; pc_id <  pc_num + 1; ++pc_id) {
            lane_local_static_congestion_state[lane.first]["null"][pc_id] = Congestion_Level::unblocked;
        }
        lane_local_static_congestion_state[lane.first]["meter"] = {};
        for (int i = 0; i < congestion_matrix.shape(0); ++i) {
            lane_local_static_congestion_state[lane.first]["meter"][i] = Congestion_Level::unblocked;
        }
        lane_mean_space_headway_info[lane.first] = std::make_shared<Mean_Space_Headway_Info>(1000.0);
    }
}
//读取检测事件的标志
void Traffic_Flow::init_abnormal_event_sign(const std::string &lane_config_path) {
    std::ifstream csv_reader;
    csv_reader.open(lane_config_path + "/traffic_event.csv");

    int k = 0;
    std::string line;
    char s = ',';
    while (getline(csv_reader, line)) {
        if (!k) {
            k++;
            continue;
        } else {}
        ops::Trim(line);

        std::string::size_type pos1, pos2;
        pos1 = 0;
        pos2 = line.find(s, pos1);
        std::string key = line.substr(pos1, pos2 - pos1);
        pos1 = pos2 + 1;
        pos2 = line.find(s, pos1);
        bool val = ops::stringToNum<int>(line.substr(pos1, pos2 - pos1 + 1));
        event_sign[key] = val;
    }
    csv_reader.close();
}
//初始化异常事件的连续出现
void Traffic_Flow::init_abnormal_event_continue_info() {
    id_continue_occupy_dedicated_lane_info.clear(); //# 记录当前id连续占用其他车道的信息,包括机动车占用非机动车道,非机动车占用机动车道
    id_continue_people_occupy_motor_lane_info.clear();//记录当前行人连续占用机动车道的信息
    id_continue_non_motor_occupy_motor_lane_info.clear();  //非机动车在机动车道逗留
    id_continue_occupy_emergency_lane_info.clear();//记录目标连续占用紧急车道的信息，占用紧急车道
    id_continue_retrograde_info_for_motor.clear();//记录目标连续逆行的信息,机动车逆行
    id_continue_retrograde_info_for_non_motor.clear();//记录目标连续逆行的信息,非机动车逆行
    id_continue_speeding_info_for_motor.clear();//记录当前id连续超速的信息,机动车超速
    id_continue_speeding_info_for_non_motor.clear();//记录当前id连续超速的信息,非机动车超速
    id_continue_stroll_info.clear();//记录当前id连续慢行的信息,机动车慢行
    id_continue_cross_line_info.clear();//记录当前id连续压线的信息,机动车压线
    id_continue_stop_info.clear();//记录当前机动车id连续停的信息,机动车违停
    id_continue_cross_lane_info_for_people.clear();//记录当前id连续横穿马路的信息,行人横穿马路
    id_continue_cross_lane_info_for_motor.clear();//记录当前id连续横穿马路的信息,机动车横穿马路
    id_continue_cross_lane_info_for_non_motor.clear();//记录当前id连续横穿马路的信息,非机动车横穿马路
    id_continue_run_the_red_light_info_for_motor.clear();//记录当前id连续闯红灯的信息,包括机动车闯红灯,机动车闯红灯
    id_continue_run_the_red_light_info_for_people.clear();//记录当前id连续闯红灯的信息,包括机动车闯红灯,行人闯红灯
    id_continue_occupy_bus_lane_info.clear();//记录当前id连续占用公交车道的信息
    id_continue_change_lanes_info.clear();//记录当前id连续变道的信息
    spills_coordinate_continue_info.clear();//记录当前位置连续检测出抛洒物的信息
    accident_coordinate_continue_info.clear();//记录当前位置连续检测出交通事故的信息
    roadwork_coordinate_continue_info.clear();  // 记录当前位置连续检测出道路施工的信息
    id_continue_person_not_walk_zebra_stripe.clear();  // 记录当前id的行人连续未走斑马线
    id_continue_stop_in_zebra_stripe.clear();  //记录当前车辆占用斑马线
    id_continue_not_following_lane_guide.clear();  //记录当前车辆不按车道导向行驶
    id_continue_stop_over_terminateline.clear();   //记录当前非機動越过停止线听停车
    id_continue_stop_over_terminateline_for_motor.clear();   //记录当前機動越过停止线听停车
    congestion_continue_info.clear();
}
//初始化历史异常事件
void Traffic_Flow::init_history_abnormal_event() {
    confirm_run_the_red_light_info = {};
    confirm_not_following_info = {};
    confirm_not_following_info_cur_time = {}; // xcb add
    confirm_not_following_info_lane = {}; // xcb add
    history_occupy_dedicated_lane = {};  //# 占用专用车道
    history_people_occupy_motor_lane = {};  //# 行人在机动车道逗留
    history_non_motor_occupy_motor_lane = {};  //非机动车在机动车道逗留
    history_retrograde_for_motor = {};  //# 机动车逆行
    history_retrograde_for_non_motor = {};  //# 非机动车逆行
    history_speeding_for_motor = {};  //# 机动车超速
    history_speeding_for_non_motor = {};  // # 非机动车超速
    history_stroll = {};  //# 慢行
    history_cross_line = {};  //# 压线
    history_illegal_stop = {};  //# 违停
    history_congestion = {};
    history_cross_lane_for_non_motor = {};  //# 非机动车横穿马路
    history_cross_lane_for_people = {};  //# 行人横穿马路
    history_cross_lane_for_motor = {};  //# 机动车横穿马路
    history_run_the_red_light_for_motor = {};  //# 机动车闯红灯
    history_run_the_red_light_for_people = {};  //# 行人闯红灯
    history_occupy_bus_lane = {};  //# 占用公交车道
    history_change_lanes = {};  //# 变道
    history_spills = {};  //# 抛洒物
    history_accident = {};  //# 交通事故
    history_roadwork = {};  // 道路施工
    history_person_not_walk_zebra_stripe = {};  // 行人未走斑马线
    history_occupy_emergency_lane = {};
    history_not_following_lane_guide = {}; //不按车道线行驶
    history_car_in_zebra_stripe = {};
    history_stop_over_terminateline = {};  //history stop over terminateline 1120 非机动车越线停车
    history_stop_over_terminateline_for_motor = {};
    id_dist_list.clear();
    history_all_event_output_for_B5.clear();
    history_all_event_output_for_B5_all.clear(); // cjm 0824
//    # # zhushi
    save_b4.clear();
//    self.event_save_by_my_own = {}
//    for i in range(1, 23):
//        self.event_save_by_my_own[i] = {}
}
//xcb add
void Traffic_Flow::init_history_signal_light()
{
    history_pedstrian_waiting_area.clear();
    std::vector<int>pedstrian_waiting_area = {1,2,3,4};
    std::map<int, std::vector<double>> info;
    for(int i; i < pedstrian_waiting_area.size(); ++i)
    {
        history_pedstrian_waiting_area[i] = info;
    }
    history_car_waiting_area.clear();
    for (auto item : map_param["map_param"]["lane_terminate_xy"].items())
    {
        int car_waiting_area_num =  atoi(item.key().c_str());
        history_car_waiting_area[car_waiting_area_num] = info;
    }
}
//xcb add
void Traffic_Flow::init_judge_camera()
{
    judge_camera_lists.push_back(std::make_shared<Judge_Camera>(1110, 971, 1025, 1176));
    judge_camera_lists.push_back(std::make_shared<Judge_Camera>(868, 1127, 1094, 1069));
    judge_camera_lists.push_back(std::make_shared<Judge_Camera>(1000, 900, 1060, 1019));
}
//xcb add
void Traffic_Flow::init_for_nearest_out_lane()
{
    std::string lane;
    // std::vector<int>start_loc;
    // std::vector<std::vector<int>>pxy_out_lane;
    std::vector<int>pxy_out_lane;
    for(auto it=lane_info.begin(); it!=lane_info.end(); it++)
    {
        lane = to_string(it->first);
        if(lane.length() != normal_lane_str_len){
            continue;
        }
        out_lane.push_back(it->first);
        // start_loc = {it->second->start_px, it->second->start_py};
        pxy_out_lane.push_back(it->second->start_px);
        pxy_out_lane.push_back(it->second->start_py);
        // start_loc.clear();
    }
    std::vector<std::size_t>shape = {std::size_t(pxy_out_lane.size()/2), 2};
    pxy_of_out_lane = xt::adapt(pxy_out_lane, shape);
}

//车道对应航向角、输出车道号
void Traffic_Flow::init_lane_fix_info(const std::string &lane_config_path) {
    std::ifstream csv_reader;
    csv_reader.open(lane_config_path + "/traffic_lane_fix.csv");
    int k = 0;
    std::string line;
    while (getline(csv_reader, line)) {
        if (!k) {
            k++;
            continue;
        } else {}
        ops::Trim(line);
        std::vector<int> res = ops::vStrSplit<int>(line, ',');
        lane_fix_info[res[0]] = std::make_shared<Lane_Fix_Info>(res[1], res[2]);
        std::cout << "xcb--------> " << "res[0]: " << res[0] << "  res[1]: " << res[1] << "  res[2]: " << res[2] << std::endl;
    }
    csv_reader.close();
}
//获取各基站在车道图上的像素坐标
void Traffic_Flow::init_pc_bmp_xy() {
    std::string suffix = "lastest";
    for (int i = 1; i <  pc_num + 1; ++i) {
        // std::cout << "xcb------------> : "  <<  <<  std::endl;
        base_station_pos[i] = {param["lon" + std::to_string(i) + "_" + suffix],
                               param["lat" + std::to_string(i) + "_" + suffix],
                               param["angle" + std::to_string(i) + "_" + suffix],
                               param["angle" + std::to_string(i) + "_" + suffix] -
                               param["angle" + std::to_string(1) + "_" + suffix]};
    }
}

void Traffic_Flow::use(xt::xarray<double> &input_info, const bool &statistics_flag, xt::xarray<double> &boxes_final, int pc_id) {
//    use_time_traffic_fd.close();
//    std::string record_txt = "/data/event-alg/use_time_traffic/use_time_traffic.txt";
//    double fs = ops::file_len(record_txt) / 1024 / 1024;
//    if (fs > 5000.0) {
//        if (std::remove(record_txt.c_str()) == 0) {
//            std::cout<< ">>> Use_time_traffic.txt is too large. Delete --> Sucess.\n";
//        } else {
//            std::cout<< ">>> Use_time_traffic.txt is too large. Delete --> Failed.\n";
//        }
//    }
//    use_time_traffic_fd.open(record_txt, std::ios::app | std::ios::out);
    // std::string dataTimeNow = ops::GetLocalTimeWithMs();
    if (event_sign["save_input_info"])
    {
        std::string path = m_config_path + "/event_source/";
        if (access(path.c_str(), 0) == -1) {
            mkdir(path.c_str(), S_IRWXU);
        }
        xt::dump_npy(path + std::to_string(frame) + ".npy", input_info); // cjm 0911
        ++frame;
        // xt::dump_npy(path + dataTimeNow + ".npy", input_info);
    }
    int boxnum = 0;
    if (input_info.shape(0) == 0) {
        cur_time = ops::getTimeStamp();
        boxnum = 0;
    } else {
        cur_time = input_info(0,44); // cjm 0830 must sure timestamp unit, lixianceshi with python
        cur_frame_id = input_info(0,45);
        boxnum =input_info.shape(0);
        if (input_info.shape(1)<53){
            int colum = 53 - input_info.shape(1);
            xt::xarray<double> temp = xt::zeros<double>({int(input_info.shape(0)), colum});
            input_info = xt::concatenate(xt::xtuple(input_info, temp), 1);
        }

        /** 1 获取计算统计量和异常事件检测的预备信息 */
        long t_now = ops::getTimeStamp();
        get_pre_info(input_info, boxes_final);
        long t_1 = ops::getTimeStamp();

        /** 2 计算瞬时量 */  //b3
        t_now = ops::getTimeStamp();
        get_instantaneous_info();
        t_1 = ops::getTimeStamp();
        std::cout << "pc_id: "<< pc_id <<", get_instantaneous_info" << std::endl;

        /** 3 计算统计量 */  //b4
        // t_now = ops::getTimeStamp();
        // get_statistics_info(statistics_flag, boxes_final); //yk:9.9-17.41 no wrong
        // t_1 = ops::getTimeStamp();
        std::cout << "pc_id: "<< pc_id <<", get_statistics_info" << std::endl;

        /**""" 4 异常事件检测"""*/ //b5
        t_now = ops::getTimeStamp();
        detect_abnormal_event(statistics_flag);
        t_1 = ops::getTimeStamp();
        std::cout << "pc_id: "<< pc_id <<", detect_abnormal_event" << std::endl;
    }
}
//初始化当前帧信息
void Traffic_Flow::init_cur_info() {
    cur_id_info.clear();
    cur_lane_id.clear();
    cur_id_lane.clear();

    cur_lane_sorted_id.clear();
    cur_lane_id_for_flow.clear();

    cur_statistics.clear();
    cur_lane_classes_num.clear();

    cur_lane_mean_v.clear();
    cur_lane_pc_car_num.clear();


//    self.cur_lane_ids_box = {}
//    self.cur_lane_box_info = {}
//    std::shared_ptr<Detect_Class_Type> classType = std::make_shared<Detect_Class_Type>();
    for (decltype(lane_info)::const_iterator it = lane_info.cbegin(); it != lane_info.cend(); ++it) {
        cur_lane_classes_num[it->first] = {};
        cur_lane_pc_car_num[it->first] = {};
        for (int i = 0; i < classType->all.size(); ++i) {
            cur_lane_classes_num[it->first][classType->all[i]] = 0;
        }
        for (int pc_id = 1; pc_id <  + 1; ++pc_id) {
            cur_lane_pc_car_num[it->first][pc_id] = {};
        }
        cur_lane_id[it->first] = {};
        cur_lane_id_for_flow[it->first] = {};
        cur_statistics[it->first] = {};
        cur_lane_mean_v[it->first] = Mean_V_Info::default_mean_v;
    }
}
//获取计算统计量和异常事件检测的预备信息
void Traffic_Flow::get_pre_info(xt::xarray<double> &input_info, xt::xarray<double> &boxes_final) {
    init_cur_info();// yi duiqi
    get_cur_id_info(input_info, boxes_final);   //yk:8.29-14:19
    get_current_lane_id();   //python has not this function
    get_id_count();// yi duiqi
    save_history_id_speed(); //yi duiqi
}
//获取当前检测信息
void Traffic_Flow::get_cur_id_info(xt::xarray<double> &input_info, xt::xarray<double> &boxes_final) {
    /*xcb note python no use
    get_original_box_lane(boxes_final);
    std::map<int, int> match_res = match_org_track_res(input_info, boxes_final);

    if (int(param["use_lastest_lonlat"]) == 1){
        lonlat_to_xyz_batch(input_info, param["lon1_lastest"], param["lat1_lastest"], param["angle1_lastest"]);
    }*/

    int id_index = Pc_Info_Index::all_area_id;
    std::vector<int> index = {Pc_Info_Index::x,  // 0
                              Pc_Info_Index::y,   // 1
                              Pc_Info_Index::bottom_front_right_x,     // 20
                              Pc_Info_Index::bottom_front_right_y,     // 21
                              Pc_Info_Index::bottom_front_left_x,      // 32
                              Pc_Info_Index::bottom_front_left_y,      // 33
                              Pc_Info_Index::bottom_behind_left_x,     // 41
                              Pc_Info_Index::bottom_behind_left_y,     // 42
                              Pc_Info_Index::bottom_behind_right_x,    // 29
                              Pc_Info_Index::bottom_behind_right_y     // 30
    };    // 为三维目标检测框的角点 is int. that is ok? index,is ok
    std::vector<int> index_x = {0, 2, 4, 6, 8};  // 中心点x，左前，右前，左后，右后
    std::vector<int> index_y = {1, 3, 5, 7, 9};  // 中心点y，左前，右前，左后，右后
    xt::xarray<double> input_info_temp = xt::empty<double>({int(input_info.shape(0)), int(index.size())});  //save chosen info
    for (int i = 0; i < index.size(); ++i) {
        xt::view(input_info_temp, xt::all(), i) = xt::view(input_info, xt::all(), index[i]);
    }

    xt::xarray<int> bmp_xy;
    if (input_info.shape(0)) {
        bmp_xy = lidar_to_bmp<double, int>(input_info_temp, index_x, index_y);  //yk:lidar_to_bmp output wrong
    } else {
        bmp_xy = xt::empty<int>({0, int(input_info_temp.shape(1))});
    }

    for (int i = 0; i < bmp_xy.shape(0); ++i) {
        xt::xarray<double> this_info = xt::view(input_info, i);
        xt::xarray<int> this_bmp_xy = xt::view(bmp_xy, i);

        if (std::find(classType->all.cbegin(), classType->all.cend(), int(this_info(Pc_Info_Index::c))) ==
            classType->all.cend()) {
            continue;
        }
        bool del_flag = false;
        for (int point_index=0; point_index<5; point_index++)
        {
            int point_px = this_bmp_xy(0 + 2 * point_index);
            int point_py = this_bmp_xy(1 + 2 * point_index);
            if (point_px >= config_matrix.shape(0) or point_py >= config_matrix.shape(1) or point_px <= 0 or point_py <= 0)
            {
                del_flag = true;
                break;
            }
        }
        if (del_flag)
        {
            continue;
        }
        int c_px = this_bmp_xy(0), c_py = this_bmp_xy(1);

        int this_lane_no = config_matrix(c_px, c_py, Config_Matrix_Index::LANE_NO);
        int this_lane = lane_no_lane_name.count(this_lane_no)!=0 ? lane_no_lane_name[this_lane_no] : -1;
        int calc_flow = config_matrix(c_px, c_py, Config_Matrix_Index::CALC_FLOW);
        int len_lane_name = std::to_string(this_lane).length();

        if (len_lane_name == normal_lane_str_len)
        {
            cur_lane_id[this_lane].push_back(this_info(id_index));
            cur_id_lane[int(this_info(id_index))] = this_lane;
            if (calc_flow==1)
            {
                cur_lane_id_for_flow[this_lane].push_back(this_info(id_index));
            }
        }
        std::vector<double>center = {this_info(Pc_Info_Index::x), this_info(Pc_Info_Index::y)};//python 3 wei
        if (this_lane == -1)
        {
            this_lane = lane_no_lane_name[1];
        }
        std::vector<double> distVec;
        auto res = lane_name_centerline_pcd_tree[this_lane]->nearest_point(center); //yk: the return of pcd_tree is not same
        for (double b : res) {
            distVec.push_back(b);
        }
        double pos_coincidence = 0;
        double time_gap_each = 0;
        if (res.size() != 0)
        {
            double dis_gap = 0;
            double dist = std::sqrt(std::pow((distVec[0] - center[0]), 2) + std::pow((distVec[1] - center[1]), 2));
            double lanecenter_0 = lane_name_centerline_mediumpoint[this_lane](lane_name_centerline_mediumpoint[this_lane].shape(0)-1, 0) -
                                  lane_name_centerline_mediumpoint[this_lane](0, 0);
            double lanecenter_1 = lane_name_centerline_mediumpoint[this_lane](lane_name_centerline_mediumpoint[this_lane].shape(0)-1, 1) -
                                  lane_name_centerline_mediumpoint[this_lane](0, 1);    
            double dist_direct = lanecenter_1 * center[0] - lanecenter_0 * center[1];
            if (dist_direct < 0)
            {
                dist *= -1;
            }              
            // cjm 0831 this_info(Pc_Info_Index::cur_frame_time) --> cur_time
            if (id_dist_list.count(int(this_info(id_index))) == 0)
            {
                id_dist_list[int(this_info(id_index))] = {{cur_time, dist, dis_gap,
                                                      pos_coincidence, time_gap_each, this_info(Pc_Info_Index::x),
                                                      this_info(Pc_Info_Index::l), this_info(Pc_Info_Index::y)}};
            }
            else
            {
                time_gap_each = std::abs(cur_time - id_dist_list[int(this_info(id_index))][id_dist_list[this_info(id_index)].size() - 1][0]) + 0.001;
                dis_gap = std::abs(dist - id_dist_list[int(this_info(id_index))][id_dist_list[this_info(id_index)].size() -1][1]);
                double dis_gap_coincidence;
                if (dis_gap<= 0.15)
                {
                    dis_gap_coincidence = 1 - dis_gap *4 /3;
                }
                else
                {
                    dis_gap_coincidence = std::exp(-2 *  std::pow(dis_gap- 0.15, 2) / 0.25) / (0.5 * std::sqrt(2 * M_PI));
                }
                double dis_gap_vel = 0.1 * dis_gap / time_gap_each;
                if (dis_gap_vel <= 0.15)
                {
                    pos_coincidence = 1- dis_gap_vel *4 / 3;
                }
                else
                {
                    pos_coincidence = std::exp(-2 * std::pow(dis_gap_vel - 0.15, 2) / 0.25) / (0.5 * std::sqrt(2 * M_PI));
                }
                pos_coincidence = pos_coincidence < 0.3 ? 0.3 : pos_coincidence;
                // cjm 0831 this_info(Pc_Info_Index::cur_frame_time) --> cur_time
                id_dist_list[int(this_info(id_index))].push_back({cur_time,
                                                             dist, dis_gap_coincidence, pos_coincidence,
                                                             time_gap_each,
                                                             this_info(Pc_Info_Index::x),
                                                             this_info(Pc_Info_Index::l),
                                                             this_info(Pc_Info_Index::y)});
            }
        }
        else
        {
            continue;
        }

        if (id_dist_list[int(this_info(id_index))].size() > 50)
        {
            id_dist_list[int(this_info(id_index))].erase(id_dist_list[int(this_info(id_index))].begin());
        }

        cur_id_info[int(this_info(id_index))] = std::make_shared<Detect_Info>();
        //fill data
        // int single_info = None;
        /*xcb note
        if (match_res.count(int(this_info[id_index]))) {
            std::cout<< ">>> This module doesn't complete!\n";
        } else {
            single_info = None;
        }*/
        int id_temp = int(this_info(id_index));
        cur_id_info[id_temp]->fill_data(  //yk:no wrong
                xt::view(this_info, xt::range(0, int(Pc_Info_Index::bottom_front_right_x))),
                this_bmp_xy,
                xt::view(this_info, xt::range(int(Pc_Info_Index::cur_frame_time), _)));
    }
    for (decltype(id_dist_list)::iterator iter = id_dist_list.begin(); iter != id_dist_list.end();)
    {
        if (input_info.shape(0) == 0)
        {
            continue;
        }
        // double cur_frame_time  = input_info(0, 44); // cjm 0831
        // if (cur_frame_time - id_dist_list[int(iter->first)][id_dist_list[int(iter->first)].size() - 1][0] >= 3 * 1000)
        if (cur_time - id_dist_list[int(iter->first)][id_dist_list[int(iter->first)].size() - 1][0] >= 3 * 1000)
        {
            iter = id_dist_list.erase(iter);
        }
        ++iter;
    }
}
#if 1
//获取各车道上的id信息
void Traffic_Flow::get_current_lane_id() {  //yk: contrary to python on the for cycle
    decltype(cur_id_info)::const_iterator it;
    for (it = cur_id_info.cbegin(); it != cur_id_info.cend(); ++it) {
        int c_px = it->second->center_px;
        int c_py = it->second->center_py;

        int this_lane_no = config_matrix(c_px, c_py, Config_Matrix_Index::LANE_NO);   //config
        int this_lane;
//        cout<<"lane_no:"<<this_lane_no<<endl;
//        cout<<"lane_nolane:"<<lane_no_lane_name.count(this_lane_no)<<endl;
        if (lane_no_lane_name.count(this_lane_no)) {
            this_lane = lane_no_lane_name[this_lane_no];
            it->second->lane_no = this_lane;
        } else {
            this_lane = -1;
            it->second->lane_no = -1;
        }
//        cout<<"this_lane:"<<this_lane<<endl;
        int calc_flow = config_matrix(c_px, c_py, Config_Matrix_Index::CALC_FLOW);
        int len_lane_name = std::to_string(this_lane).size();
//        std::cout << "xcb---------> lane_name: " << this_lane << "   this_lane_no: " << this_lane_no <<std::endl;
        // this _lane = 110001
        if (len_lane_name == normal_lane_str_len) {
//            std::cout << "xcb---------> it->second->pc_id: " << it->second->pc_id << std::endl;
            if (cur_lane_pc_car_num[this_lane].count(it->second->pc_id) and
                std::find(classType->motor_type.begin(), classType->motor_type.end(), int (it->second->class_id)) != classType->motor_type.end())
            {
                cur_lane_pc_car_num[this_lane][it->second->pc_id].push_back(it->first);

            }
            cur_lane_id[this_lane].push_back(it->first);
            cur_id_lane[it->first] = this_lane;
            cur_id_info[it->first]->lane_no = this_lane;//4位编号
            if (calc_flow == 1) {
                cur_lane_id_for_flow[this_lane].push_back(it->first);
            }
        }
    }
}

#endif

void Traffic_Flow::get_original_box_lane(xt::xarray<double> &boxes_final) {
//    cur_lane_ids_box = {}
//    cur_lane_box_info = {}
    if (boxes_final.shape(0)) {
        std::cout<< ">>> This module doesn't complete!\n";  //all_area track
    }
}

std::map<int, int> Traffic_Flow::match_org_track_res(xt::xarray<double> &input_info, xt::xarray<double> &boxes_final) {
    std::map<int, int> match_res = {};
    if (boxes_final.shape(0)) {
        std::cout<< ">>> This module doesn't complete!\n";
        return match_res;
    } else {
        return match_res;
    }
}
// void Traffic_Flow::get_id_count() {
//     for (decltype(cur_id_info)::const_iterator it = cur_id_info.cbegin(); it != cur_id_info.cend(); ++it) {
//         if (!id_count.count(it->first)) {
//             id_count[it->first] = std::make_shared<Cur_Time>(0, cur_time); //cur_time from csv
//         }
//         ++id_count[it->first]->times;
//             id_count[it->first]->cur_time = cur_time;
//     }

//     for (decltype(id_count)::const_iterator it = id_count.cbegin(); it != id_count.cend(); ++it) {
//         if (cur_id_info.count(it->first)) {
//             continue;
//         }
// //        std::cout<<cur_time - id_count[it->first]->cur_time<<std::endl;
// //      xcb: 该车10秒没出现，就删除
//         if (cur_time - id_count[it->first]->cur_time > 10 * 1000) {      // Take Care!!!. 此处需注意，C++ chrone获取时间戳为long类型，毫秒为最后三位，python的毫秒为浮点型，小数点后三位
//             id_count.erase(it->first);
//         }
//     }
// }
//获取id计数器
void Traffic_Flow::get_id_count() {
    for (decltype(cur_id_info)::const_iterator it = cur_id_info.cbegin(); it != cur_id_info.cend(); ++it) {
        if (!id_count.count(it->first)) {
            id_count[it->first] = std::make_shared<Cur_Time>(0, cur_time); //cur_time from csv
        }
        ++id_count[it->first]->times;
        id_count[it->first]->cur_time = cur_time;
        if(cur_id_lane.count(it->first) != 0 and cur_id_lane[it->first] > 0)
        {
            if (!id_history_lane.count(it->first))
            {
                id_history_lane[it->first] = {std::make_shared<History_Different_Lane_Info>(cur_id_lane[it->first])};
            }
            if (cur_id_lane[it->first] != id_history_lane[it->first][id_history_lane[it->first].size()-1]->lane)
            {
                id_history_lane[it->first].push_back(std::make_shared<History_Different_Lane_Info>(cur_id_lane[it->first]));
            }
        }
    }
    for (decltype(id_count)::const_iterator it = id_count.cbegin(); it != id_count.cend();) {
        if (cur_id_info.count(it->first)) {
            ++it;
            continue;
        }
//        std::cout<<cur_time - id_count[it->first]->cur_time<<std::endl;
//      xcb: 该车10秒没出现，就删除
        if (cur_time - id_count[it->first]->cur_time > 10 * 1000) {      // Take Care!!!. 此处需注意，C++ chrone获取时间戳为long类型，毫秒为最后三位，python的毫秒为浮点型，小数点后三位
            it = id_count.erase(it);
            --it;
            if (id_history_lane.count(it->first != 0))
            {
                id_history_lane.erase(it->first);
            }
            ++it;
        }
        else
        {
            ++it;
        }
    }
}

//yk::SumVector
template<typename T>
T SumVector(std::vector<T>& vec)
{
    T res = 0;
    for (size_t i=0; i<vec.size(); i++)
    {
        res += vec[i];
    }
    return res;
}
//xcb add
void Traffic_Flow::save_history_id_speed() {
    for (const auto &it: cur_id_info) {   // save the every car in every frame
        if (!history_speed_info.count(it.first)) {
            history_speed_info[it.first] = std::make_shared<History_Info>();
        }
        history_speed_info[it.first]->t.push_back(cur_id_info[it.first]->cur_id_time);
        history_speed_info[it.first]->x.push_back(cur_id_info[it.first]->x);
        history_speed_info[it.first]->px.push_back(cur_id_info[it.first]->center_px);
        history_speed_info[it.first]->y.push_back(cur_id_info[it.first]->y);
        history_speed_info[it.first]->py.push_back(cur_id_info[it.first]->center_py);
        history_speed_info[it.first]->v.push_back(cur_id_info[it.first]->v);
        history_speed_info[it.first]->car_l.push_back(cur_id_info[it.first]->l);

        history_speed_info[it.first]->label.push_back(cur_id_info[it.first]->class_id);

        if (history_speed_info[it.first]->t.size() == 1)
        {
            history_speed_info[it.first]->cal_speed.push_back(cur_id_info[it.first]->v);
            history_speed_info[it.first]->moving_average_speed.push_back(cur_id_info[it.first]->v);
            history_speed_info[it.first]->acceleration.push_back(0);
            history_speed_info[it.first]->x_diff.push_back(0);
        }
        else if (history_speed_info[it.first]->t.size() > 1 and
                history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
                history_speed_info[it.first]->t[0] != 0)
        {
            double d_range = std::pow((history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] -
                                      history_speed_info[it.first]->x[0]), 2) +
                             std::pow((history_speed_info[it.first]->y[history_speed_info[it.first]->y.size() - 1] -
                                      history_speed_info[it.first]->y[0]), 2);
            d_range = std::sqrt(d_range);
            d_range = std::floor(d_range * 1e3 + 0.5) / 1e3;  //what to do ?
            d_range /= (history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
                        history_speed_info[it.first]->t[0]);
            d_range = std::floor(d_range * 1e2 + 0.5) / 1e2;  //what to do ?  4she5ru?
            history_speed_info[it.first]->cal_speed.push_back(d_range);

            double moving_average_speed = SumVector(history_speed_info[it.first]->cal_speed) /
                   history_speed_info[it.first]->cal_speed.size();
//            double moving_average_speed = std::accumulate(history_speed_info[it.first]->cal_speed.begin(),
//                                                          history_speed_info[it.first]->cal_speed.end(), 0) /
//                                          double(history_speed_info[it.first]->cal_speed.size());  //yk:wrong  accumulate return is integer
            moving_average_speed = std::floor(moving_average_speed * 1e2 + 0.5) / 1e2;   //4she5ru
            history_speed_info[it.first]->moving_average_speed.push_back(moving_average_speed);

            double px = history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] - 
                        history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 2];
            double py = history_speed_info[it.first]->x[history_speed_info[it.first]->y.size() - 1] - 
                        history_speed_info[it.first]->x[history_speed_info[it.first]->y.size() - 2];

            double pos_delta;
            if (px == 0 and py == 0)
            {
                pos_delta = 0;
            }
            else
            {
                xt::xarray<double>v{px,py};
                xt::xarray<double>v_norm = v / xt::linalg::norm(v);
                double vehicle_angle = 270 * M_PI / 180 - cur_id_info[it.first]->angle;
                xt::xarray<double>angle_v = {std::cos(vehicle_angle), std::sin(vehicle_angle)};
                double cos_angle = xt::linalg::dot(v_norm, angle_v)(0);
                pos_delta = xt::linalg::norm(v);
                if (cos_angle < 0)
                {
                    pos_delta = -pos_delta;
                }
            }
            history_speed_info[it.first]->x_diff.push_back(pos_delta);
            if (history_speed_info[it.first]->x_diff.size() > 50) {
                int k = 0;
                for (int i = 0; i < history_speed_info[it.first]->x_diff.size(); ++i) {
                        if (history_speed_info[it.first]->x_diff[i] < 0) {
                            k++;
                        } else {}
                }
                history_speed_info[it.first]->normal_rate = k / 51.0;
            }

            //# 如果计算的平均速度个数超过了10个 那么就根据当前帧和前面第10帧的加速度
            double a;
            if (history_speed_info[it.first]->moving_average_speed.size() > 10) {
                a = (history_speed_info[it.first]->moving_average_speed[
                                history_speed_info[it.first]->moving_average_speed.size() - 1] -
                        history_speed_info[it.first]->moving_average_speed[
                                history_speed_info[it.first]->moving_average_speed.size() - 10]) /
                    (history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
                        history_speed_info[it.first]->t[0]);
                a = std::floor(a * 1e2 + 0.5) / 1e2;
            } else {
                a = (history_speed_info[it.first]->moving_average_speed[
                                history_speed_info[it.first]->moving_average_speed.size() - 1] -
                        history_speed_info[it.first]->moving_average_speed[0]) /
                    (history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
                        history_speed_info[it.first]->t[0]);
                a = std::floor(a * 1e2 + 0.5) / 1e2;
            }
            history_speed_info[it.first]->acceleration.push_back(a);
        }
        else
        {
            std::cout << ">>> ERROR : " + std::to_string(it.first) + "\n";
        }
        //# ？？？？？？？？？？？？？？  yk:save the new 10 frame
        if (history_speed_info[it.first]->t.size() > 9) {
            history_speed_info[it.first]->t.erase(history_speed_info[it.first]->t.begin(),
                                                    history_speed_info[it.first]->t.begin() +
                                                    (history_speed_info[it.first]->t.size() - 9));
            history_speed_info[it.first]->x.erase(history_speed_info[it.first]->x.begin(),
                                                    history_speed_info[it.first]->x.begin() +
                                                    (history_speed_info[it.first]->x.size() - 9));
            history_speed_info[it.first]->px.erase(history_speed_info[it.first]->px.begin(),
                                                    history_speed_info[it.first]->px.begin() +
                                                    (history_speed_info[it.first]->px.size() - 9));
            history_speed_info[it.first]->y.erase(history_speed_info[it.first]->y.begin(),
                                                    history_speed_info[it.first]->y.begin() +
                                                    (history_speed_info[it.first]->y.size() - 9));
            history_speed_info[it.first]->py.erase(history_speed_info[it.first]->py.begin(),
                                                    history_speed_info[it.first]->py.begin() +
                                                    (history_speed_info[it.first]->py.size() - 9));
            history_speed_info[it.first]->v.erase(history_speed_info[it.first]->v.begin(),
                                                    history_speed_info[it.first]->v.begin() +
                                                    (history_speed_info[it.first]->v.size() - 9));
            history_speed_info[it.first]->car_l.erase(history_speed_info[it.first]->car_l.begin(),
                                                        history_speed_info[it.first]->car_l.begin() +
                                                        (history_speed_info[it.first]->car_l.size() - 9));
            history_speed_info[it.first]->acceleration.erase(history_speed_info[it.first]->acceleration.begin(),
                                                                history_speed_info[it.first]->acceleration.begin() +
                                                                (history_speed_info[it.first]->acceleration.size() - 9));
        }
        if (history_speed_info[it.first]->moving_average_speed.size() > 50) {
            history_speed_info[it.first]->moving_average_speed.erase(
                    history_speed_info[it.first]->moving_average_speed.begin(),
                    history_speed_info[it.first]->moving_average_speed.begin() +
                    (history_speed_info[it.first]->moving_average_speed.size() - 50));
        }
        if (history_speed_info[it.first]->x_diff.size() > 50) {
            history_speed_info[it.first]->x_diff.erase(
                    history_speed_info[it.first]->x_diff.begin(),
                    history_speed_info[it.first]->x_diff.begin() +
                    (history_speed_info[it.first]->x_diff.size() - 50));
        }
        if (history_speed_info[it.first]->cal_speed.size() > 50) {
            history_speed_info[it.first]->cal_speed.erase(
                    history_speed_info[it.first]->cal_speed.begin(),
                    history_speed_info[it.first]->cal_speed.begin() +
                    (history_speed_info[it.first]->cal_speed.size() - 50));
        }
    }    //save the content in 3s
    if (!cur_id_info.empty()) {

        double currentframe_time = cur_id_info[cur_id_info.rbegin()->first]->cur_id_time;  //iterator    id
        
        for (decltype(history_speed_info)::const_iterator iterator = history_speed_info.cbegin();
                iterator != history_speed_info.cend();) {
            // std::cout<<"xcb -------cur_id_info rrrrr"<<std::endl;
            if (std::abs(currentframe_time - iterator->second->t[iterator->second->t.size()-1]) > 3 * 1000) {  //yk:wrong (id)-> (iterator->first)       
                iterator = history_speed_info.erase(iterator);
            }
            ++iterator;
        }
    }
}

//存储每个目标之前的速度,t,x, y, l
/*xcb note
void Traffic_Flow::save_history_id_speed() {
    int id = -1;
    for (const auto &it: cur_id_info) {   // save the every car in every frame
        id = it.first;
        int lane;
        if (cur_id_lane.count(id)){
            lane = cur_id_lane[id];
        }
        else{
            lane = 0;
        }
        if (!history_speed_info.count(it.first)) {
            history_speed_info[it.first] = std::make_shared<History_Info>();
        }
        history_speed_info[it.first]->t.push_back(cur_id_info[it.first]->cur_id_time);
        history_speed_info[it.first]->x.push_back(cur_id_info[it.first]->x);
        history_speed_info[it.first]->y.push_back(cur_id_info[it.first]->y);
        history_speed_info[it.first]->v.push_back(cur_id_info[it.first]->v);
        history_speed_info[it.first]->car_l.push_back(cur_id_info[it.first]->l);

        if (history_speed_info[it.first]->t.size() == 1)   // 该id是第一次出现
        {
            history_speed_info[it.first]->cal_speed.push_back(cur_id_info[it.first]->v);
            history_speed_info[it.first]->moving_average_speed.push_back(cur_id_info[it.first]->v);
            history_speed_info[it.first]->acceleration.push_back(0);
            history_speed_info[it.first]->x_diff.push_back(0);
            history_speed_info[it.first]->y_diff.push_back(0);
        }
        else if (history_speed_info[it.first]->t.size() > 1 and history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] - history_speed_info[it.first]->t[0] != 0)
        {
            double d_range = std::pow((history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] -
                                       history_speed_info[it.first]->x[0]), 2) +
                             std::pow((history_speed_info[it.first]->y[history_speed_info[it.first]->y.size() - 1] -
                                       history_speed_info[it.first]->y[0]), 2);
            d_range = std::sqrt(d_range);
            d_range = std::floor(d_range * 1e3 + 0.5) / 1e3;  //what to do ?
            d_range /= ((history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
                        history_speed_info[it.first]->t[0])/1000);  // t is ms  /1000 transfer second
            d_range = std::floor(d_range * 1e2 + 0.5) / 1e2;  //what to do ?  4she5ru?
            history_speed_info[it.first]->cal_speed.push_back(d_range);

            double moving_average_speed = SumVector(history_speed_info[it.first]->cal_speed) /
                                          history_speed_info[it.first]->cal_speed.size();
//            double moving_average_speed = std::accumulate(history_speed_info[it.first]->cal_speed.begin(),
//                                                          history_speed_info[it.first]->cal_speed.end(), 0) /
//                                          double(history_speed_info[it.first]->cal_speed.size());  //yk:wrong  accumulate return is integer
            moving_average_speed = std::floor(moving_average_speed * 1e2 + 0.5) / 1e2;   //4she5ru
            history_speed_info[it.first]->moving_average_speed.push_back(moving_average_speed);
            //# 记录id存储的帧数中的最大值和最小值
            if (history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] >
                history_speed_info[it.first]->x_max) {
                history_speed_info[it.first]->x_max = history_speed_info[it.first]->x[
                        history_speed_info[it.first]->x.size() - 1];
            }
            if (history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] <
                history_speed_info[it.first]->x_max) {  //yk:x_max->x_min
                history_speed_info[it.first]->x_min = history_speed_info[it.first]->x[
                        history_speed_info[it.first]->x.size() - 1];
            }
            if (history_speed_info[it.first]->y[history_speed_info[it.first]->y.size()-1] >
                history_speed_info[it.first]->y_max){
                history_speed_info[it.first]->y_max = history_speed_info[it.first]->y[
                        history_speed_info[it.first]->y.size()-1];
            }
            if (history_speed_info[it.first]->y[history_speed_info[it.first]->y.size()-1] <
                history_speed_info[it.first]->y_min){
                history_speed_info[it.first]->y_min = history_speed_info[it.first]->y[
                        history_speed_info[it.first]->y.size()-1];
            }
            //# 计算id前后帧的位置偏移
            if (history_speed_info[it.first]->x.size() >= 2)
            {
                history_speed_info[it.first]->x_diff.push_back(
                        history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] -
                        history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 2]);
                history_speed_info[it.first]->y_diff.push_back(
                        history_speed_info[it.first]->y[history_speed_info[it.first]->y.size() - 1] -
                        history_speed_info[it.first]->y[history_speed_info[it.first]->y.size() - 2]);
            }
            else
            {
                history_speed_info[it.first]->x_diff.push_back(0);//yk:no wrong 9.5-18:03
                history_speed_info[it.first]->y_diff.push_back(0);
            }
            //# 根据前后帧的差，来判断该目标是否发生了前后偏移的情况
            if (history_speed_info[id]->y_diff.size() > 50) {
                int k = 0;
//                for (int i = 0; i < history_speed_info[it.first]->x_diff.size(); ++i) {
//                    if (history_speed_info[it.first]->x_diff[i] < 0) {
//                        k++;
//                    } else {}
//                }
                if (lane == 0){
                    continue;
                }
                else{
                    if (lane_info[lane]->radar_direct ==1){
                        for (int i = 0; i < history_speed_info[it.first]->y_diff.size(); ++i) {
                            if (history_speed_info[it.first]->y_diff[i] < 0) {
                                k++;
                            } else {}
                        }
                    }
                    if (lane_info[lane]->radar_direct == -1) {
                        for (int i = 0; i < history_speed_info[it.first]->y_diff.size(); ++i) {
                            if (history_speed_info[it.first]->y_diff[i] > 0) {
                                k++;
                            } else {}
                        }
                    }
                }

                history_speed_info[it.first]->normal_rate = k / 51.0;
            }

            //# 如果计算的平均速度个数超过了10个 那么就根据当前帧和前面第10帧的加速度
            double a;
            if (history_speed_info[it.first]->moving_average_speed.size() > 10) {
                a = (history_speed_info[it.first]->moving_average_speed[
                             history_speed_info[it.first]->moving_average_speed.size() - 1] -
                     history_speed_info[it.first]->moving_average_speed[
                             history_speed_info[it.first]->moving_average_speed.size() - 10]) /
                    ((history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
                     history_speed_info[it.first]->t[0]) / 1000);
                a = std::floor(a * 1e2 + 0.5) / 1e2;
            } else {
                a = (history_speed_info[it.first]->moving_average_speed[
                             history_speed_info[it.first]->moving_average_speed.size() - 1] -
                     history_speed_info[it.first]->moving_average_speed[0]) /
                    ((history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
                     history_speed_info[it.first]->t[0])/1000);
                a = std::floor(a * 1e2 + 0.5) / 1e2;
            }
            history_speed_info[it.first]->acceleration.push_back(a);
        }
        else{
            std::cout << ">>> ERROR : " + std::to_string(it.first) + "\n";
        }

        //# ？？？？？？？？？？？？？？  yk:save the new 10 frame
        if (history_speed_info[it.first]->t.size() > 9) {
            history_speed_info[it.first]->t.erase(history_speed_info[it.first]->t.begin(),
                                                  history_speed_info[it.first]->t.begin() +
                                                  (history_speed_info[it.first]->t.size() - 9));
            history_speed_info[it.first]->x.erase(history_speed_info[it.first]->x.begin(),
                                                  history_speed_info[it.first]->x.begin() +
                                                  (history_speed_info[it.first]->x.size() - 9));
            history_speed_info[it.first]->y.erase(history_speed_info[it.first]->y.begin(),
                                                  history_speed_info[it.first]->y.begin() +
                                                  (history_speed_info[it.first]->y.size() - 9));
            history_speed_info[it.first]->v.erase(history_speed_info[it.first]->v.begin(),
                                                  history_speed_info[it.first]->v.begin() +
                                                  (history_speed_info[it.first]->v.size() - 9));
            history_speed_info[it.first]->car_l.erase(history_speed_info[it.first]->car_l.begin(),
                                                      history_speed_info[it.first]->car_l.begin() +
                                                      (history_speed_info[it.first]->car_l.size() - 9));
            history_speed_info[it.first]->acceleration.erase(history_speed_info[it.first]->acceleration.begin(),
                                                             history_speed_info[it.first]->acceleration.begin() +
                                                             (history_speed_info[it.first]->acceleration.size() - 9));
        }
        if (history_speed_info[it.first]->moving_average_speed.size() > 50) {
            history_speed_info[it.first]->moving_average_speed.erase(
                    history_speed_info[it.first]->moving_average_speed.begin(),
                    history_speed_info[it.first]->moving_average_speed.begin() +
                    (history_speed_info[it.first]->moving_average_speed.size() - 50));
        }
        if (history_speed_info[it.first]->x_diff.size() > 50) {
            history_speed_info[it.first]->x_diff.erase(
                    history_speed_info[it.first]->x_diff.begin(),
                    history_speed_info[it.first]->x_diff.begin() +
                    (history_speed_info[it.first]->x_diff.size() - 50));
        }
        if (history_speed_info[it.first]->y_diff.size() > 50) {
            history_speed_info[it.first]->y_diff.erase(
                    history_speed_info[it.first]->y_diff.begin(),
                    history_speed_info[it.first]->y_diff.begin() +
                    (history_speed_info[it.first]->y_diff.size() - 50));}
        if (history_speed_info[it.first]->cal_speed.size() > 50) {
            history_speed_info[it.first]->cal_speed.erase(
                    history_speed_info[it.first]->cal_speed.begin(),
                    history_speed_info[it.first]->cal_speed.begin() +
                    (history_speed_info[it.first]->cal_speed.size() - 50));
        }
    }    //save the content in 3s
    for (decltype(history_speed_info)::const_iterator iterator = history_speed_info.cbegin();
         iterator != history_speed_info.cend(); ++iterator) {
        int lng = iterator->second->t.size();
        double save_time = iterator->second->t.at(lng-1);
        if (std::abs(cur_time - save_time) > 30 * 1000) {  //yk:wrong (id)-> (iterator->first)
            history_speed_info.erase(iterator->first);   //yk:cannot come into
        }
    }

        for (decltype(history_speed_info)::const_iterator iterator = history_speed_info.cbegin();
             iterator != history_speed_info.cend(); ++iterator) {
//            std::cout<<"for id"<<std::endl;
//            std::cout<<iterator->first<<std::endl;
            int lng = iterator->second->t.size();
            double save_time = iterator->second->t.at(lng-1);
            if (std::abs(cur_time - save_time) > 30 * 1000) {  //yk:wrong (id)-> (iterator->first)
                history_speed_info.erase(iterator->first);   //yk:cannot come into
//                std::cout<<"erase id"<<std::endl;
//                std::cout<<iterator->first<<std::endl;

            }
        }

    }
*/
//void Traffic_Flow::save_history_id_speed() {
//    int id = -1;
//    for (const auto &it: cur_id_info) {   // save the every car in every frame
//        id = it.first;
//        if (!history_speed_info.count(it.first)) {
//            history_speed_info[it.first] = std::make_shared<History_Info>();
//        }
//        history_speed_info[it.first]->t.push_back(cur_id_info[it.first]->cur_id_time);
//        history_speed_info[it.first]->x.push_back(cur_id_info[it.first]->x);
//        history_speed_info[it.first]->y.push_back(cur_id_info[it.first]->y);
//        history_speed_info[it.first]->v.push_back(cur_id_info[it.first]->v);
//        history_speed_info[it.first]->car_l.push_back(cur_id_info[it.first]->l);
//
//        if (history_speed_info[it.first]->t.size() == 1)
//        {
//            history_speed_info[it.first]->cal_speed.push_back(cur_id_info[it.first]->v);
//            history_speed_info[it.first]->moving_average_speed.push_back(cur_id_info[it.first]->v);
//            history_speed_info[it.first]->acceleration.push_back(0);
//            history_speed_info[it.first]->x_diff.push_back(0);
//        }
//        else if (history_speed_info[it.first]->t.size() > 1 and
//                   history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
//                   history_speed_info[it.first]->t[0] != 0)
//        {
//            double d_range = std::pow((history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] -
//                                       history_speed_info[it.first]->x[0]), 2) +
//                             std::pow((history_speed_info[it.first]->y[history_speed_info[it.first]->y.size() - 1] -
//                                       history_speed_info[it.first]->y[0]), 2);
//            d_range = std::sqrt(d_range);
//            d_range = std::floor(d_range * 1e3 + 0.5) / 1e3;  //what to do ?
//            d_range /= (history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
//                        history_speed_info[it.first]->t[0]);
//            d_range = std::floor(d_range * 1e2 + 0.5) / 1e2;  //what to do ?  4she5ru?
//            history_speed_info[it.first]->cal_speed.push_back(d_range);
//
//            double moving_average_speed = SumVector(history_speed_info[it.first]->cal_speed) /
//                    history_speed_info[it.first]->cal_speed.size();
////            double moving_average_speed = std::accumulate(history_speed_info[it.first]->cal_speed.begin(),
////                                                          history_speed_info[it.first]->cal_speed.end(), 0) /
////                                          double(history_speed_info[it.first]->cal_speed.size());  //yk:wrong  accumulate return is integer
//            moving_average_speed = std::floor(moving_average_speed * 1e2 + 0.5) / 1e2;   //4she5ru
//            history_speed_info[it.first]->moving_average_speed.push_back(moving_average_speed);
//            //# 记录id存储的帧数中的最大值和最小值
//            if (history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] >
//                history_speed_info[it.first]->x_max) {
//                history_speed_info[it.first]->x_max = history_speed_info[it.first]->x[
//                        history_speed_info[it.first]->x.size() - 1];
//            }
//            if (history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] <
//                history_speed_info[it.first]->x_max) {  //yk:x_max->x_min
//                history_speed_info[it.first]->x_min = history_speed_info[it.first]->x[
//                        history_speed_info[it.first]->x.size() - 1];
//            }
//            //# 计算id前后帧的位置偏移
//            if (history_speed_info[it.first]->x.size() >= 2)
//            {
//                history_speed_info[it.first]->x_diff.push_back(
//                        history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 1] -
//                        history_speed_info[it.first]->x[history_speed_info[it.first]->x.size() - 2]);
//            }
////            if (history_speed_info[it.first]->y.size() >= 2){
////                history_speed_info[it.first]->y_diff.push_back(
////                        history_speed_info[it.first]->y[history_speed_info[it.first]->y.size() - 1] -
////                        history_speed_info[it.first]->y[history_speed_info[it.first]->y.size() - 2]);
////            }
//            else
//            {
//                history_speed_info[it.first]->x_diff.push_back(0);//yk:no wrong 9.5-18:03
////                history_speed_info[it.first]->y_diff.push_back(0);
//            }
//            //# 根据前后帧的差，来判断该目标是否发生了前后偏移的情况
//            if (!cur_id_lane.count(id)){
//                continue;
//            }
//            int lane = cur_id_lane[id];
//            if (history_speed_info[it.first]->x_diff.size() > 50) {
//                int k = 0;
//                for (int i = 0; i < history_speed_info[it.first]->x_diff.size(); ++i) {
//                        if (history_speed_info[it.first]->x_diff[i] < 0) {
//                            k++;
//                        } else {}
////                if (lane_info[lane]->radar_direct ==1){
////                    for (int i = 0; i < history_speed_info[it.first]->y_diff.size(); ++i) {
////                        if (history_speed_info[it.first]->y_diff[i] < 0) {
////                            k++;
////                        } else {}
////                    }
////                }
////                if (lane_info[lane]->radar_direct ==0){
////                    for (int i = 0; i < history_speed_info[it.first]->y_diff.size(); ++i) {
////                        if (history_speed_info[it.first]->y_diff[i] > 0) {
////                            k++;
////                        } else {}
////                    }
//                }
//                history_speed_info[it.first]->normal_rate = k / 51.0;
//            }
//
//            //# 如果计算的平均速度个数超过了10个 那么就根据当前帧和前面第10帧的加速度
//            double a;
//            if (history_speed_info[it.first]->moving_average_speed.size() > 10) {
//                a = (history_speed_info[it.first]->moving_average_speed[
//                             history_speed_info[it.first]->moving_average_speed.size() - 1] -
//                     history_speed_info[it.first]->moving_average_speed[
//                             history_speed_info[it.first]->moving_average_speed.size() - 10]) /
//                    (history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
//                     history_speed_info[it.first]->t[0]);
//                a = std::floor(a * 1e2 + 0.5) / 1e2;
//            } else {
//                a = (history_speed_info[it.first]->moving_average_speed[
//                             history_speed_info[it.first]->moving_average_speed.size() - 1] -
//                     history_speed_info[it.first]->moving_average_speed[0]) /
//                    (history_speed_info[it.first]->t[history_speed_info[it.first]->t.size() - 1] -
//                     history_speed_info[it.first]->t[0]);
//                a = std::floor(a * 1e2 + 0.5) / 1e2;
//            }
//            history_speed_info[it.first]->acceleration.push_back(a);
//        }
//        else
//        {
//            std::cout << ">>> ERROR : " + std::to_string(it.first) + "\n";
//        }
//        //# ？？？？？？？？？？？？？？  yk:save the new 10 frame
//        if (history_speed_info[it.first]->t.size() > 9) {
//            history_speed_info[it.first]->t.erase(history_speed_info[it.first]->t.begin(),
//                                                  history_speed_info[it.first]->t.begin() +
//                                                  (history_speed_info[it.first]->t.size() - 9));
//            history_speed_info[it.first]->x.erase(history_speed_info[it.first]->x.begin(),
//                                                  history_speed_info[it.first]->x.begin() +
//                                                  (history_speed_info[it.first]->x.size() - 9));
//            history_speed_info[it.first]->y.erase(history_speed_info[it.first]->y.begin(),
//                                                  history_speed_info[it.first]->y.begin() +
//                                                  (history_speed_info[it.first]->y.size() - 9));
//            history_speed_info[it.first]->v.erase(history_speed_info[it.first]->v.begin(),
//                                                  history_speed_info[it.first]->v.begin() +
//                                                  (history_speed_info[it.first]->v.size() - 9));
//            history_speed_info[it.first]->car_l.erase(history_speed_info[it.first]->car_l.begin(),
//                                                      history_speed_info[it.first]->car_l.begin() +
//                                                      (history_speed_info[it.first]->car_l.size() - 9));
//            history_speed_info[it.first]->acceleration.erase(history_speed_info[it.first]->acceleration.begin(),
//                                                             history_speed_info[it.first]->acceleration.begin() +
//                                                             (history_speed_info[it.first]->acceleration.size() - 9));
//        }
//        if (history_speed_info[it.first]->moving_average_speed.size() > 50) {
//            history_speed_info[it.first]->moving_average_speed.erase(
//                    history_speed_info[it.first]->moving_average_speed.begin(),
//                    history_speed_info[it.first]->moving_average_speed.begin() +
//                    (history_speed_info[it.first]->moving_average_speed.size() - 50));
//        }
//        if (history_speed_info[it.first]->x_diff.size() > 50) {
//            history_speed_info[it.first]->x_diff.erase(
//                    history_speed_info[it.first]->x_diff.begin(),
//                    history_speed_info[it.first]->x_diff.begin() +
//                    (history_speed_info[it.first]->x_diff.size() - 50));
//        }
//        if (history_speed_info[it.first]->cal_speed.size() > 50) {
//            history_speed_info[it.first]->cal_speed.erase(
//                    history_speed_info[it.first]->cal_speed.begin(),
//                    history_speed_info[it.first]->cal_speed.begin() +
//                    (history_speed_info[it.first]->cal_speed.size() - 50));
//        }
//    }    //save the content in 3s
//    if (!cur_id_info.empty()) {
//        double currentframe_time = cur_id_info[id]->cur_id_time;   //iterator    id
//
//        for (decltype(history_speed_info)::const_iterator iterator = history_speed_info.cbegin();
//             iterator != history_speed_info.cend(); ++iterator) {
////            std::cout<<"for id"<<std::endl;
////            std::cout<<iterator->first<<std::endl;
//            if (std::abs(currentframe_time - iterator->second->t[iterator->second->t.size()-1]) > 3) {  //yk:wrong (id)-> (iterator->first)
////                history_speed_info.erase(iterator->first);   //yk:cannot come into
////                std::cout<<"erase id"<<std::endl;
////                std::cout<<iterator->first<<std::endl;
////                history_speed_info.erase(iterator->first);   //yk:wrong:after erase id 4383382, cbegin-id change to 131079
////                for id
////                4383382
////                erase id
////                4383382
////                for id
////                131079
//            }
//        }
////        for (int i = 0; i < history_speed_info.size(); ++i) {
////            if (std::abs(currentframe_time - history_speed_info[i]->t[history_speed_info[i]->t.size()-1]) > 3) {
////                history_speed_info.erase(history_speed_info.);   //yk:cannot come into
////            }
////        }
//
//    }
//}
//计算瞬时量

void Traffic_Flow::get_instantaneous_info() {
    /**""" 2.1 获取每个车道上依照到车道出口距离排序的车的信息"""*/
    get_sorted_car_info_by_dis_to_out();

    /**""" 2.2 计算当前帧的车头间距"""*/
    get_space_headway();

    /**""" 2.3 车头间距+所有车长"""   queue_length  */
    get_queue_length_by_space_headway();

    /**""" 2.4 计算车道占有率"""   lane_occupancy  */
    get_lane_occupancy();

    /**""" 2.5 获取车道上各类别的数量"""*/
    get_cur_lane_classes_num();

    /**""" 2.6 计算当前帧每条车道的平均速度"""*/
    get_cur_lane_mean_v();

    // cjm 0921
    get_queue_waitingtime();

    // cjm 0921
    // get_cur_crosswalk();
    
}
//获取每个车道上依照到车道出口距离排序的车的信息, 出口道方向从停止线第一辆车到最后一辆车
void Traffic_Flow::get_sorted_car_info_by_dis_to_out() {

        for (decltype(cur_lane_id)::const_iterator it = cur_lane_id.cbegin(); it != cur_lane_id.cend(); ++it) {
            try {
                if (!it->second.size()) {
                    cur_lane_sorted_id[it->first].clear();
                    continue;
                }
                static int out_px = None, out_py = None;

                if (!lane_info[it->first]->out_in) {
                    out_px = lane_info[it->first]->start_px;
                    out_py = lane_info[it->first]->start_py;
                } else {
                    out_px = lane_info[it->first]->end_px;
                    out_py = lane_info[it->first]->end_py;
                }
                std::vector<std::pair<int, int>> vec;

                // 遍历每一个id
                for (auto id_sort = cur_lane_id[it->first].begin(); id_sort != cur_lane_id[it->first].end(); ++id_sort){
                    if (cur_id_info.count(*id_sort)){
                        int dist = cur_id_info[*id_sort]->center_py - out_py;
                        std::pair<int, int> id_dist = {*id_sort, dist};
                        vec.push_back(id_dist);
                    }else{
                        continue;
                    }
                }
                if (vec.size()){
                    std::sort(vec.begin(), vec.end(), [](std::pair<int, int>& a, std::pair<int, int>& b){
                        return a.second < b.second;
                    });
                    std::vector<int> temp;
                    for (auto& p : vec){
                        temp.push_back(p.first);
                    }
//                    std::sort(temp.begin(), temp.end(), [this](int a, int b) {
//                        return int(std::abs(cur_id_info[a]->center_py - out_py)) <
//                               int(std::abs(cur_id_info[b]->center_py - out_py));
//                    });
                    cur_lane_sorted_id[it->first] = temp;
                }else{
                    continue;
                }

            }
            catch (...) {
                std::cout << "sorted Failed !" <<std::endl;
                continue;
            }
        }



}
//计算当前帧的车头间距
void Traffic_Flow::get_space_headway() {
    for (decltype(cur_lane_sorted_id)::const_iterator it = cur_lane_sorted_id.cbegin();
         it != cur_lane_sorted_id.cend(); ++it) {
        // 遍历每一个车道
        cur_statistics[it->first]["space_headway"] = {};
        if (cur_lane_sorted_id[it->first].size() >= 2) {
            //  出口道方向从停止线第一辆车到最后一辆车
            for (int i = 0; i < cur_lane_sorted_id[it->first].size() - 1; ++i) {
                int this_car_id = cur_lane_sorted_id[it->first][i];
                int next_car_id = cur_lane_sorted_id[it->first][i + 1];
                auto this_car_info = cur_id_info[this_car_id];
                auto next_car_info = cur_id_info[next_car_id];
                cur_statistics[it->first]["space_headway"].push_back(std::sqrt(
                        std::pow((next_car_info->x - this_car_info->x), 2) +
                        std::pow((next_car_info->y - this_car_info->y), 2)) +
                                                                     this_car_info->l / 2 - next_car_info->l / 2);
            }
        }
    }
}
//计算车道占有率
void Traffic_Flow::get_lane_occupancy() {
    for (decltype(cur_lane_sorted_id)::const_iterator it = cur_lane_sorted_id.cbegin(); it != cur_lane_sorted_id.cend(); ++it) {
        if (!cur_lane_sorted_id[it->first].size()){
            cur_statistics[it->first]["lane_occupancy"] = {0};
            continue;
        }
        float length = 0;
        for (int i = 0; i < cur_lane_sorted_id[it->first].size(); ++i) {
            length += cur_id_info[cur_lane_sorted_id[it->first][i]]->l;
        }
        cur_statistics[it->first]["lane_occupancy"] = {length / lane_info[it->first]->length};
    }
}
//获取车道上各类别的数量
void Traffic_Flow::get_cur_lane_classes_num() {
//    std::shared_ptr<Detect_Class_Type> classType = std::make_shared<Detect_Class_Type>();
    for (decltype(cur_lane_sorted_id)::const_iterator it = cur_lane_sorted_id.cbegin(); it != cur_lane_sorted_id.cend(); ++it) {
        for (const auto & id : cur_lane_sorted_id[it->first]) {
            int this_class_id = cur_id_info[id]->class_id;
            cur_lane_classes_num[it->first][this_class_id] += 1;
        }
        cur_lane_classes_num2[it->first]["non_motor_type"] = 0;     // 注意，cur_lane_classes_num变量从这里开始一分为2，key为string，使用cur_lane_classes_num2，int用原来的
        for (const auto & class_id : classType->non_motor_type) {
            cur_lane_classes_num2[it->first]["non_motor_type"] += cur_lane_classes_num[it->first][class_id];
        }
        cur_lane_classes_num2[it->first]["motor_type"] = 0;
        for (const auto & class_id : classType->motor_type) {
            cur_lane_classes_num2[it->first]["motor_type"] += cur_lane_classes_num[it->first][class_id];
        }
        cur_lane_classes_num2[it->first]["all"] = 0;
        for (const auto & class_id : classType->all) {
            cur_lane_classes_num2[it->first]["all"] += cur_lane_classes_num[it->first][class_id];
        }
    }
}
//计算当前帧每条车道的平均速度
void Traffic_Flow::get_cur_lane_mean_v() {
    for (decltype(cur_lane_sorted_id)::const_iterator it = cur_lane_sorted_id.cbegin();
         it != cur_lane_sorted_id.cend(); ++it) {
        if (!cur_lane_sorted_id[it->first].size()){
            continue;
        }
        double sum_v = 0;
        for (const auto id : cur_lane_sorted_id[it->first]) {
            sum_v += cur_id_info[id]->v;
        }
        cur_lane_mean_v[it->first] = sum_v * 3.6 / cur_lane_sorted_id[it->first].size();
    }
}

// // cjm 0921 no signal input,canot achieve
// void Traffic_Flow::get_cur_lane_crosswalk() {
//     pedstrain_waiting_area_for_crosswalk = map_param["map_param"]["pedstrain_waiting_area_for _crosswalk"];

//     pedstrain_waiting_area = map_param["map_param"]["pedstrain_waiting_area"];

//     for (auto & it : pedstrain_waiting_area)
//     {
//         for (decltype(cur_id_info)::iterator id = cur_id_info.begin(); id != cur_id_info.end(); ++id)
//         {
//             if (std::find(classType->people.begin(), classType->people.end(), int (id->second->class_id)) == classType->people.end() and
//                 std::find(classType->non_motor_type.begin(), classType->non_motor_type.end(), int (id->second->class_id)) == classType->non_motor_type.end())
//             {
//                 continue;
//             }
//             int center_px = id->second->center_px;
//             int center_py = id->second->center_py;
//             int pedstrian_waiting_area_num = config_matrix(center_px, center_py, Config_Matrix_Index::EMERGENCY_AREA);
//             if (pedstrian_waiting_area_num != it)
//             {
//                 continue;
//             }

//             if (id->first) history_pedstrain_waiting_area[it]
//             {

//             }
//         }
//     }


// }

//车头间距+所有车长
// void Traffic_Flow::get_queue_length_by_space_headway() {
//     for (decltype(cur_lane_sorted_id)::const_iterator it = cur_lane_sorted_id.cbegin();
//          it != cur_lane_sorted_id.cend(); ++it) {
//         cur_statistics[it->first]["queue_length"] = {0};        // 此处由于python程序随意变类型，c++只能这么写了
//         if (!cur_lane_sorted_id[it->first].size()) {
//             continue;
//         }
//         if (lane_info[it->first]->out_in == 1) {
// //            continue;
//         }
// //        int sum_space_headway = cur_statistics[it->first]["space_headway"].size() == 0 ? 0 : std::accumulate(
// //                cur_statistics[it->first]["space_headway"].begin(), cur_statistics[it->first]["space_headway"].end(),0);  //yk:acumulate return the in class data
// //        float sum_space_headway = cur_statistics[it->first]["space_headway"].size() == 0 ? 0 : std::accumulate(
// //                cur_statistics[it->first]["space_headway"].begin(), cur_statistics[it->first]["space_headway"].end(),0);
//         float sum_space_headway = 0;
//         float sum_car_length = 0;
//         int this_car_id = 0;
//         if (cur_lane_sorted_id[it->first].size() >= 2) {
//             for (int i = 0; i < cur_lane_sorted_id[it->first].size() - 1; ++i) {
//                 this_car_id = cur_lane_sorted_id[it->first][i];
//                 int next_car_id = cur_lane_sorted_id[it->first][i + 1];
//                 sum_car_length += cur_id_info[this_car_id]->l;
//                 auto this_car_info = cur_id_info[this_car_id];
//                 auto next_car_info = cur_id_info[next_car_id];
//                 double dist =(std::sqrt(std::pow((next_car_info->x - this_car_info->x), 2) + std::pow((next_car_info->y - this_car_info->y), 2)) -
//                               this_car_info->l / 2 - next_car_info->l / 2);
//                 sum_space_headway += dist;
//             }
//             this_car_id = cur_lane_sorted_id[it->first][cur_lane_sorted_id[it->first].size()-1];
//             sum_car_length += cur_id_info[this_car_id]->l;
//         }
//         cur_statistics[it->first]["queue_length"] = {float (sum_car_length + sum_space_headway)};
//     }
// }


//车头间距+所有车长 cjm 1013
void Traffic_Flow::get_queue_length_by_space_headway() {

    std::map<int, std::vector<double>> lane_xy;
    std::vector<double> xy;
    for (auto item : map_param["map_param"]["lane_terminate_xy"].items())
    {
        int i = 1;
        for (auto v : item.value().items())
        {
            if(i%2)
            {
                xy.push_back(double(v.value()) - 5.0);      
            }
            else
            {
                xy.push_back(double(v.value()) + 5.0);  
            }
            ++i;
        }
        lane_xy[atoi(item.key().c_str())] = xy;
        xy.clear();      
    }

    for (decltype(cur_lane_sorted_id)::const_iterator it = cur_lane_sorted_id.cbegin();
         it != cur_lane_sorted_id.cend(); ++it) {
        cur_statistics[it->first]["queue_length"] = {0};        // 此处由于python程序随意变类型，c++只能这么写了
        cur_statistics[it->first]["queue_id"] = {};
        if (!cur_lane_sorted_id[it->first].size()) {
            continue;
        }
        if (lane_info[it->first]->out_in == 0) {
           continue;
        }
        
        if (std::to_string(it->first)[1] == '0')
        {
            continue;
        }
        
        if (lane_xy.count(it->first) != 0)
        {
            // 保存排队长度的id
            std::vector<float> queue_id_list;
            std::vector<std::shared_ptr<Detect_Info>> queue_list;
            std::vector<std::vector<double>> vehicles;
            std::vector<double> vehicles_l;
            
            // vector
            for (auto & key : it->second)
            {
                if (cur_id_info[key]->v < 0.5)
                {
                    queue_id_list.push_back(key);
                    queue_list.push_back(cur_id_info[key]);
                    vehicles.push_back({cur_id_info[key]->x, cur_id_info[key]->y});
                    vehicles_l.push_back(cur_id_info[key]->l);
                }
            }

            if (queue_list.size() == 0)
            {
                cur_statistics[it->first]["queue_length"] = {0};
                cur_statistics[it->first]["queue_id"] = {};
                continue;
            }
            else if (queue_list.size() == 1)
            {
                double l_x = lane_xy[it->first][0];
                double l_y = lane_xy[it->first][1];

                double x1 = queue_list[0]->x;
                double y1 = queue_list[0]->y;
                // 頭車到停止線的距離
                double dis_l = std::sqrt((x1 - l_x) * (x1 - l_x) + (y1 - l_y) * (y1 - l_y));
                // 一辆车的排队长度
                float dis_queue = dis_l + 0.5 * queue_list[0]->l;
                if (lane_info[it->first]->is_non_motorway)
                {
                    if (dis_l<5)
                    {
                        cur_statistics[it->first]["queue_length"] = {dis_queue};
                        cur_statistics[it->first]["queue_id"] = queue_id_list;
                    }
                    else
                    {
                        cur_statistics[it->first]["queue_length"] = {0};
                        cur_statistics[it->first]["queue_id"] = {};
                    }
                }
                else
                {
                    if (dis_l<15)
                    {
                        cur_statistics[it->first]["queue_length"] = {dis_queue};
                        cur_statistics[it->first]["queue_id"] = queue_id_list;
                    }
                    else
                    {
                        cur_statistics[it->first]["queue_length"] = {0};
                        cur_statistics[it->first]["queue_id"] = {};
                    }
                
                }
                  continue;
            }
            else if (queue_list.size() >= 2)
            {
                double l_x = lane_xy[it->first][0];
                double l_y = lane_xy[it->first][1];

                // 前车位置列表
                std::vector<std::vector<double>> head_vehicles = {{l_x, l_y}};  
                head_vehicles.insert(head_vehicles.end(), vehicles.begin(), vehicles.end());  

                std::vector<double> head_vehicles_l = {0};  
                head_vehicles_l.insert(head_vehicles_l.end(), vehicles_l.begin(), vehicles_l.end()); 

                std::vector<double> vehicles_gaps;  
                std::vector<double> vehicles_head_tail_gaps;  

                for (int i = 0; i < head_vehicles.size()-2; ++i) 
                {  
                    double diff = std::sqrt(std::pow(head_vehicles[i][0] - head_vehicles[i+1][0], 2) + std::pow(head_vehicles[i][1] - head_vehicles[i+1][1], 2));  
                    vehicles_gaps.push_back(diff);  
                    vehicles_head_tail_gaps.push_back(diff-0.5*head_vehicles_l[i] - 0.5*head_vehicles_l[i+1]); 
                }  
                bool queue_existence_flag = false;
                int queue_tail_vehicle_index = -1;

                for (int i = 0; i < vehicles_head_tail_gaps.size(); ++i)
                {
                    ++queue_tail_vehicle_index;
                    int lane_gaps = 50;
                    if (lane_info[it->first]->is_non_motorway)
                    {
                        lane_gaps = 5;
                    }
                    if (vehicles_head_tail_gaps[queue_tail_vehicle_index]<lane_gaps)
                    {
                        queue_existence_flag = true;
                    }
                    else
                    {
                        queue_tail_vehicle_index = queue_tail_vehicle_index-1;
                        break;
                    }
                }
                if (queue_existence_flag)
                {
                    double l_x = lane_xy[it->first][0];
                    double l_y = lane_xy[it->first][1];
                    double x1 = queue_list[queue_tail_vehicle_index]->x;
                    double y1 = queue_list[queue_tail_vehicle_index]->y;

                    double dis_l = std::sqrt((x1 - l_x) * (x1 - l_x) + (y1 - l_y) * (y1 - l_y));
                    // 一辆车的排队长度
                    float dis_queue = dis_l + 0.5 * queue_list[queue_tail_vehicle_index]->l;
                    cur_statistics[it->first]["queue_length"] = {dis_queue};

                    std::vector<float>::const_iterator First = queue_id_list.begin(); 
                    std::vector<float>::const_iterator Second = queue_id_list.begin() + queue_tail_vehicle_index +1;
                    std::vector<float> Arrs2(First, Second); 
                    cur_statistics[it->first]["queue_id"] = Arrs2;
                }
                else
                {
                    cur_statistics[it->first]["queue_length"] = {0};
                    cur_statistics[it->first]["queue_id"] = {};
                    continue;
                }
            }
        }
    }
}


// cjm 0921
void Traffic_Flow::get_queue_waitingtime() {
    for (auto & it : lane_info)
    {
        if (std::to_string(it.first).size() != normal_lane_str_len or 
            std::to_string(it.first)[1] == '0' )
        {
            continue;
        }

        cur_statistics[it.first]["waitingTime"].push_back(0);
        std::vector<int> sorted_ids = cur_lane_sorted_id[it.first];

        if (sorted_ids.size() >= 1)
        {
            for (auto & id : sorted_ids)
            {
                if (cur_id_info.find(id) == cur_id_info.end())
                {
                    continue;
                }

                if (std::find(classType->motor_type.begin(), classType->motor_type.end(), int (cur_id_info[id]->class_id)) == classType->motor_type.end())
                {
                    continue;
                }
                if (history_car_waiting_area[it.first].find(id) == history_car_waiting_area[it.first].end())
                {
                    double waiting_time = 0;
                    if (cur_id_info[id]->v > 0.3)
                    {
                        continue;
                    }
                    history_car_waiting_area[it.first][id] = {double(cur_time), double(cur_time), double(waiting_time)};
                }
                else
                {
                    if (cur_id_info[id]->v > 0.3)
                    {
                        continue;
                    }
                    history_car_waiting_area[it.first][id][1] = cur_time;
                    history_car_waiting_area[it.first][id][2] = cur_time - history_car_waiting_area[it.first][id][0];
                }
            }
            if (history_car_waiting_area.find(it.first) == history_car_waiting_area.end())
            {
                continue;
            }
            if (history_car_waiting_area[it.first].size() < 1)
            {
                continue;
            }
            std::vector<int> del_car_id_list = {};
            for (auto & key : history_car_waiting_area[it.first])
            {
                double waiting_time = history_car_waiting_area[it.first][key.first][2];
                double disappear_time = cur_time - history_car_waiting_area[it.first][key.first][1];

                if (cur_id_info.find(key.first) == cur_id_info.end())
                {
                    del_car_id_list.push_back(key.first);
                    continue;
                }
                if (cur_id_info[key.first]->v > 0.5 or waiting_time > 100 * 1000 or disappear_time > 5 * 1000)
                {
                    del_car_id_list.push_back(key.first);
                }
            }
            for (auto & del_id : del_car_id_list)
            {
                history_car_waiting_area[it.first].erase(del_id);
            }
            std::vector<int> waiting_time_list = {};
            for (auto & key : history_car_waiting_area[it.first])
            {
                waiting_time_list.push_back(history_car_waiting_area[it.first][key.first][2]);
            }
            float max_waiting_time = 0;
            if (waiting_time_list.size() > 0)
            {
                auto maxposition = max_element(waiting_time_list.begin(), waiting_time_list.end());
                float max_waiting_time = *maxposition;
            }
            else
            {
                float max_waiting_time = 0;
            }
            cur_statistics[it.first]["waitingTime"][0] = max_waiting_time;
        }
    }
}


//计算统计量
void Traffic_Flow::get_statistics_info(const bool &statistics_flag, xt::xarray<double> &boxes_final) {
    /**""" 3.1 获取统计时间段里断面不同id对应的所需信息,time,v"""*/
    get_history_lane_id_info_of_section();

    /**""" 3.2 计算断面法得到的车头时距:两车通过某断面的时间差 flag:是否进行统计"""*/
    get_time_headway_section(statistics_flag);//yk:not complete

    /**""" 3.3 计算跟车百分比 flag:是否进行统计"""*/
    get_follow_car_percent(statistics_flag);  //yk:not complete

    /**""" 3.4 计算时间占有率"""*/
    get_time_occupancy_by_id_time(statistics_flag);//yk:not complete

    /**""" 3.5 计算车道的车流量，假设时间段都满足，只进行车流量的统计 flag:是否进行统计"""*/
    get_lane_flow(statistics_flag);//yk:not complete

    /**""" 3.6 通过断面求平均车速"""*/
    get_lane_mean_v_info_by_section(statistics_flag);  // 断面的平均速度  //wrong in total statistics, lane_mean_v_info_by_section

    /**""" 3.7 获取整个统计时间段内检测到目标的所有速度、速度数量、平均速度"""*/
    get_lane_mean_v_info_by_all_v(statistics_flag, boxes_final);  // # 所有车辆在每一时刻的平均速度  //yk:9.9-16:02 no wrong

    /**""" 3.8 局部静态的拥堵评估 flag:进行评估的标志"""*/
    get_local_static_congestion_state(statistics_flag);

    /**""" 3.9 计算统计时间段内的平均车头间距"""*/
    get_lane_mean_space_headway_info(statistics_flag); //yk:9.9-16:02 wrong in update  //yk:9.26-11:09 fixed

    /**""" 3.10 统计时间段内的信息清零 flag:是否进行统计"""*/
    clear_history_lane_id_info_of_section(statistics_flag);


}
//yk:cur_lane_id_for_flow ={lane:[id] }
//获取统计时间段里断面不同id对应的所需信息,time,v
void Traffic_Flow::get_history_lane_id_info_of_section() {
    for (decltype(cur_lane_id_for_flow)::const_iterator it = cur_lane_id_for_flow.cbegin();
         it != cur_lane_id_for_flow.cend(); ++it) {
        // 遍历每一个车道
        for (const int & id : cur_lane_id_for_flow[it->first]) {
            // 遍历该车道上的每一辆车
            if (id_count[id]->times <= 4){
                continue;
            }
            history_lane_id_info_of_section[it->first][cur_id_info[id]->class_id][id] = {cur_id_info[id]->v, double (cur_time), double(cur_frame_id)};
            history_lane_id_order_of_section[it->first].push_back({{double (id), cur_id_info[id]->class_id}});
        }
    }
}
//计算断面法得到的车头时距:两车通过某断面的时间差 flag:是否进行统计 weikaifa
void Traffic_Flow::get_time_headway_section(const bool &statistics_flag) {
    if (statistics_flag){
        //YK:9.29
        decltype(history_lane_id_order_of_section)::iterator it;  //history_lane_id_order_of_section:  map{int,<vector<vector<double>>>}
        double this_id_time, last_id_time, this_frame_id, last_frame_id;
        int this_id, last_id;
        int this_class_id, last_class_id;
        double car_head_time_distance;

        for (it=history_lane_id_order_of_section.begin();it!=history_lane_id_order_of_section.end();it++)
        {
            //yk:clear the total_statistics1[lane]["time_headway"]
            total_statistics1[it->first]["time_headway"].clear();
            if (history_lane_id_order_of_section[it->first].size() > 1){

                // 遍历该车道上每一个 {car_id, class_id}
            for (int i = 0; i < history_lane_id_order_of_section[it->first].size()-1; ++i)
            {   //42-46(42car,43car) 112lane
                this_id = history_lane_id_order_of_section[it->first][i+1][0];
                this_class_id = history_lane_id_order_of_section[it->first][i+1][1];
                this_id_time = history_lane_id_info_of_section[it->first][this_class_id][this_id][1];
                this_frame_id = history_lane_id_info_of_section[it->first][this_class_id][this_id][2];
                // std::map<int, std::map<int, std::map<int, std::vector<vector<double>>>>> history_lane_id_info_of_section;

                last_id = history_lane_id_order_of_section[it->first][i][0];    // id
                last_class_id = history_lane_id_order_of_section[it->first][i][1];// class
                last_id_time = history_lane_id_info_of_section[it->first][last_class_id][last_id][1]; //time
                last_frame_id = history_lane_id_info_of_section[it->first][last_class_id][last_id][2];

                car_head_time_distance = std::abs(this_id_time-last_id_time);  //yk:not the Same class_id
                total_statistics1[it->first]["time_headway"].push_back(car_head_time_distance);
            }}
        }
        std::vector<std::vector<double>> chetoushiju_list = {};
        for(it=history_lane_id_order_of_section.begin();it!=history_lane_id_order_of_section.end();it++)  //use the lane
        {
            chetoushiju_list.push_back(total_statistics1[it->first]["time_headway"]);
        }
    }
}
//计算跟车百分比 flag:是否进行统计  weikaifa
void Traffic_Flow::get_follow_car_percent(const bool &statistics_flag) {
    if (statistics_flag){
        //yk:9.30
        decltype(total_statistics)::iterator it;   //it->first is lane

        // 遍历车道
        for(it=total_statistics.begin();it!=total_statistics.end();it++)  //use the lane
        {
            int cnt=0, zero_cnt=0, no_zero_cnt=0;

//            遍历该车道上每一个车头时间差
            for (int i = 0; i < total_statistics1[it->first]["time_headway"].size(); ++i)
            {
                //zero_cnt is the num of wrong num
                if(total_statistics1[it->first]["time_headway"][i]==0)
                {
                    zero_cnt+=1;
                    continue;
                }
                //cnt is the data!= 0 num and cnt < the set data
                if (total_statistics1[it->first]["time_headway"][i] < param["min_time_headway"] * 1000)
                {
                    cnt+=1;
                }
            }
            //cal no_zero_cnt
            no_zero_cnt = total_statistics1[it->first]["time_headway"].size() - zero_cnt;
            if (no_zero_cnt){
                total_statistics[it->first]["follow_car_percent"][0] = (double(cnt) / double(no_zero_cnt)) * 100;
            }else{
                total_statistics[it->first]["follow_car_percent"][0] = 0;
            }
        }
    }
}

//yk:10.8 ykmerge for get_time_occupancy_by_id_time
std::vector<std::vector<double>> ykmerge(std::map<int,std::vector<double>> infomap)
{
    std::vector<std::vector<double>> two_d_vec_interals;
    two_d_vec_interals.clear();
    for(decltype(infomap)::iterator it=infomap.begin();it!=infomap.end();it++)
    {
        if (two_d_vec_interals.size()==0 or two_d_vec_interals[two_d_vec_interals.size()-1][1]<it->second[0])
        {
            two_d_vec_interals.push_back(it->second);
        }
        else  //exist 交集
        {
            two_d_vec_interals[two_d_vec_interals.size()-1][1] = std::max(two_d_vec_interals[two_d_vec_interals.size()-1][1],it->second[1]);
        }
    }
    return two_d_vec_interals;
}
//计算时间占有率
void Traffic_Flow::get_time_occupancy_by_id_time(const bool &statistics_flag) {
    for (decltype(cur_lane_id_for_flow)::const_iterator it = cur_lane_id_for_flow.cbegin(); it != cur_lane_id_for_flow.cend(); ++it) {
        if (!cur_lane_id_for_flow[it->first].size()){
            continue;
        }

        for (const auto & id : cur_lane_id_for_flow[it->first]) {
            if (std::find(classType->motor_type.begin(), classType->motor_type.end(), int(cur_id_info[id]->class_id)) == classType->motor_type.end()){
                continue;
            }

            if (!history_lane_id_time_info[it->first].count(id)){
                history_lane_id_time_info[it->first][id] = {double (cur_time), double (cur_time)}; //yk:first start time, second time
            }
            else
            {
                history_lane_id_time_info[it->first][id][1] = double (cur_time);
            }
        }
    }
    if (statistics_flag){
        //yk:10.8
        decltype(history_lane_id_time_info)::iterator it;
        for(it=history_lane_id_time_info.begin();it!=history_lane_id_time_info.end();it++) //yk:for every lane
        {
            std::vector<std::vector<double>> two_d_vec_interals;
            two_d_vec_interals.clear();
            if (it->second.size()) //yk:history_lane_id_time_info is {int{int{[double,]}}}
            {
                two_d_vec_interals = ykmerge(it->second);  //yk:youwenti
            }

            //after,merge,include the before_interrals


            double occupy_time = 0.0;
            for (int i = 0; i < two_d_vec_interals.size(); ++i) {
                occupy_time += two_d_vec_interals[i][1] - two_d_vec_interals[i][0];
            }

            if (occupy_time > param["statistics_time"] * 1000)
            {
                total_statistics[it->first]["time_occupancy"][0] = 100;
            }
            else
            {
                total_statistics[it->first]["time_occupancy"][0] = occupy_time / (param["statistics_time"] * 10);
            }
        }

        //yk:12.01_clear

        for (auto it:lane_info) {
            history_lane_id_time_info[it.first].clear();
        }

    }
}
//计算车道的车流量，假设时间段都满足，只进行车流量的统计 weikaifa
void Traffic_Flow::get_lane_flow(const bool &statistics_flag) {
    if (statistics_flag){
        //yk:10.8
        //int->int->int->vector: lane->class_id->id->vector
        decltype(history_lane_id_info_of_section)::iterator it;
        for(it = history_lane_id_info_of_section.begin();it!=history_lane_id_info_of_section.end();it++)
        {
            decltype(it->second)::iterator it2;
            for(it2 = it->second.begin();it2!=it->second.end();it2++)
            {
                total_statistics[it->first]["lane_flow"][it2->first] = it2->second.size(); //the class_id car num
            }
//            for (auto class_id: classType->motor_type) {
//                total_statistics[it->first]["lane_flow"][10]+= total_statistics[it->first]["lane_flow"][class_id];  //yk:10means motor type total_statistics  int->string->int->double
//            }
        }
        //total_statistics[it->first]["lane_flow"][10]  10->"motor_type"
        //yk:10.8
    }
}
//通过断面求平均车速 weikaifa
void Traffic_Flow::get_lane_mean_v_info_by_section(const bool &statistics_flag) {
    for (decltype(cur_lane_id_for_flow)::const_iterator it = cur_lane_id_for_flow.cbegin(); it != cur_lane_id_for_flow.cend(); ++it) {
        for (const auto & id : it->second) {
            auto this_id_info = cur_id_info[id];
            lane_mean_v_info_by_section[it->first][std::to_string(int (this_id_info->class_id))]->update(this_id_info->v);
            lane_mean_v_info_by_section[it->first]["all"]->update(this_id_info->v);
        }
    }
    if (statistics_flag){
        //yk:10.9
        decltype(lane_mean_v_info_by_section)::iterator it;
        for(it=lane_mean_v_info_by_section.begin();it!=lane_mean_v_info_by_section.end();it++)
        {
            //cal mean_v
            for (auto class_id: classType->all)
            {
                lane_mean_v_info_by_section[it->first][std::to_string(class_id)]->get_mean_v();
            }
            lane_mean_v_info_by_section[it->first]["all"]->get_mean_v();
            //clear
            for (auto class_id: classType->all)
            {
                lane_mean_v_info_by_section[it->first][std::to_string(class_id)]->clear();
            }
            lane_mean_v_info_by_section[it->first]["all"]->clear();
        }
    }
}
//获取整个统计时间段内检测到目标的所有速度、速度数量、平均速度
void Traffic_Flow::get_lane_mean_v_info_by_all_v(const bool &statistics_flag, xt::xarray<double> &boxes_final) {
    for (decltype(cur_lane_id)::const_iterator it = cur_lane_id.cbegin(); it != cur_lane_id.cend(); ++it) {  //it is {lane:[id]}, it->first is lane
        for (int pc_id = 1; pc_id <  + 1; ++pc_id) {
            lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->cur_num = 0;
            lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->cur_target_length = 0;
            lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->cur_target_v = 0;
        }
        //cur_mean_v,cur_num,cur_target_v
        // the id list of c++ is reversed.
        for (const auto & id : it->second){             // it->second is [id]
            auto this_info = cur_id_info[id];
            if (std::find(classType->motor_type.begin(), classType->motor_type.end(), int (this_info->class_id)) == classType->motor_type.end()){
                continue;
            }
            if (it->second.size() <= 3 and this_info->v < 0.5){
                continue;
            }

            lane_mean_v_info_by_all_v[it->first]["null"][int (this_info->class_id)]->update(this_info->v); //Because it is a three-layer dictionary, the middle layer is replaced by blank
            lane_mean_v_info_by_all_v[it->first]["all"][0]->update(this_info->v);
            lane_mean_v_info_by_all_v[it->first]["pc_id"][int (this_info->pc_id)]->update(this_info->v);  //update() is to get sum_v, sum_num of the single base station
            lane_mean_v_info_by_all_v[it->first]["pc_id"][int (this_info->pc_id)]->cur_num += 1;
            lane_mean_v_info_by_all_v[it->first]["pc_id"][int (this_info->pc_id)]->cur_target_length += this_info->l;
            lane_mean_v_info_by_all_v[it->first]["pc_id"][int (this_info->pc_id)]->cur_target_v += this_info->v;

            double this_cur_target_v = lane_mean_v_info_by_all_v[it->first]["pc_id"][int (this_info->pc_id)]->cur_target_v;
            int this_cur_num = lane_mean_v_info_by_all_v[it->first]["pc_id"][int (this_info->pc_id)]->cur_num;
            if (this_cur_num == 0){
                lane_mean_v_info_by_all_v[it->first]["pc_id"][int (this_info->pc_id)]->cur_mean_v = 0;
            }else{
                lane_mean_v_info_by_all_v[it->first]["pc_id"][int (this_info->pc_id)]->cur_mean_v = this_cur_target_v / this_cur_num;
            }
            xt::xarray<int> congestion_map_x = bmp_to_congestion_map({this_info->center_px, this_info->center_py});
            lane_mean_v_info_by_all_v[it->first]["meter"][congestion_map_x(0)]->update(this_info->v);
        }
        // congestion/occupancy
        for (int pc_id = 1; pc_id <  + 1; ++pc_id) {
            if (boxes_final.shape(0)){
                std::cout<< ">>> This module didn't complete_7\n";
            }
            else
            {
                lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->cur_occupancy = lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->cur_target_length / 120.0;
                lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->congestion = judge_cur_congestion(
                        lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->cur_mean_v,
                        lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->cur_occupancy,
                        lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->cur_num);
            }
            if (lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->congestion){
                int val = lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->congestion;
                if (!lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion_count.count(val)){
                    lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion_count[val] = 1;  //init
                }
                else
                {
                    lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion_count[val] += 1;
                }  //Count the number of congestion
            }
        }
    }
    if (statistics_flag){
        decltype(lane_mean_v_info_by_all_v)::iterator it;
        for(it=lane_mean_v_info_by_all_v.begin();it!=lane_mean_v_info_by_all_v.end();it++)
        {



            for (auto class_id:classType->all)
            {
                lane_mean_v_info_by_all_v[it->first]["null"][class_id]->get_mean_v();  //cal mean_v
            }



            lane_mean_v_info_by_all_v[it->first]["all"][0]->get_mean_v();



            for (int pc_id = 1; pc_id < +1; ++pc_id)
            {
                lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->get_mean_v(); //cal mean_v
                //yk:linxiao  what for,for 基站拥挤度 for get_local_static_congestion_state  :  move to get_local_static_congestion_state
            }


            for (int i = 0; i < congestion_matrix.shape(0); ++i) {
                lane_mean_v_info_by_all_v[it->first]["meter"][i]->get_mean_v(); //cal mean_v
            }



            //yk:12.02 compared with python, lack of clear

        }
//        std::cout<<ops::getTimeStamp() - t_now<< std::endl;
        //yk:10.9 //        ";

    }
}
//统计时间段内的信息清零
void Traffic_Flow::clear_history_lane_id_info_of_section(const bool &statistics_flag) {
    if (statistics_flag){
        decltype(cur_lane_id_for_flow)::iterator it;
        for (it=cur_lane_id_for_flow.begin();it!=cur_lane_id_for_flow.end();it++)
        {
            history_lane_id_order_of_section[it->first].clear();
            for (auto class_id : classType->all)
            {
                history_lane_id_info_of_section[it->first][class_id].clear();
            }
        }
    }
}

int Traffic_Flow::judge_cur_congestion(double &cur_mean_v, const double &cur_occupancy, const int &cur_num) {
    int cur_congestion_state = -1;
    cur_mean_v *= 3.6;
    if (cur_num <= 3){
        cur_congestion_state = Congestion_Level::unblocked;
    }
    else
    {
        if (cur_mean_v >= 50 and cur_occupancy < 0.5)
        {
            cur_congestion_state = Congestion_Level::unblocked;
        }
        else if (cur_mean_v >= 50 and cur_occupancy >= 0.5)
        {
            cur_congestion_state = Congestion_Level::moderately_congested;
        }
        else if (cur_mean_v>= 30 and cur_mean_v < 50 and cur_occupancy < 0.4)
        {
            cur_congestion_state = Congestion_Level::unblocked;
        }
        else if (cur_mean_v>= 30 and cur_mean_v < 50 and cur_occupancy >= 0.4 and cur_occupancy < 0.5)
        {
            cur_congestion_state = Congestion_Level::slightly_congested;
        }
        else if (cur_mean_v>= 30 and cur_mean_v < 50 and cur_occupancy>= 0.5 and cur_occupancy < 0.7)
        {
            cur_congestion_state = Congestion_Level::moderately_congested;
        }
        else if (cur_mean_v>= 30 and cur_mean_v < 50 and cur_occupancy >= 0.7)
        {
            cur_congestion_state = Congestion_Level::seriously_congested;
        }
        else if (cur_mean_v >= 20 and cur_mean_v < 30 and cur_occupancy < 0.2)
        {
            cur_congestion_state = Congestion_Level::unblocked;
        }
        else if (cur_mean_v >= 20 and cur_mean_v < 30 and cur_occupancy >= 0.2 and cur_occupancy < 0.35)
        {
            cur_congestion_state = Congestion_Level::moderately_congested;
        }
        else if (cur_mean_v >= 20 and cur_mean_v < 30 and cur_occupancy >= 0.35 and cur_occupancy < 0.55)
        {
            cur_congestion_state = Congestion_Level::moderately_congested;
        }
        else if (cur_mean_v >= 20 and cur_mean_v < 30 and 0.55 <= cur_occupancy)
        {
            cur_congestion_state = Congestion_Level::seriously_congested;
        }
        else if (cur_mean_v < 20 and cur_occupancy < 0.15)
        {
            cur_congestion_state = Congestion_Level::unblocked;
        }
        else if (cur_mean_v < 20 and cur_occupancy >= 0.15 and cur_occupancy < 0.3)
        {
            cur_congestion_state = Congestion_Level::slightly_congested;
        }
        else if (cur_mean_v < 20 and cur_occupancy >= 0.3 and cur_occupancy < 0.4)
        {
            cur_congestion_state = Congestion_Level::moderately_congested;
        }
        else if (cur_mean_v < 20 and 0.4 <= cur_occupancy)
        {
            cur_congestion_state = Congestion_Level::seriously_congested;
        }
        else
        {
            cur_congestion_state = -1;
        }
    }
    return cur_congestion_state;
}
int judge_congestion_level_by_mean_v(std::shared_ptr<Lane_Info> lane_info, double mean_v, int car_num, int flag)
{
    auto state = None;
    if (flag==2)
    {
        int min_target_num = 4;
        if (car_num < min_target_num)
        {
            state = Congestion_Level::unblocked;
        }
        else
        {
            if (mean_v >= lane_info->unblocked_min_v)
            {
                state = Congestion_Level::unblocked;
            }
            else if (lane_info->slightly_congested_min_v <= mean_v < lane_info->slightly_congested_max_v)
            {
                state = Congestion_Level::slightly_congested;
            }
            else if (lane_info->moderately_congested_min_v <= mean_v < lane_info->moderately_congested_max_v)
            {
                state = Congestion_Level::moderately_congested;
            }
            else if (lane_info->seriously_congested_min_v <= mean_v < lane_info->seriously_congested_max_v)
            {
                state = Congestion_Level::seriously_congested;
            }
        }
    }
    else
    {
        if (mean_v >= lane_info->unblocked_min_v)
        {
            state = Congestion_Level::unblocked;
        }
        else if (lane_info->slightly_congested_min_v <= mean_v < lane_info->slightly_congested_max_v)
        {
            state = Congestion_Level::slightly_congested;
        }
        else if (lane_info->moderately_congested_min_v <= mean_v < lane_info->moderately_congested_max_v)
        {
            state = Congestion_Level::moderately_congested;
        }
        else if (lane_info->seriously_congested_min_v <= mean_v < lane_info->seriously_congested_max_v)
        {
            state = Congestion_Level::seriously_congested;
        }
    }
    return state;
}


typedef std::pair<int, int> PAIR;
struct CmpByValue {
    bool operator()(const PAIR& lhs, const PAIR& rhs) {
        return lhs.second < rhs.second;
    }
};

//局部静态的拥堵评估 weikaifa
void Traffic_Flow::get_local_static_congestion_state(const bool &statistics_flag) {
    if (statistics_flag){
        //yk:10.9-16:46
        map_event<std::shared_ptr<Mean_V_Info>>::triple_map_multi_type used_lane_mean_v_info = lane_mean_v_info_by_all_v;
        decltype(used_lane_mean_v_info)::iterator it;

        for (it=used_lane_mean_v_info.begin();it!=used_lane_mean_v_info.end();it++)
        {
            //yk:10.11
            //cal the statistice_congestion for advance
            for (int pc_id = 1; pc_id < +1; ++pc_id)
            {
                // set a variable list to sort the statistice_congestion_count
                std::map<int, int> this_pc_statistice_congestion_count_map = lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion_count;
                //sort the statistice_congestion_count
                /*把map中元素转存到vector中*/
                std::vector<PAIR> this_pc_statistice_congestion_count_vec(this_pc_statistice_congestion_count_map.begin(), this_pc_statistice_congestion_count_map.end());
                sort(this_pc_statistice_congestion_count_vec.begin(), this_pc_statistice_congestion_count_vec.end(), CmpByValue());
                //assign the statistice_congestion
                if (this_pc_statistice_congestion_count_vec[3].first == 1) //if (the most value is unblocked)
                {
                    //cal yougdu_num
                    int yongdu_num = this_pc_statistice_congestion_count_vec[0].second+
                                     this_pc_statistice_congestion_count_vec[1].second+
                                     this_pc_statistice_congestion_count_vec[2].second;
                    //cal unblocked_num
                    int unblocked_num = this_pc_statistice_congestion_count_vec[3].second;
                    if(float(yongdu_num) / float(unblocked_num) > 0.8)
                    {
                        if (this_pc_statistice_congestion_count_vec[3].first == 2)//if (slightly_congested_num is most)
                        {
                            lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion = Congestion_Level::slightly_congested;
                        }
                        else if (this_pc_statistice_congestion_count_vec[3].first == 3)  //  else if (medium_congested_num)
                        {
                            lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion = Congestion_Level::moderately_congested;
                        }
                        else if (this_pc_statistice_congestion_count_vec[3].first == 4)//else if (serious_congested_num)
                        {
                            lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion = Congestion_Level::seriously_congested;
                        }
                        else
                        {
                            lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion = Congestion_Level::unblocked;
                        }
                    }
                    else
                    {
                        lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion = Congestion_Level::unblocked;
                    }
                }
                else if (this_pc_statistice_congestion_count_vec[3].first == 2)
                {
                    lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion = Congestion_Level::slightly_congested;
                }
                else if (this_pc_statistice_congestion_count_vec[3].first == 3)
                {
                    lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion = Congestion_Level::moderately_congested;
                }
                else if (this_pc_statistice_congestion_count_vec[3].first == 4)
                {
                    lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion = Congestion_Level::seriously_congested;
                }
                else
                {
                    lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion = Congestion_Level::unblocked;
                }
            } //cal the statistice_congestion for advance done.



            lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion;

            //lane_local_static_congestion_state : int->string->int->int
            //对各车道进行拥堵评估  lane_local_static_congestion_state:int->string->int->int
            lane_local_static_congestion_state[it->first]["all"][0] =
                    judge_congestion_level_by_mean_v(lane_info[it->first],used_lane_mean_v_info[it->first]["all"][0]->mean_v, 0, 1);
            //int->Lane_Info



            //对各车道各基站进行拥堵评估  //better as word
            for (int pc_id=1;pc_id<+1;++pc_id)
            {
                lane_local_static_congestion_state[it->first]["pc_id"][pc_id] = lane_mean_v_info_by_all_v[it->first]["pc_id"][pc_id]->statistice_congestion;
            }
            //对各车道的拥堵热力图进行拥堵评估
//            for (auto &congestion_matrix_x:congestion_matrix.reshape({congestion_matrix.size()}))
//            {
//                lane_local_static_congestion_state[it->first]["meter"][congestion_matrix_x] =
//                        judge_congestion_level_by_mean_v(lane_info[it->first], used_lane_mean_v_info[it->first]["meter"][congestion_matrix_x]->mean_v, 0, 3);
//            }

            for (int i = 0; i < congestion_matrix.shape(0); ++i) {
                lane_local_static_congestion_state[it->first]["meter"][i] =
                        judge_congestion_level_by_mean_v(lane_info[it->first], used_lane_mean_v_info[it->first]["meter"][i]->mean_v, 0, 3);
            }
        }
    }
}
void Mean_Space_Headway_Info::get_mean_sh()
{
//    self.mean_sh = self.__max_sh if self.sum_num == 0 else self.sum_sh / self.sum_num

    if(Mean_Space_Headway_Info::sum_num == 0)
    {
        Mean_Space_Headway_Info::mean_sh = Mean_Space_Headway_Info::max_sh;
    }
    else
    {
        Mean_Space_Headway_Info::mean_sh = Mean_Space_Headway_Info::sum_sh / Mean_Space_Headway_Info::sum_num;
    }
}
void Mean_Space_Headway_Info::clear()
{
    Mean_Space_Headway_Info::sum_sh = 0;
    Mean_Space_Headway_Info::sum_num = 0;
}

//计算统计时间段内的平均车头间距 weikaifa
void Traffic_Flow::get_lane_mean_space_headway_info(const bool &statistics_flag) {
    for (const auto & lane : lane_info) {
        lane_mean_space_headway_info[lane.first]->update(cur_statistics[lane.first]["space_headway"]);  //yk:the accumulate() of update() is wrong
    }
    if (statistics_flag){
        //yk:10.12-13:15
        decltype(lane_mean_space_headway_info)::iterator it;
        for(it=lane_mean_space_headway_info.begin();it!=lane_mean_space_headway_info.end();it++)
        {

            lane_mean_space_headway_info[it->first]->get_mean_sh();


            lane_mean_space_headway_info[it->first]->clear();
        }
        //yk:10.12-14:15
//        ";
    }
}
//every loop is one pointframe
//异常事件检测主函数
void Traffic_Flow::detect_abnormal_event(const bool &statistics_flag) {
    del_flag = statistics_flag;
    long t_now = ops::getTimeStamp();
    // std::cout<<"start detect abnormal event-----"<<std::endl;
    /**""" 4.1 始化异常事件检测的信息"""*/
    init_abnormal_event_info();
    clear_id_info();

    //xcb add
    std::set<int> all_unhit_id_continue_stop_over_terminateline;
    for (auto & it : id_continue_stop_over_terminateline)
    {
        all_unhit_id_continue_stop_over_terminateline.insert(it.first);
    }

    std::set<int> all_unhit_id_continue_stop_over_terminateline_for_motor;
    for (auto & it : id_continue_stop_over_terminateline_for_motor)
    {
        all_unhit_id_continue_stop_over_terminateline_for_motor.insert(it.first);
    }

    std::set<int> all_unhit_id_continue_speeding_for_motor;
    for (auto & it : id_continue_speeding_info_for_motor) {
        all_unhit_id_continue_speeding_for_motor.insert(it.first);
    }
    std::set<int> all_unhit_id_continue_speeding_for_non_motor;
    for (auto & it : id_continue_speeding_info_for_non_motor) {
        all_unhit_id_continue_speeding_for_non_motor.insert(it.first);
    }
    std::set<int> all_unhit_id_continue_stroll;
    for (auto & it : id_continue_stroll_info) {
        all_unhit_id_continue_stroll.insert(it.first);
    }
    std::set<int> all_unhit_id_continue_occupy_dedicated_lane;
    for (auto & it : id_continue_occupy_dedicated_lane_info) {
        all_unhit_id_continue_occupy_dedicated_lane.insert(it.first);
    }
    std::set<int> all_unhit_id_continue_people_occupy_motor_lane;
    for (auto & it : id_continue_people_occupy_motor_lane_info) {
        all_unhit_id_continue_people_occupy_motor_lane.insert(it.first);
    }
    //xcb add
    std::set<int>all_unhit_id_continue_non_motor_occupy_motor_lane;
    for (auto & it : id_continue_non_motor_occupy_motor_lane_info){
        all_unhit_id_continue_non_motor_occupy_motor_lane.insert(it.first);
    }
    //xcb add
    std::set<int>all_unhit_id_continue_not_following_lane_guide;
    for (auto & it : id_continue_not_following_lane_guide){
        all_unhit_id_continue_not_following_lane_guide.insert(it.first);
    }

    std::set<int> all_unhit_id_continue_occupy_emergency_lane;
    for (auto & it : id_continue_occupy_emergency_lane_info) {
        all_unhit_id_continue_occupy_emergency_lane.insert(it.first);
    }
    std::set<int> all_unhit_id_continue_stop;
    for (auto & it : id_continue_stop_info) {
        all_unhit_id_continue_stop.insert(it.first);
    }
    std::set<int> all_unhit_id_continue_retrograde_for_motor;
    for (auto & it : id_continue_retrograde_info_for_motor) {
        all_unhit_id_continue_retrograde_for_motor.insert(it.first);
    }

    // add cjm 0902
    std::set<int> all_unhit_id_continue_occupy_bus_lane;
    for (auto & it : id_continue_occupy_bus_lane_info) {
        all_unhit_id_continue_occupy_bus_lane.insert(it.first);
    }


    std::set<int> all_unhit_id_continue_retrograde_for_non_motor;
    for (auto & it : id_continue_retrograde_info_for_non_motor) {
        all_unhit_id_continue_retrograde_for_non_motor.insert(it.first);
    }
    std::set<int> all_unhit_id_continue_change_lanes;
    for (auto & it : id_continue_change_lanes_info) {
        all_unhit_id_continue_change_lanes.insert(it.first);
    }

    std::set<int> all_unhit_construction;
    for(auto & it : construction_continue_info){
        all_unhit_construction.insert(it.first);
    }

    std::set<int> all_unhit_spills_coordinate_continue;
    std::set<int> all_unhit_congestion_continue;
    for (auto& it : congestion_continue_info){
        all_unhit_congestion_continue.insert(it.first);
    }
    for (auto & it : spills_coordinate_continue_info) {
        all_unhit_spills_coordinate_continue.insert(it.first);
    }

    std::set<int> all_unhit_big_car_in_dedicated_lane_continue;
    for (auto & it : big_car_in_dedicated_lane_continue_info) {
        all_unhit_big_car_in_dedicated_lane_continue.insert(it.first);
    }

    std::set<int> all_unhit_traverse_continue;
    for (auto & it : traverse_continue_info) {
        all_unhit_traverse_continue.insert(it.first);
    }

    std::map<int, int> lane_box_sorted = input_land_box_process();

    for (decltype(cur_id_info)::iterator it = cur_id_info.begin(); it != cur_id_info.end(); ++it)
    {
        //guiji
        // trajectory_evalute(const_cast<int &>(it->first), it->second);

        /**""" 4.3 超速"""*/ //chaosu

        if (event_sign["speeding"])
        {
            get_id_continue_speeding_info(const_cast<int &>(it->first), it->second, all_unhit_id_continue_speeding_for_motor,
                                          all_unhit_id_continue_speeding_for_non_motor);
        }
        /**""" 4.4 慢行"""*/      //--> error  //manxing
        if (event_sign["stroll"])
        {
            get_id_continue_stroll_info(const_cast<int &>(it->first), it->second, all_unhit_id_continue_stroll);
        }
        /**""" 4.6 非机动车或行人进入隧道"""*/
        if (event_sign["occupy_dedicated_lane"])
        {
            get_id_continue_occupy_dedicated_lane_info(const_cast<int &>(it->first), it->second,
                                                       all_unhit_id_continue_occupy_dedicated_lane,
                                                       all_unhit_id_continue_people_occupy_motor_lane, 
                                                       all_unhit_id_continue_non_motor_occupy_motor_lane);
        }
        /*xcb add车辆不按车道导向行驶*/
        if (event_sign["not_following_lane_guide"])
        {
            get_id_continue_not_following_lane_guide(const_cast<int &>(it->first),it->second,
                                                    all_unhit_id_continue_not_following_lane_guide);
        }
        /**""" 4.7 占用应急车道"""*/
        if (event_sign["occupy_emergency_area"])
        {
            get_id_continue_occupy_emergency_lane_info(const_cast<int &>(it->first), it->second, all_unhit_id_continue_occupy_emergency_lane);
        }
        /**""" 4.8 判断违停"""*/  //weiting
//         if (event_sign["illegal_stop"])
//         {
// //            if (it->first == 28355)
//             if (it->second->lane_no < 0)
//             {

//                 get_id_continue_stop_info(const_cast<int &>(it->first), it->second, all_unhit_id_continue_stop);
//             }

//         }

        /**""" 4.9 判断逆行"""*/
        // if (event_sign["retrograde"])
        // {
        //     get_id_continue_retrograde_info(const_cast<int &>(it->first), it->second, all_unhit_id_continue_retrograde_for_motor, all_unhit_id_continue_retrograde_for_non_motor);
        // }

        /**""" 4.10 判断变道"""*/ //biandao
        // if (event_sign["change_lanes"])
        // {
        //     get_id_continue_change_lanes_info(const_cast<int &>(it->first), it->second, all_unhit_id_continue_change_lanes);
        // }

        /**""" 4.11 抛洒物检测"""*/
        if (event_sign["spills_detect"])
        {
            get_spills_coordinate_continue_info(const_cast<int &>(it->first), it->second, all_unhit_spills_coordinate_continue);
        }
        /** 施工牌检测 **/
        if (event_sign["construction"])
        {
            get_construction_coordinate_continue_info(const_cast<int &>(it->first), it->second, all_unhit_construction);
        }

        if(event_sign["congestion"])
        {
            get_congestion_continue_info(const_cast<int &>(it->first), it->second,all_unhit_congestion_continue);
        }
        if(event_sign["large_vehicle_occupy_small_vehicle"])
        {
            get_big_car_in_dedicated_lane_continue_info(const_cast<int &>(it->first), it->second,all_unhit_big_car_in_dedicated_lane_continue);
        }
        if(event_sign["cross_diversion"])
        {
            get_traverse_continue_info(const_cast<int &>(it->first), it->second,all_unhit_traverse_continue);
        }
    }


    get_cur_pix_v_by_angle();
    if (event_sign["change_lanes"])
    {
        // std::cout<<"xcb---------change_lanes"<<std::endl;
        get_id_continue_change_lanes_info(all_unhit_id_continue_change_lanes);
    }

    if (event_sign["stop_over_terminate_line"])
    {
        // std::cout<<"xcb---------stop_over_terminate_line"<<std::endl;
        get_id_continue_stop_over_terminateline(all_unhit_id_continue_stop_over_terminateline, all_unhit_id_continue_stop_over_terminateline_for_motor);
    }
    if (event_sign["illegal_stop"])
    {
        // std::cout<<"xcb---------illegal_stop"<<std::endl;
        get_id_continue_stop_info(all_unhit_id_continue_stop);
    }
    if (event_sign["retrograde"])
    {
        // std::cout<<"xcb---------retrograde"<<std::endl;
        get_cur_pix_v();
        get_id_continue_retrograde_info(all_unhit_id_continue_retrograde_for_motor, all_unhit_id_continue_retrograde_for_non_motor);
    }
    //cjm add 0902 
    if (event_sign["occupy_bus_lane"])
    {
        // std::cout<<"xcb---------occupy_bus_lane"<<std::endl;
        get_id_continue_occupy_bus_lane_info(all_unhit_id_continue_occupy_bus_lane);
    }

    // according to continue info, judge all car


    judge_speeding(all_unhit_id_continue_speeding_for_motor, all_unhit_id_continue_speeding_for_non_motor);
    judge_stroll(all_unhit_id_continue_stroll, lane_box_sorted);
    detect_occupy_dedicated_lane(all_unhit_id_continue_occupy_dedicated_lane, all_unhit_id_continue_people_occupy_motor_lane, all_unhit_id_continue_non_motor_occupy_motor_lane);
    detect_car_not_following_lane_guide(all_unhit_id_continue_not_following_lane_guide);
    detect_occupy_emergency_lane(all_unhit_id_continue_occupy_emergency_lane);
    judge_illegal_stop(all_unhit_id_continue_stop, lane_box_sorted);
    judge_retrograde(all_unhit_id_continue_retrograde_for_motor, all_unhit_id_continue_retrograde_for_non_motor, lane_box_sorted);
    judge_change_lanes(all_unhit_id_continue_change_lanes, lane_box_sorted);
    spills_detect(all_unhit_spills_coordinate_continue);
    judge_congestion_detect(all_unhit_congestion_continue);
    judge_big_car_in_dedicated_lane(all_unhit_big_car_in_dedicated_lane_continue);
    judge_traverse(all_unhit_traverse_continue);
    detect_stop_over_terminateline(all_unhit_id_continue_stop_over_terminateline, all_unhit_id_continue_stop_over_terminateline_for_motor);
    detect_occupy_bus_lane(all_unhit_id_continue_occupy_bus_lane); // cjm 0902 add


    /**""" 4.12 事故检测"""*/
    if (event_sign["accident_detect"])
    {
        t_now = ops::getTimeStamp();
        accident_detect();
        long t_1 = ops::getTimeStamp();
//        use_time_traffic_fd << "_accident_detect:" << std::to_string(t_1 - t_now) << "ms\n";
    }

    /**""" 4.13 保存当前帧的id信息，用于下一帧的雷达bmp像素速度计算"""*/
    save_id_info();

}
//抛洒物检测
void Traffic_Flow::get_spills_coordinate_continue_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                       std::set<int> &all_unhit_spills_coordinate_continue) {
    if (info->l < 3 and std::find(classType->spills.begin(), classType->spills.end(), int (info->class_id)) != classType->spills.end())
    {
        double coincidence = 0.5;
        update_now_and_history(id, coincidence, spills_coordinate_continue_info, history_spills, all_unhit_spills_coordinate_continue);
    }
}
// 施工牌检测
void Traffic_Flow::get_construction_coordinate_continue_info(int &id, std::shared_ptr<Detect_Info> &info,
                                          std::set<int> &all_unhit_construction){
    // if (std::find(classType->construction.begin(), classType->construction.end(), int (info->class_id)) != classType->construction.end()){
    //     std::cout << "xcb-----------> get_construction_coordinate_continue_info  00000000000000" << std::endl;
    //     double coincidence = 0.5;
    //     update_now_and_history(id, coincidence, construction_continue_info, history_construction, all_unhit_construction);
    //     std::cout << "xcb-----------> get_construction_coordinate_continue_info  1111111111111" << std::endl;
    // }
}
//变道 biandao
/*xcb note
void Traffic_Flow::get_id_continue_change_lanes_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                     std::set<int> &all_unhit_id_continue_change_lanes) {
//    for (auto & it : lane_info) {
//        if (cur_lane_pc_car_num[it.first][1].front() > 0) {
//            cout << "ceshi:" << cur_lane_pc_car_num[it.first][1].front() << endl;
//        }
//    }


    if (cur_id_lane.count(id) and std::to_string(cur_id_lane[id]).size() == normal_lane_str_len and
        std::find(classType->motor_type.begin(), classType->motor_type.end(), int (info->class_id)) != classType->motor_type.end() and
        id_trajectory_eval.count(id))
    {
        std::map<int, std::vector<int>> lane_idx_dic;

//        int i = (id_dist_list[id].size() >= 20) ? (id_trajectory_eval.size() - 20) : (0);//yk:id_trajectory_eval.size() is wrong
//        int i_child = (id_dist_list[id].size() >= 20) ? (id_trajectory_eval.size() - 20) : (0);
        int i = (id_dist_list[id].size() >= 30) ? (id_trajectory_eval[id].size() - 30) : (0);
        int i_child = (id_dist_list[id].size() >= 30) ? (id_trajectory_eval[id].size() - 30) : (0);//yk:有待推敲
        for (; i < id_trajectory_eval[id].size(); ++i) {
            auto this_lane = id_trajectory_eval[id][i]->lane;
            if (this_lane <= 0)
            {
                continue;
            }
            if (!lane_idx_dic.count(this_lane))
            {
                lane_idx_dic[this_lane] = {i};
            }
            else
            {
                lane_idx_dic[this_lane].push_back(i);
            }
        }
        if (history_speed_info.count(id) and history_speed_info[id]->x_diff.size() >= 10)
        {
            if (id_dist_list[id].size() >= 20)
            {
                std::vector<std::vector<double>> pos_dist_list_np_change = id_dist_list[id];
                //pos_coin is the newest frame
                std::vector<double> pos_coin = id_dist_list[id][id_dist_list[id].size() - 1];
                //only save the dist vector
                std::vector<double> pos_dist_list(pos_dist_list_np_change.size());
                for (int j = 0; j < pos_dist_list_np_change.size(); ++j) {
                    pos_dist_list[j] = pos_dist_list_np_change[j][1];
                }
                std::vector<double> temp = pos_dist_list;
                std::sort(temp.begin(), temp.end());  //the ans of dist exist error.
                double pose_dist = (temp.size() % 2) ? (temp[temp.size() / 2]) : ((temp[temp.size() / 2 - 1] +  temp[temp.size() / 2]) / 2.0);
                double st_ed_gap = std::abs(pos_coin[1] - pose_dist);
                double gap = 1.0;   //yk:9.14-15:53 no wrong

                std::vector<std::vector<double>> pose_value;
                if (st_ed_gap >= gap and lane_idx_dic.size() > 1)   //yk:cannot come into
                {
                    if (float(pos_dist_list_np_change[0][2]) == 0)  //yk:wrong int should be revised to float
                    {
                        for (int j = 1; j < pos_dist_list_np_change.size(); ++j) {  //yk:missing an up-to-date
                            pose_value.push_back({pos_dist_list_np_change[j][2], pos_dist_list_np_change[j][3]});
                        }
                    }
                    else
                    {
                        for (int j = 0; j < pos_dist_list_np_change.size(); ++j) {
                            pose_value.push_back({pos_dist_list_np_change[j][2], pos_dist_list_np_change[j][3]});
                        }
                    }

                    //pos_mwan_coin = np.mean(pose_value, axis=0)
                    std::vector<double> pos_mwan_coin(pose_value[0].size());
                    for (int j = 0; j < pose_value.size(); ++j) {
                        for (int k = 0; k < pose_value[0].size(); ++k) {
                            pos_mwan_coin[k] += pose_value[j][k];
                        }
                    }
                    for (int j = 0; j < pos_mwan_coin.size(); ++j) {
                        pos_mwan_coin[j] /= pose_value.size();
                    }

                    double coincidence = (pos_mwan_coin[0] + pos_mwan_coin[1]) / 2;
                    if (int (info->class_id) == 2 or int (info->class_id) == 6 or int (info->class_id) == 8){
                        coincidence = coincidence * 0.9;
                    }
                    if (info->x >= -55.0){
                        coincidence = coincidence * 0.7;
                    }
                    if (info->v > 30 and info->v < 130)
                    {
                        coincidence = coincidence * (1 - (info->v - 30) / 100.0);
                    }

                    double car_l_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->length_std;
                    double pre_cnt_rate = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pre_cnt_rate;
                    double pos_anno_coin = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_coin;
                    double pos_anno_vel_coin = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_vel_coin;
                    double vel_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->vel_std;
                    double pose_x_diff_mean = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pose_x_diff_mean;

                    id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->coincidence[Abnormal_Class_Index::change_lanes]= coincidence;

                    if (coincidence < 0.5 or pos_anno_coin < 0.1 or pos_anno_vel_coin < 0.1 or vel_std > 3 or car_l_std > 1 or info->l < 3 or history_speed_info[id]->x_diff[history_speed_info[id]->x_diff.size() - 1] > 2)
                    {
                        coincidence = -100;
                    }
                    id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->final_coincidence[Abnormal_Class_Index::change_lanes] = coincidence;
                    update_now_and_history(id, coincidence, id_continue_change_lanes_info, history_change_lanes, all_unhit_id_continue_change_lanes);
                }
            }
        }
    }
}
*/
//

//xcb add
void Traffic_Flow::get_id_continue_stop_over_terminateline(std::set<int> &all_unhit_id_continue_stop_over_terminateline,std::set<int> &all_unhit_id_continue_stop_over_terminateline_for_motor)
{
    for (decltype(cur_id_info)::iterator iter = cur_id_info.begin(); iter != cur_id_info.end(); ++iter)
    {
        
        // if (iter->first == 740){
        //     std::cout<<"cjm000000000000000"<<history_speed_info[iter->first]->x_diff.size()<<std::endl;
        // }
        if (float(iter->second->x) <= forbid_over_lane[0] or
            float(iter->second->x) >= forbid_over_lane[1] or
            float(iter->second->y) <= forbid_over_lane[2] or
            float(iter->second->y) >= forbid_over_lane[3]
        ){
            continue;
        }
        if (history_speed_info[iter->first]->x_diff.size() < 8)
        {
            continue;
        }
        if (iter->second->v > 0.2)
        {
            continue;
        }

        if (cur_id_lane.count(iter->first) != 0 and cur_id_lane[iter->first] > 0)
        {
            continue;
        }
        if (std::find(classType->non_motor_type.begin(), classType->non_motor_type.end(), iter->second->class_id) != classType->non_motor_type.end()
         or std::find(classType->motor_type.begin(), classType->motor_type.end(), iter->second->class_id) != classType->motor_type.end())
        {

            if (id_last_lane.count(iter->first))
            {
                int lane = id_last_lane[iter->first][id_last_lane[iter->first].size() - 1]->lane;
                std::string lane_name = std::to_string(lane);
                if (lane_name.size() != normal_lane_str_len)
                {
                    continue;
                }
                //前點
                int fl_px = iter->second->front_left_px, fr_px = iter->second->front_right_px;
                int fl_py = iter->second->front_left_py, fr_py = iter->second->front_right_py;
                int f_center_px = (fl_px + fr_px) / 2;
                int f_center_py = (fl_py + fr_py) / 2;

                //後點
                int bl_px = iter->second->behind_left_px, br_px = iter->second->behind_right_px;
                int bl_py = iter->second->behind_left_py, br_py = iter->second->behind_right_py;
                int b_center_px = (bl_px + br_px) / 2;
                int b_center_py = (bl_py + br_py) / 2;

                // 3/4車身
                int three_quarters_mid_px = (b_center_px + iter->second->center_px)/2;
                int three_quarters_mid_py = (b_center_py + iter->second->center_py)/2;

                int three_quarters_left_px = (iter->second->front_left_px + 3 * iter->second->behind_left_px)/4;
                int three_quarters_left_py = (iter->second->front_left_py + 3 * iter->second->behind_left_py)/4;

                int three_quarters_right_px = (iter->second->front_right_px + 3 * iter->second->behind_right_px)/4;
                int three_quarters_right_py = (iter->second->front_right_py + 3 * iter->second->behind_right_py)/4;


                
                if (std::find(classType->motor_type.begin(), classType->motor_type.end(), iter->second->class_id) != classType->motor_type.end() and
                    (config_matrix(b_center_px, b_center_py, Config_Matrix_Index::CENTER_AREA) > 10 or 
                     config_matrix(f_center_px, f_center_py, Config_Matrix_Index::CENTER_AREA) > 10
                    ))
                {
                    continue;
                }

                // 車身３/4部分全部越線才算　越線
                bool stop_over_terminal = false;
                if (int(config_matrix(three_quarters_mid_px, three_quarters_mid_py, Config_Matrix_Index::LANE_NO)) > 0 and 
                    int(config_matrix(three_quarters_left_px, three_quarters_left_py, Config_Matrix_Index::LANE_NO)) > 0 and
                    int(config_matrix(three_quarters_right_px, three_quarters_right_py, Config_Matrix_Index::LANE_NO)) > 0)
                {
                    continue;
                }
                else{
                    stop_over_terminal = true;
                }

                if (config_matrix(f_center_px, f_center_py, Config_Matrix_Index::SIDEWALK) > 0 or
                    config_matrix(f_center_px, f_center_py, Config_Matrix_Index::CENTER_AREA) == 1 or 
                    stop_over_terminal)
                {
                    double coincidence = iter->second->coincidence;
                    if ((std::find(classType->non_motor_type.begin(), classType->non_motor_type.end(), iter->second->class_id) != classType->non_motor_type.end()))
                    {

                        update_now_and_history(const_cast<int &> (iter->first), coincidence, id_continue_stop_over_terminateline,
                                           history_stop_over_terminateline, all_unhit_id_continue_stop_over_terminateline);
                    }
                    else{
                        update_now_and_history(const_cast<int &> (iter->first), coincidence, id_continue_stop_over_terminateline_for_motor,
                                           history_stop_over_terminateline_for_motor, all_unhit_id_continue_stop_over_terminateline_for_motor);
                    }
                }
            }
            else
            {
                if (history_speed_info.count(iter->first) == 0)
                {
                    continue;
                }
                if (history_speed_info[iter->first]->x_diff.size() < 20)
                {
                    continue;
                }
                if (history_speed_info[iter->first]->moving_average_speed.back() > 0.5 or
                    history_speed_info[iter->first]->cal_speed.back() > 0.3)
                {
                    continue;
                }
                int center_px = iter->second->center_px;
                int center_py = iter->second->center_py;
                int front_px = (iter->second->front_left_px + iter->second->front_right_px) / 2;
                int front_py = (iter->second->front_left_py + iter->second->front_right_py) / 2;
                int behind_px = (iter->second->behind_left_px + iter->second->behind_right_px) / 2;
                int behind_py = (iter->second->behind_left_py + iter->second->behind_right_py) / 2;
                bool inArea = false;

                int behind_center_mid_px = (behind_px + iter->second->center_px)/2;
                int behind_center_mid_py = (behind_py + iter->second->center_py)/2;

                if (std::find(classType->motor_type.begin(), classType->motor_type.end(), iter->second->class_id) != classType->motor_type.end() and
                    config_matrix(behind_px, behind_py, Config_Matrix_Index::CENTER_AREA) > 10)
                {
                    continue;
                }

                if (int(config_matrix(behind_center_mid_px, behind_center_mid_py, Config_Matrix_Index::LANE_NO)) > 0)
                {
                    continue;
                }

                if ((int(config_matrix(front_px, front_py, Config_Matrix_Index::SIDEWALK)) > 0 or
                    int(config_matrix(front_px, front_py, Config_Matrix_Index::CENTER_AREA)) > 0 or 
                    int(config_matrix(front_px, front_py, Config_Matrix_Index::LANE_NO)) <= 0) 
                    and 
                    (int(config_matrix(behind_center_mid_px, behind_center_mid_py, Config_Matrix_Index::SIDEWALK)) > 0 or
                    int(config_matrix(behind_center_mid_px, behind_center_mid_py, Config_Matrix_Index::CENTER_AREA)) > 0 or 
                    int(config_matrix(behind_center_mid_px, behind_center_mid_py, Config_Matrix_Index::LANE_NO)) <= 0) )
                {
                    inArea = true;
                }
                else
                {
                    continue;
                }
                if (inArea)
                {
                    double coincidence = iter->second->coincidence;

                    if ((std::find(classType->non_motor_type.begin(), classType->non_motor_type.end(), iter->second->class_id) != classType->non_motor_type.end()))
                    {

                        update_now_and_history(const_cast<int &> (iter->first), coincidence, id_continue_stop_over_terminateline,
                                           history_stop_over_terminateline, all_unhit_id_continue_stop_over_terminateline);
                    }
                    else{
                        update_now_and_history(const_cast<int &> (iter->first), coincidence, id_continue_stop_over_terminateline_for_motor,
                                           history_stop_over_terminateline_for_motor, all_unhit_id_continue_stop_over_terminateline_for_motor);
                    }
                }
            }
        }
    }
}

//xcb add
void Traffic_Flow::get_id_continue_change_lanes_info(std::set<int> &all_unhit_id_continue_change_lanes)
{
    for (decltype(id_last_lane)::iterator it = id_last_lane.begin(); it != id_last_lane.end();)
    {
        if (cur_id_info.count(it->first) != 0)
        {
            ++it;
            continue;
        }
        if (cur_time - id_last_lane[it->first][id_last_lane[it->first].size() -1]->time > 10 * 1000)
        {   
            it = id_last_lane.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (decltype(cur_id_info)::iterator iter = cur_id_info.begin(); iter != cur_id_info.end(); ++iter)
    {
        std::vector<double> pos_coin = id_dist_list[iter->first][id_dist_list[iter->first].size() - 1];
        if (cur_id_lane.count(iter->first) == 0)
        {
            continue;
        }
        if (cur_id_lane[iter->first] <= 0){
            continue;
        }
        int lane = cur_id_lane[iter->first];
        int v_px = cur_id_pix_v.count(iter->first) == 0 ? 0 : cur_id_pix_v[iter->first][0];
        int v_py = cur_id_pix_v.count(iter->first) == 0 ? 0 : cur_id_pix_v[iter->first][1];
        double v_px_by_angle = cur_id_pix_v_by_angle[iter->first][0];
        double v_py_by_angle = cur_id_pix_v_by_angle[iter->first][1];
        if (id_last_lane.count(iter->first) == 0)
        {
            // id_last_lane[iter->first] = std::vector<std::shared_ptr<History_Continue_Lane_Info>>();
            id_last_lane[iter->first] = {std::make_shared<History_Continue_Lane_Info>(lane, iter->second, v_px, v_py,
                                                                                             v_px_by_angle, v_py_by_angle, cur_time)};
        }
        else
        {
            int index = iter->first;
            // std::cout<<"id_last_lane 2 size:"<<id_last_lane[2].size()<<std::endl;
            std::shared_ptr<History_Continue_Lane_Info> item;
            item.reset(new History_Continue_Lane_Info(lane, iter->second, v_px, v_py,v_px_by_angle, v_py_by_angle, cur_time));
            id_last_lane[index].push_back(item);
            if (id_last_lane[iter->first].size() >= 50)
            {
                id_last_lane[iter->first].erase(id_last_lane[iter->first].begin());
            }
        }
        if (std::find(classType->motor_type.begin(), classType->motor_type.end(), 
            iter->second->class_id) == classType->motor_type.end())
        {
            continue;
        }
        if(std::to_string(lane).length() != normal_lane_str_len)
        {
            id_dist_list.erase(iter->first);
            id_dist_list[iter->first] = {pos_coin};
            continue;
        }

        if (id_last_lane.count(iter->first))
        {
            if (std::find(classType->big_car.begin(), classType->big_car.end(), 
                iter->second->class_id) != classType->big_car.end())
            {
                lane = id_last_lane[iter->first][id_last_lane[iter->first].size() - 1]->lane;
                if (lane2zone.count(lane))
                {
                    zone2bus[lane2zone[lane]] = 1;
                }
            }
            std::map<int, std::vector<int>> lane_idx_dic;
            int start_pos = id_last_lane[iter->first].size() >= 20 ? id_last_lane[iter->first].size() - 20 : 0;
            std::vector<std::shared_ptr<History_Continue_Lane_Info>>new_history_continue_lane_info(id_last_lane[iter->first].begin() + start_pos, id_last_lane[iter->first].end());
            for (int i=0; i < new_history_continue_lane_info.size(); ++i)
            {
                int this_lane = new_history_continue_lane_info[i]->lane;
                if (lane_idx_dic.count(this_lane) == 0)
                {
                    lane_idx_dic[this_lane] = {i};
                }
                else
                {
                    lane_idx_dic[this_lane].push_back(i);
                }
            }
            if(history_speed_info.count(iter->first))
            {
                if (history_speed_info[iter->first]->x_diff.size() < 20)
                {
                    continue;
                }
                else
                {
                    double mean_a;
                    double sum_a = 0;
                    double mean_pose;
                    double sum_pose = 0;
                    for (auto acc : history_speed_info[iter->first]->acceleration)
                    {
                        sum_a += acc;
                    }
                    mean_a = sum_a / history_speed_info[iter->first]->acceleration.size();
                    int x_diff_pos = history_speed_info[iter->first]->x_diff.size() - 20;
                    for (auto it = history_speed_info[iter->first]->x_diff.begin() + x_diff_pos; it != history_speed_info[iter->first]->x_diff.end(); ++it)
                    {
                        sum_pose += *it;
                    }
                    mean_pose = sum_pose / 20.0;
                    double std_pos;
                    double sum_square_diff = 0.0;
                    for (auto it = history_speed_info[iter->first]->x_diff.begin() + x_diff_pos; it != history_speed_info[iter->first]->x_diff.end(); ++it)
                    {
                        double difference = *it - mean_pose;
                        sum_square_diff += std::pow(difference, 2);
                    }
                    std_pos = std::sqrt(sum_square_diff / 20.0);
                    if ((std::abs(mean_pose) > 0.2 and std_pos < 0.005) or (std::abs(mean_a) > 2))
                    {
                        continue;
                    }
                }

            }
            else
            {
                continue;
            }
            const std::vector<std::vector<double>>dist_temp = id_dist_list[iter->first];
            std::vector<double>flatVec;

            for (const auto& row : dist_temp)
            {
                flatVec.insert(flatVec.end(), row.begin(), row.end());
            }
            std::vector<std::size_t>shape = {std::size_t(dist_temp.size()),std::size_t(dist_temp[0].size())};
            xt::xarray<double> pos_dist_list_np = xt::adapt(flatVec, shape);
            xt::xarray<double> target_length_np = xt::view(pos_dist_list_np, xt::all(), 6);
            double length_var = xt::stddev(target_length_np)();
            double volum_car = iter->second->l;
            if ((pos_coin[2] < 0.1 and pos_coin[3] < 0.1 and lane_idx_dic.size() != 1) or volum_car < 3 or
                length_var > 1 or history_speed_info[iter->first]->x_diff[history_speed_info[iter->first]->x_diff.size() -1] <= -1)
            {
                id_dist_list.erase(iter->first);
                id_dist_list[iter->first] = {pos_coin};
                continue;
            }
            if (id_dist_list[iter->first].size() < 20)
            {
                continue;
            }
            xt::xarray<double>pos_dist_list = xt::view(pos_dist_list_np, xt::all(), 1);
            double pose_dist = xt::median(pos_dist_list);
            double st_ed_gap = std::abs(pos_coin[1] - pose_dist);
            double gap = 1.3;
            double change_lane_coincidence;
            if (st_ed_gap >= gap)
            {
                if (lane_idx_dic.size() == 1)
                {
                    continue;
                }
                xt::xarray<double> pose_value;
                if (pos_dist_list_np(0, 2) == 0)
                {
                    pose_value = xt::view(pos_dist_list_np, xt::range(1,_), xt::range(2,4));
                }
                else
                {
                    pose_value = xt::view(pos_dist_list_np, xt::all(), xt::range(2,4));
                }
                xt::xarray<double> pos_mwan_coin = xt::mean(pose_value, 0);
                change_lane_coincidence = (pos_mwan_coin(0) + pos_mwan_coin(1)) / 2;
                if (iter->second->class_id == 2 or iter->second->class_id == 6 or iter->second->class_id == 8)
                {
                    change_lane_coincidence = change_lane_coincidence * 0.9;
                }
                if (iter->second->v > 30 and iter->second->v < 130)
                {
                    double discret = 1 - (iter->second->v - 30) / 100;
                    change_lane_coincidence = change_lane_coincidence * discret;
                }
            }
            else
            {
                continue;
            }
            int non_motorway_flag = 0;
            std::vector<char> section_name;
            for (decltype(lane_idx_dic)::iterator i = lane_idx_dic.begin(); i != lane_idx_dic.end(); ++i)
            {
                if (non_motorway_flag == 0 and lane_info[i->first]->is_non_motorway)
                {
                    non_motorway_flag = 1;
                }
                if (std::find(section_name.begin(),section_name.end(), std::to_string(i->first)[0]) == section_name.end())
                {
                    section_name.push_back(std::to_string(i->first)[0]);
                }
            }
            int intersection_flag = section_name.size() > 1 ? 1 : 0;
            if(cur_id_info.count(iter->first) and config_matrix(cur_id_info[iter->first]->center_px, cur_id_info[iter->first]->center_py, Config_Matrix_Index::FORBID_CROSS_LINE) < 1)
            {
                continue;
            }
            if (non_motorway_flag == 0 and intersection_flag == 0)
            {
                update_now_and_history(const_cast<int &>(iter->first), change_lane_coincidence, id_continue_change_lanes_info,
                                       history_change_lanes, all_unhit_id_continue_change_lanes, 0);
            }
        }
    }
}
//yongdu
void Traffic_Flow::get_congestion_continue_info(int &id, std::shared_ptr<Detect_Info> &info,std::set<int> &all_unhit_congestion_continue){
    for (auto & it : lane_info)
    {
        int num = lane_mean_v_info_by_all_v[it.first]["pc_id"][1]->cur_num;
        int mean_v = lane_mean_v_info_by_all_v[it.first]["pc_id"][1]->cur_mean_v;
        if(num >= 2 and mean_v <=50){
            static int out_px = 0;
            static int out_py = 0;

            if (it.first == 110001){
                out_px = 293;
                out_py = 7338;
            }
            if (it.first == 110002){
                out_px = 360;
                out_py = 7338;
            }
            if (it.first == 110003){
                out_px = 325;
                out_py = 7343;
            }
            if (it.first == 110004){
                out_px = 259;
                out_py = 7362;
            }
            else{
                continue;
            }

            bool signal = 0;
            std::vector<int> tmp = cur_lane_pc_car_num[it.first][1];
//            if (std::find(tmp.begin(), tmp.end(), int (id)) == tmp.end()){
//                continue;
//            }
            if(id!=tmp[0]){
                continue;
            }
            std::sort(tmp.begin(), tmp.end(), [this](int a, int b) {
                return std::pow(cur_id_info[a]->center_px - out_px, 2) + std::pow(cur_id_info[a]->center_py - out_py, 2) <
                       std::pow(cur_id_info[b]->center_px - out_px, 2) + std::pow(cur_id_info[b]->center_py - out_py, 2);
            });


            double coincidence = 0.5;
            update_now_and_history(tmp[0],coincidence,congestion_continue_info,history_congestion,all_unhit_congestion_continue);
        }

    }
}


void Traffic_Flow::get_traverse_continue_info(int &id, std::shared_ptr<Detect_Info> &info, std::set<int> &all_unhit_traverse_continue){
    if (info->l > 3 and std::find(classType->motor_type.begin(), classType->motor_type.end(), int (info->class_id)) == classType->motor_type.end() and \
    100 > info->x and info->x > 80 and cur_id_lane.count(id) and cur_id_lane[id] == 110003)
    {
//        if ()
        double coincidence = 0.5;
        update_now_and_history(id,coincidence,traverse_continue_info,history_traverse,all_unhit_traverse_continue);
    }

}


void Traffic_Flow::get_big_car_in_dedicated_lane_continue_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                               std::set<int> &all_unhit_big_car_continue){
    if ((info->l > 5 or std::find(classType->big_car.begin(), classType->big_car.end(), int (info->class_id)) == classType->big_car.end())\
    and cur_id_lane.count(id) and cur_id_lane[id] == 110003) {

        double coincidence = 0.5;
        update_now_and_history(id, coincidence, big_car_in_dedicated_lane_continue_info,
                               history_big_car_in_dedicated_lane, all_unhit_big_car_continue);
    }
}

void Traffic_Flow::get_cur_pix_v()
{
    if (last_id_info_0.size() != 0)
    {
        for (decltype(cur_id_info)::iterator iter=cur_id_info.begin(); iter != cur_id_info.end(); ++iter)
        {
            if (last_id_info_0.back().count(iter->first) != 0)
            {
                cur_id_pix_v[iter->first] = {iter->second->center_px - last_id_info_0.back()[iter->first]->center_px,
                                             iter->second->center_py - last_id_info_0.back()[iter->first]->center_py};
            }
        }
    }
}

void Traffic_Flow::get_cur_pix_v_by_angle()
{
    for (decltype(cur_id_info)::iterator iter=cur_id_info.begin(); iter != cur_id_info.end(); ++iter)
    {
        double theta = 1.5 * M_PI - iter->second->angle;
        cur_id_pix_v_by_angle[iter->first] = {std::cos(theta), std::sin(theta)};
    }
}

//nixing
/*xcb note
void Traffic_Flow::get_id_continue_retrograde_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                   std::set<int> &all_unhit_id_continue_retrograde_for_motor,
                                                   std::set<int> &all_unhit_id_continue_retrograde_for_non_motor)
{
    if (cur_id_lane.count(id) and id_trajectory_eval.count(id) and history_speed_info.count(id)){
        double car_v = history_speed_info[id]->moving_average_speed[history_speed_info[id]->moving_average_speed.size() - 1];
        if (history_speed_info[id]->y_diff.size() >= 10){
            double this_v_px = history_speed_info[id]->x[history_speed_info[id]->x.size() - 1] - history_speed_info[id]->x[history_speed_info[id]->x.size() - 2];
            double this_v_py = history_speed_info[id]->y[history_speed_info[id]->y.size() - 1] - history_speed_info[id]->y[history_speed_info[id]->y.size() - 2];
            double this_pc_angle = base_station_pos[int (info->pc_id)][3] * M_PI / 180.0;
            double this_pc_x = std::cos(-1 * this_pc_angle);
            double this_pc_y = std::sin(-1 * this_pc_angle);
            double cos_angle = this_v_px * this_pc_x + this_v_py * this_pc_y;
            int lane = cur_id_lane[id];
            int direct = cur_id_info[id]->v_director;
            double vx = cur_id_info[id]->v * direct;
            if (vx < 0){
                double pose_x_mean = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pose_x_diff_mean;
                double re_pos_coin;
                if (id_dist_list[id].size())
                {
                    re_pos_coin = std::exp(-1 * std::abs(car_v * id_dist_list[id][id_dist_list[id].size() - 1][4] - pose_x_mean));
                }
                else
                {
                    re_pos_coin = 1;
                }
                double coincidence = history_speed_info[id]->normal_rate * re_pos_coin;
                if (cos_angle < 0){
                    coincidence = coincidence * 0.7;
                }

                double car_l_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->length_std;
                double pre_cnt_rate = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pre_cnt_rate;
                double pos_anno_coin = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_coin;
                double pos_anno_vel_coin = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_vel_coin;
                double vel_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->vel_std;
                double pose_x_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pose_x_std;

                if (std::find(classType->motor_type.begin(), classType->motor_type.end(), int (info->class_id)) != classType->motor_type.end() and info->l >= 3)
                {
                    id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->coincidence[Abnormal_Class_Index::retrograde_for_motor] = coincidence;
                    if (coincidence < 0.5 or car_l_std > 1 or pre_cnt_rate > 0.3 or pos_anno_vel_coin < 0.7 or pos_anno_coin < 0.7)
                    {
                        coincidence = -100;
                    }
                    id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->final_coincidence[Abnormal_Class_Index::retrograde_for_motor] = coincidence;
                    update_now_and_history(id, coincidence, id_continue_retrograde_info_for_motor, history_retrograde_for_motor, all_unhit_id_continue_retrograde_for_motor);
                }
                else if (std::find(classType->non_motor_type.begin(), classType->non_motor_type.end(), int (info->class_id)) != classType->non_motor_type.end() and info->l <= 3){
                    id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->coincidence[Abnormal_Class_Index::retrograde_for_non_motor] = coincidence;
                    if (coincidence < 0.5 or car_l_std > 1 or pre_cnt_rate > 0.3 or pos_anno_vel_coin < 0.7 or pos_anno_coin < 0.7)
                    {
                        coincidence = -100;
                    }
                    id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->final_coincidence[Abnormal_Class_Index::retrograde_for_non_motor] = coincidence;
                    update_now_and_history(id, coincidence, id_continue_retrograde_info_for_non_motor, history_retrograde_for_non_motor,
                                           all_unhit_id_continue_retrograde_for_non_motor);

                }
            }
        }
    }

}
*/
//xcb add
// void Traffic_Flow::get_id_continue_retrograde_info(int &id, std::shared_ptr<Detect_Info> &info,
//                                                    std::set<int> &all_unhit_id_continue_retrograde_for_motor,
//                                                    std::set<int> &all_unhit_id_continue_retrograde_for_non_motor)
// {
//     if(history_speed_info[id]->v.size() >= 3 and cur_id_lane.count(id) != 0)
//     {
//         if(history_speed_info.count(id) != 0 and history_speed_info[id]->x_diff.size() >= 10)
//         {
//             std::vector<double>car_l = history_speed_info[id]->car_l;
//             double car_l_sum = std::accumulate(car_l.begin(), car_l.end(), 0);
//             double car_l_mean = car_l_sum / car_l.size();
//             double accum = 0.0;
//             std::for_each(std::begin(car_l), std::end(car_l), [&](const double d){
//                 accum += (d - car_l_mean) * (d - car_l_mean);
//             });
//             double car_l_std = std::sqrt(accum / (car_l.size()));
//             if (car_l_std <= 1)
//             {
//                 double px = history_speed_info[id]->px[history_speed_info[id]->px.size() - 1] -
//                             history_speed_info[id]->px[history_speed_info[id]->px.size() - 2];
//                 double py = history_speed_info[id]->py[history_speed_info[id]->px.size() - 1] -
//                             history_speed_info[id]->py[history_speed_info[id]->px.size() - 2];
//                 if(px != 0 and py != 0)
//                 {
//                     double this_v_px = px / std::sqrt(std::pow(px, 2) + std::pow(py, 2));
//                     double this_v_py = py / std::sqrt(std::pow(px, 2) + std::pow(py, 2));
//                     int lane = cur_id_lane[id];
//                     double this_lane_v_px = lane_info[lane]->v_px;
//                     double this_lane_v_py = lane_info[lane]->v_py;
//                     double cos_angle = this_v_px * this_lane_v_px + this_v_py * this_lane_v_py;

//                     double coincidence = 0;
//                     if (cos_angle < 0)
//                     {
//                         coincidence += 50;
//                         if (cur_id_info[id]->coincidence > 0.4 and cur_id_info[id]->coincidence < 0.7)
//                         {
//                             coincidence += 20;
//                         }
//                         if (cur_id_info[id]->coincidence > 0.7)
//                         {
//                             coincidence += 30;
//                         }
//                         if (std::find(classType->non_motor_type.begin(), classType->non_motor_type.end(), int(cur_id_info[id]->class_id)) !=
//                             classType->non_motor_type.end())
//                         {
//                             coincidence += 10;
//                             if (lane_info[lane]->is_non_motorway == 1)
//                             {
//                                 if (history_speed_info[id]->x[0] - history_speed_info[id]->x[history_speed_info[id]->x.size() -1] > 0 and
//                                     std::sqrt(std::pow(px, 2) + std::pow(py, 2)) > 2)
//                                 {
//                                     coincidence += 10;
//                                 }
//                             }
//                             coincidence /= 100;
//                             update_now_and_history(id, coincidence, id_continue_retrograde_info_for_non_motor,
//                                                    history_retrograde_for_non_motor, all_unhit_id_continue_retrograde_for_non_motor);
//                         }
//                         if (std::find(classType->motor_type.begin(), classType->motor_type.end(), int(cur_id_info[id]->class_id)) !=
//                             classType->motor_type.end())
//                         {
//                             coincidence += 20;
//                             // if (history_speed_info[id]->x[0] - history_speed_info[id]->x[history_speed_info[id]->x.size() -1] > 0 and
//                             //         std::sqrt(std::pow(px, 2) + std::pow(py, 2)) > 2)
//                             //     {
//                             //         coincidence += 20;
//                             //     }
//                             update_now_and_history(id, coincidence, id_continue_retrograde_info_for_motor,
//                                                    history_retrograde_for_motor, all_unhit_id_continue_retrograde_for_motor);
//                         }
//                     }
//                 }
//             }
//         }
//     }
// }

void Traffic_Flow::get_id_continue_retrograde_info(std::set<int> &all_unhit_id_continue_retrograde_for_motor,
                                                   std::set<int> &all_unhit_id_continue_retrograde_for_non_motor)
{
    for (decltype(cur_id_info)::iterator iter = cur_id_info.begin(); iter != cur_id_info.end(); ++iter)
    {
        if (cur_id_lane.count(iter->first) == 0)
        {
            continue;
        }

        if (cur_id_lane[iter->first] <= 0){
            continue;
        }
        if(history_speed_info.count(iter->first) == 0)
        {
            continue;
        }
        if (cur_id_info[iter->first]->v < 1.5){
            continue;
        }

        std::vector<double>car_l = history_speed_info[iter->first]->car_l;
        double car_l_sum = std::accumulate(car_l.begin(), car_l.end(), 0);
        double car_l_mean = car_l_sum / car_l.size();
        double accum = 0.0;
        std::for_each(std::begin(car_l), std::end(car_l), [&](const double d){
            accum += (d - car_l_mean) * (d - car_l_mean);
        });
        double car_l_std = std::sqrt(accum / (car_l.size()));
        if (car_l_std > 1.0)
        {
            continue;
        }

        double x = history_speed_info[iter->first]->x[history_speed_info[iter->first]->x.size() - 1] -
                    history_speed_info[iter->first]->x[history_speed_info[iter->first]->x.size() - 8];
        double y = history_speed_info[iter->first]->y[history_speed_info[iter->first]->x.size() - 1] -
                    history_speed_info[iter->first]->y[history_speed_info[iter->first]->x.size() - 8];
        
        if(std::abs(x) <= 2 and std::abs(y) <= 2)
        {
            continue;
        }

        double px = history_speed_info[iter->first]->px[history_speed_info[iter->first]->px.size() - 1] -
                    history_speed_info[iter->first]->px[history_speed_info[iter->first]->px.size() - 8];
        double py = history_speed_info[iter->first]->py[history_speed_info[iter->first]->px.size() - 1] -
                    history_speed_info[iter->first]->py[history_speed_info[iter->first]->px.size() - 8];

        double this_v_px = px / std::sqrt(std::pow(px, 2) + std::pow(py, 2));
        double this_v_py = py / std::sqrt(std::pow(px, 2) + std::pow(py, 2));
        int lane = cur_id_lane[iter->first];
        double this_lane_v_px = lane_info[lane]->v_px;
        double this_lane_v_py = lane_info[lane]->v_py;
        double cos_angle = this_v_px * this_lane_v_px + this_v_py * this_lane_v_py;

        double coincidence = 0;
        if (cos_angle < -0.7)
        {
            coincidence += 50;
            if (cur_id_info[iter->first]->coincidence > 0.4 and cur_id_info[iter->first]->coincidence < 0.7)
            {
                coincidence += 20;
            }
            if (cur_id_info[iter->first]->coincidence > 0.7)
            {
                coincidence += 30;
            }
            if (std::find(classType->non_motor_type.begin(), classType->non_motor_type.end(), int(cur_id_info[iter->first]->class_id)) !=
                classType->non_motor_type.end())
            {
                coincidence += 10;
                if (lane_info[lane]->is_non_motorway == 1)
                {
                    if (history_speed_info[iter->first]->x[0] - history_speed_info[iter->first]->x[history_speed_info[iter->first]->x.size() -1] > 0 and
                        std::sqrt(std::pow(px, 2) + std::pow(py, 2)) > 2)
                    {
                        coincidence += 10;
                    }
                }
                coincidence /= 100;
                update_now_and_history(const_cast<int &>(iter->first), coincidence, id_continue_retrograde_info_for_non_motor,
                                    history_retrograde_for_non_motor, all_unhit_id_continue_retrograde_for_non_motor);
            }
            if (std::find(classType->motor_type.begin(), classType->motor_type.end(), int(cur_id_info[iter->first]->class_id)) !=
                classType->motor_type.end())
            {
                coincidence += 20;
                // if (history_speed_info[id]->x[0] - history_speed_info[id]->x[history_speed_info[id]->x.size() -1] > 0 and
                //         std::sqrt(std::pow(px, 2) + std::pow(py, 2)) > 2)
                //     {
                //         coincidence += 20;
                //     }
                coincidence /= 100;
                if(std::abs(x) <= 3 and std::abs(y) <= 3)
                {
                    continue;
                }
                if (cos_angle > -0.8){
                    continue;
                }
                update_now_and_history(const_cast<int &>(iter->first), coincidence, id_continue_retrograde_info_for_motor,
                                    history_retrograde_for_motor, all_unhit_id_continue_retrograde_for_motor);
            }
        }
    }

}


// cjm 0902 add
void Traffic_Flow::get_id_continue_occupy_bus_lane_info(std::set<int> &all_unhit_id_continue_occupy_bus_lane)
{
    // 连续占用公交车道的次数、时间，记录历史异常事件
    for (decltype(cur_id_info)::iterator iter = cur_id_info.begin(); iter != cur_id_info.end(); ++iter)
    {
        double v_len = history_speed_info[iter->first]->x_diff.size();
        if (v_len < 3)
        {
            continue;
        }
        int c_px = iter->second->center_px;
        int c_py = iter->second->center_py;
        if (config_matrix(c_px, c_py, Config_Matrix_Index::CENTER_AREA) >= 1 or
            config_matrix(c_px, c_py, Config_Matrix_Index::SIDEWALK) >= 1)
        {
            if (id_continue_occupy_bus_lane_info.count(iter->first))
            {
                id_continue_occupy_bus_lane_info.erase(iter->first);

            }
            continue;
        }
        if (cur_id_lane.count(iter->first) == 0)
        {
            continue;
        }
        if (cur_id_lane[iter->first] <= 0){
            continue;
        }
        // if (cur_id_lane.count(iter->first) == 0)
        if (std::to_string(cur_id_lane[iter->first]).length() != normal_lane_str_len)
        {
            continue;
        }
        double coincidence = id_dist_list[iter->first][id_dist_list[iter->first].size()-1][3] * iter->second->coincidence;
        int lane = cur_id_lane[iter->first];
        if (std::find(classType->bus.begin(), classType->bus.end(), int(iter->second->class_id)) == classType->bus.end() and 
            lane_info[lane]->is_bus_lane)
        {
            update_now_and_history(const_cast<int &>(iter->first), coincidence, id_continue_occupy_bus_lane_info,
                                        history_occupy_bus_lane,
                                        all_unhit_id_continue_occupy_bus_lane);
        }  
    }
}


//xcb add违停
void Traffic_Flow::get_id_continue_stop_info(std::set<int> &all_unhit_id_continue_stop)
{
    for (decltype(cur_id_info)::iterator iter = cur_id_info.begin(); iter != cur_id_info.end(); ++iter)
    {   
        // 不在車道區不是違規停車，是越線停車
        if (std::find(classType->motor_type.begin(), classType->motor_type.end(), iter->second->class_id) == classType->motor_type.end() or cur_id_lane.count(iter->first) == 0) 
        {
            continue;
        }
        if ( cur_id_lane[iter->first] <= 0){
            continue;
        }

        // cur_time
        int lane = cur_id_lane[iter->first];
        // std::cout<<std::to_string(config_matrix(iter->second->center_px, iter->second->center_py, Config_Matrix_Index::FORBID_CROSS_LINE))<<std::endl;
        if (config_matrix(iter->second->center_px, iter->second->center_py, Config_Matrix_Index::FORBID_CROSS_LINE) == 1 and
            std::to_string(lane)[1] == '1')
        {
            continue;
        }
        std::string queue_id = "queue_id";
        if (cur_statistics[lane].count(queue_id) !=0 and std::find(cur_statistics[lane][queue_id].begin(),
            cur_statistics[lane][queue_id].end(), static_cast<float>(iter->first)) != cur_statistics[lane][queue_id].end() and (!lane_info[lane]->is_non_motorway))
        {
            continue;
        }

        if (history_speed_info[iter->first]->x_diff.size() < 10)
        {
            continue;
        }
        int px = iter->second->center_px, py = iter->second->center_py;
        if (config_matrix(px, py, Config_Matrix_Index::CENTER_AREA) > 10 and
            config_matrix(px, py, Config_Matrix_Index::SIDEWALK) == 0)
        {
            continue;
        }
        if (cur_id_info[iter->first]->v > 1.5)
        {
            continue;
        }
        double veh_x_diff = std::abs(history_speed_info[iter->first]->x[0] - history_speed_info[iter->first]->x[history_speed_info[iter->first]->x_diff.size()-8]);
        double veh_y_diff = std::abs(history_speed_info[iter->first]->y[0] - history_speed_info[iter->first]->y[history_speed_info[iter->first]->x_diff.size()-8]);
        double veh_location_begin = std::sqrt(std::pow(history_speed_info[iter->first]->x[0], 2) + std::pow(history_speed_info[iter->first]->y[0], 2));
        double veh_location_end = std::sqrt(std::pow(history_speed_info[iter->first]->x[history_speed_info[iter->first]->x_diff.size()-8], 2) + 
                                  std::pow(history_speed_info[iter->first]->y.back(), 2));
        double veh_location_diff = std::abs(veh_location_begin - veh_location_end);
        double max_location_diff = std::max(std::max(veh_x_diff, veh_y_diff), veh_location_diff);

        if (max_location_diff > 1.08)
        {
            continue;
        }

        double coincidence = cur_id_info[iter->first]->coincidence;
        if (coincidence <= 0.5)
        {
            coincidence += 0.4;
        }
        if (coincidence > 0.5 and coincidence <= 0.7)
        {
            coincidence += 0.2;
        }

        update_now_and_history(const_cast<int &>(iter->first), coincidence, id_continue_stop_info,
                               history_illegal_stop, all_unhit_id_continue_stop);

        if (history_illegal_stop.count(iter->first))
        {
            for (decltype(history_illegal_stop)::iterator it = history_illegal_stop.begin(); it != history_illegal_stop.end(); ++it)
            {
                if (it->first == iter->first)
                {
                    continue;
                }
                else
                {
                    double dist = std::sqrt(std::pow(it->second.back()->x - iter->second->x, 2) + std::pow(it->second.back()->y - iter->second->y, 2));
                    if (dist <= 0.5)
                    {
                        history_illegal_stop[iter->first].back()->event_no = it->second.back()->event_no;
                    }
                }
            }
        }
    }
}

// void Traffic_Flow::get_id_continue_stop_info(int &id, std::shared_ptr<Detect_Info> &info,
//                                              std::set<int> &all_unhit_id_continue_stop) {
//     int px = info->center_px, py = info->center_py;
// //    if (config_matrix(px, py, Config_Matrix_Index::NO_PARK) == 1){
// //        cout<<"yse!!!"<<endl;
// //    }

// //    if (history_speed_info.count(id) and id_trajectory_eval.count(id) and cur_id_lane.count(id))
//     if (history_speed_info.count(id) and id_trajectory_eval.count(id)){
//         double car_v = history_speed_info[id]->moving_average_speed[history_speed_info[id]->moving_average_speed.size() - 1];
//         if (car_v < 0.5 and history_speed_info[id]->x_diff.size() >= 10){
//             int k = 0;
//             for (auto & val : history_speed_info[id]->x_diff){
//                 if (val > 0){
//                     k += 1;
//                 }
//             }
//             double normal_rate = float(k) / float(history_speed_info[id]->x_diff.size());  //yk:wrong
//             k = 0;
//             for (auto & val : history_speed_info[id]->x_diff){
//                 if (val == 0){
//                     k += 1;
//                 }
//             }
//             double zero_rate = float(k) / history_speed_info[id]->x_diff.size();//yk:wrong
//             double d_rate = (0.5 - (std::abs(normal_rate - 0.5))) / 0.5;//yk:wrong
//             double pose_x_mean = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pose_x_diff_mean;//yk:not same with python
//             double coincidence = (d_rate + zero_rate) * std::exp(std::abs(pose_x_mean) * -1);

//             double car_l_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->length_std;
//             double pre_cnt_rate = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pre_cnt_rate;
//             double pos_anno_coin = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_coin;
//             double pos_anno_vel_coin = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_vel_coin;
//             double vel_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->vel_std;//yk:not same with python
//             double pose_x_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pose_x_std;//yk:not same with python

//             id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->coincidence[Abnormal_Class_Index::illegal_stop] = coincidence;

//             if (vel_std > 1 or coincidence < 0.5 or car_l_std > 1 or pre_cnt_rate > 0.3 or pos_anno_vel_coin < 0.7
//                 or pos_anno_coin < 0.7 or pose_x_mean > 1 or info->l < 3 or
//                 std::find(classType->motor_type.begin(), classType->motor_type.end(), int (info->class_id)) != classType->motor_type.end()){
//                 coincidence = -100;
//             }
//             id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->final_coincidence[Abnormal_Class_Index::illegal_stop] = coincidence;
//             update_now_and_history(id, coincidence, id_continue_stop_info, history_illegal_stop, all_unhit_id_continue_stop);
// //            std::cout<<"illegal_stop_hitcount_1"<<std::endl;//yk:no wrong9.22-9:46
//             if (history_illegal_stop.count(id)){
//                 for (const auto & it : history_illegal_stop){
//                     if (it.first == id){
//                         continue; //yk:no wrong 9.21-18:15
//                     }
//                     else
//                     {
//                         double dist = std::sqrt(std::pow(it.second[it.second.size() - 1]->x - info->x, 2) + std::pow(it.second[it.second.size() - 1]->y - info->y, 2));
//                         if (dist <= 0.5){
//                             history_illegal_stop[id][history_illegal_stop[id].size() - 1]->event_no = it.second[it.second.size() - 1]->event_no;
//                         }
//                     }
//                 }
//             }
//         }
//     }


// }

//占用应急车道
void Traffic_Flow::get_id_continue_occupy_emergency_lane_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                              std::set<int> &all_unhit_id_continue_occupy_emergency_lane) {
    if (config_matrix(int (info->center_px), int (info->center_py), Config_Matrix_Index::EMERGENCY_AREA) == 1){
        double car_l_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->length_std;
        double pre_cnt_rate = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pre_cnt_rate;
        double pos_anno_coin = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_coin;
        double pos_anno_vel_coin = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_vel_coin;
        double vel_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->vel_std;
        double pose_x_std = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pose_x_std;
        double pose_x_mean = id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pose_x_diff_mean;
        double coincidence = id_dist_list[id][id_dist_list[id].size() - 1][3];

        coincidence = coincidence * (1 - pose_x_std);
        id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->coincidence[Abnormal_Class_Index::illegal_stop] = coincidence;
        if (info->l < 3 or vel_std > 1 or coincidence < 0.5 or car_l_std > 1 or pre_cnt_rate > 0.3 or
            pos_anno_vel_coin < 0.7 or pos_anno_coin < 0.7 or pose_x_mean > 1 or pose_x_std > 1){
            coincidence = -100;
        }
        id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->final_coincidence[Abnormal_Class_Index::illegal_stop] = coincidence;
        update_now_and_history(id, coincidence, id_continue_occupy_emergency_lane_info, history_occupy_emergency_lane, all_unhit_id_continue_occupy_emergency_lane);
    }
}
//feijidongche
void Traffic_Flow::get_id_continue_occupy_dedicated_lane_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                              std::set<int> &all_unhit_id_continue_occupy_dedicated_lane,
                                                              std::set<int> &all_unhit_id_continue_people_occupy_motor_lane,
                                                              std::set<int> &all_unhit_id_continue_non_motor_occupy_motor_lane) {

    int in_lane = cur_id_lane.count(id);
    // double id_dist = id_dist_list[id][id_dist_list[id].size()-1][3];
    // double coincidence = info->coincidence * id_dist; //id_dist_list not complete, zan bu shi yong
    // // double coincidence = info->coincidence;
    if(history_speed_info.count(id) > 0 && in_lane > 0)
    {
        // cjm 0830 move from out into if inside
        double id_dist = id_dist_list[id][id_dist_list[id].size()-1][3];
        double coincidence = info->coincidence * id_dist; //id_dist_list not complete, zan bu shi yong
        // double coincidence = info->coincidence;

        std::string cur_lane = std::to_string(cur_id_lane[id]);
        int lane = atoi(cur_lane.c_str());
        if( cur_lane.length() == normal_lane_str_len && info->coincidence >= 0.3 && coincidence >= 0.5)
        {
            if (std::find(classType->motor_type.begin(), classType->motor_type.end(), int(info->class_id)) != classType->motor_type.end() and
                lane_info[lane]->is_non_motorway)
            {   
                if (cur_id_info[id]->l>3 and cur_id_info[id]->l * cur_id_info[id]->w >5){
                    update_now_and_history(id, coincidence, id_continue_occupy_dedicated_lane_info,history_occupy_dedicated_lane,
                                        all_unhit_id_continue_occupy_dedicated_lane);
                }
            }

            if (std::find(classType->people.begin(), classType->people.end(), int(info->class_id)) != classType->people.end() and
                !(lane_info[lane]->is_non_motorway))
            {
                if (history_speed_info.count(id) !=0)
                {
                    if (history_speed_info[id]->x_diff.size() >= 5)
                    {
                        double v_people = history_speed_info[id]->moving_average_speed[history_speed_info[id]->moving_average_speed.size()-1];
                        if(v_people < 2.5)
                        {
                            update_now_and_history(id, coincidence, id_continue_people_occupy_motor_lane_info, history_people_occupy_motor_lane,
                                            all_unhit_id_continue_people_occupy_motor_lane);
                        }
                    }
                }
            }
            
            if (std::find(classType->non_motor_type.begin(), classType->non_motor_type.end(), int(info->class_id)) != classType->non_motor_type.end() and
                lane_info[lane]->is_non_motorway != 1)
            {
                if (history_speed_info[id]->x_diff.size() >= 5)
                {
                    if (id_dist_list.count(id)){
                        if (id_dist_list[id][id_dist_list[id].size() - 1][1] + 0.001 < 1.5){

                            int occupy_flag = true;
                            // 目標類別

                            for (int i = history_speed_info[id]->x_diff.size()-1; i > history_speed_info[id]->x_diff.size()-6; i--){
                                if (std::find(classType->non_motor_type.begin(),classType->non_motor_type.end(),int(history_speed_info[id]->label[i])) == 
                                    classType->non_motor_type.end()){
                                    occupy_flag = false;
                                    break;
                                }
                            }
                            if (occupy_flag){
                                update_now_and_history(id, coincidence, id_continue_non_motor_occupy_motor_lane_info, history_non_motor_occupy_motor_lane, 
                                                all_unhit_id_continue_non_motor_occupy_motor_lane);
                            }
                        }
                    }
                }
            }
        }
    }
}

//xcb add
void Traffic_Flow::get_id_continue_not_following_lane_guide(int &id, std::shared_ptr<Detect_Info> &info,
                                                  std::set<int> &all_unhit_id_continue_not_following_lane_guide)
{
    /*
        车辆不按车道导向行驶
        Returns:
        class Lane_Direction:
        STRAIGHT = 0
        LEFT = 1
        RIGHT = 2
        STRAIGHT_LEFT = 3
        STRAIGHT_RIGHT = 4
        STRAIGHT_LEFT_RIGHT = 5
        UNKOWN = 6
    */
    if (history_speed_info[id]->x_diff.size() >= 3)
    {
        // cjm 1201 避免只觸發一幀，在軟件在丟數據
        bool car_not_follow_lane_status = false;
        if (not_follow_lane_status.count(id) != 0){
            if (cur_time - not_follow_lane_status[id] >= 0 and cur_time - not_follow_lane_status[id] <= 1.5 *1000)
            {
                car_not_follow_lane_status = true;
            }
        }

        if (std::find(classType->motor_type.begin(), classType->motor_type.end(), int(info->class_id)) != classType->motor_type.end() and
            info->v > 1 and (config_matrix(int(info->center_px), int(info->center_py), Config_Matrix_Index::CENTER_AREA) == 1 or
            config_matrix(int(info->center_px), int(info->center_py), Config_Matrix_Index::SIDEWALK) >= 1 or car_not_follow_lane_status))
        {
            double coincidence = cur_id_info[id]->coincidence;
            bool cur_car_wrong_direct = false;
            if (id_last_lane.count(id) !=0)
            {
                int lane = id_last_lane[id][id_last_lane[id].size() - 1]->lane;
                if (!(lane_info[lane]->is_non_motorway))
                {
                    cur_car_wrong_direct = true;
                }
            }
            if (cur_car_wrong_direct)
            {
                update_now_and_history(id, coincidence, id_continue_not_following_lane_guide,
                                       history_not_following_lane_guide,
                                       all_unhit_id_continue_not_following_lane_guide);
            }
        }
    }
}

//manxing
//void Traffic_Flow::get_id_continue_stroll_info(int &id, std::shared_ptr<Detect_Info> &info,
//                                               std::set<int> &all_unhit_id_continue_stroll){
//    decltype(id_continue_stroll_info) :: iterator  iter = id_continue_stroll_info.begin();
//    if (iter->first){
//        all_unhit_id_continue_stroll.insert(iter->first);
//    }
//    for (auto & it : cur_id_info){
//        if (it.second->y >240 or it.second->y < 30){
//            continue;
//        }
//        if (!cur_id_lane.count(id)){
//            continue;
//        }
//        if (history_speed_info.count(id)){
//            if (history_speed_info[id]->moving_average_speed.size() < 50){
//                continue;
//            }
////            if (history_speed_info[id]->y_max -history_speed_info[id]->y_min < 50){
////                continue;
////            }
//            if (cur_id_info[id]->l<2){
//                continue;
//            }
//        }
//        double min_v = cur_id_lane.count(id) ? lane_info[cur_id_lane[id]]->min_v : param["min_v"] / 3.6;
//
//        double mean_v = std::accumulate(history_speed_info[id]->v.begin(), history_speed_info[id]->v.end(), 0.0);
//        if (1.3<abs(history_speed_info[id]->moving_average_speed[history_speed_info[id]->moving_average_speed.size()-1])<abs(min_v) and 1.3<abs(mean_v)<min_v
//        and abs(history_speed_info[id]->acceleration[history_speed_info[id]->acceleration.size() - 1]) < 1){
//            if (find(classType->motor_type.begin(),classType->motor_type.end(),int(info->class_id)) !=classType->motor_type.end()){
//                double coindence = 0.5;
//                update_now_and_history(id,coindence,id_continue_stroll_info,history_stroll,all_unhit_id_continue_stroll);
//            }
//        }
//    }
//    for (auto & id : all_unhit_id_continue_stroll){
//        id_continue_stroll_info.erase(id);
//    }
//}

void Traffic_Flow::get_id_continue_stroll_info(int &id, std::shared_ptr<Detect_Info> &info,
                                               std::set<int> &all_unhit_id_continue_stroll) {
    if (history_speed_info.count(id) and cur_id_lane.count(id)){
        if (history_speed_info[id]->moving_average_speed.size() >= 50 and
            abs(history_speed_info[id]->y_max - history_speed_info[id]->y_min) >= 50 and
            history_speed_info[id]->normal_rate >= 0.6 and info->l >= 2){
            double min_v = cur_id_lane.count(id) ? lane_info[cur_id_lane[id]]->min_v : param["min_v"] / 3.6;

            double mean_v = std::accumulate(history_speed_info[id]->v.begin(), history_speed_info[id]->v.end(), 0.0) / history_speed_info[id]->v.size();
            if (abs(info->v) > 1.3 and abs(info->v) < min_v and abs(mean_v) > 1.3 and abs(mean_v) < min_v and
                std::abs(history_speed_info[id]->acceleration[history_speed_info[id]->acceleration.size() - 1]) < 1){
                std::vector<double> temp;
                for (int i = 0; i < history_speed_info[id]->acceleration.size(); ++i) {
//                    if (int (history_speed_info[id]->acceleration[i]) == 0){   //yk:wrong in int (history_speed_info[id]->acceleration[i]) == 0
//                        temp.push_back(history_speed_info[id]->acceleration[i]);
//                    }
                    if (history_speed_info[id]->acceleration[i] == 0){   //yk:wrong in int (history_speed_info[id]->acceleration[i]) == 0
                        temp.push_back(history_speed_info[id]->acceleration[i]);
                    }
                }
                if (temp.size() <= 8 and std::find(classType->motor_type.begin(), classType->motor_type.end(), int (info->class_id)) != classType->motor_type.end()){
                    double coincidence = history_speed_info[id]->normal_rate;
                    if (id_trajectory_eval.size())
                    {
                        id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->coincidence[int (Abnormal_Class_Index::stroll)] = coincidence;
                        id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->final_coincidence[Abnormal_Class_Index::stroll] = coincidence;
                        update_now_and_history(id, coincidence, id_continue_stroll_info, history_stroll, all_unhit_id_continue_stroll);
                    }
                }
            }
        }
    }
}
//chaosu
//void Traffic_Flow::get_id_continue_speeding_info(int &id, std::shared_ptr<Detect_Info> &info,
//                                                 std::set<int> &all_unhit_id_continue_speeding_for_motor,
//                                                 std::set<int> &all_unhit_id_continue_speeding_for_non_motor){
//    decltype(id_continue_speeding_info_for_motor)::iterator iter = id_continue_speeding_info_for_motor.begin();
//    if (iter->first)
//    {
//        all_unhit_id_continue_speeding_for_motor.insert(iter->first);
//    }
//    decltype(id_continue_speeding_info_for_non_motor)::iterator ite = id_continue_speeding_info_for_non_motor.begin();
//    if (ite->first)
//    {
//        all_unhit_id_continue_speeding_for_non_motor.insert(iter->first);
//    }
//    for (auto & it : cur_id_info){
//        if (!cur_id_lane.count(id)){
//            continue;
//        }
//        if (it.second->y >240 or it.second->y <30){
//            continue;
//        }
//        double max_v = cur_id_lane.count(id) > 0 ? lane_info[cur_id_lane[id]]->max_v : param["max_v"] / 3.6;
//        double mean_v = SumVector(history_speed_info[id]->moving_average_speed) /
//                            history_speed_info[id]->moving_average_speed.size();
//        if (abs(it.second->v) > abs(max_v)){
//            double coincidence = 0.5;
//            if (find(classType->motor_type.begin(), classType->motor_type.end(), int (info->class_id)) != classType->motor_type.end()){
//                update_now_and_history(id,coincidence,id_continue_speeding_info_for_motor,history_speeding_for_motor,all_unhit_id_continue_speeding_for_motor);
//            }
//            if (find(classType->non_motor_type.begin(), classType->non_motor_type.end(), int (info->class_id)) != classType->non_motor_type.end()){}
//                update_now_and_history(id,coincidence,id_continue_speeding_info_for_non_motor,history_speeding_for_non_motor,all_unhit_id_continue_speeding_for_non_motor);
//        }
//
//    }
//    for (auto & id : all_unhit_id_continue_speeding_for_motor) {
//        id_continue_speeding_info_for_motor.erase(id);
//    }
//    for (auto & id : all_unhit_id_continue_speeding_for_non_motor) {
//        id_continue_speeding_info_for_non_motor.erase(id);
//    }
//}



void Traffic_Flow::get_id_continue_speeding_info(int &id, std::shared_ptr<Detect_Info> &info,
                                                 std::set<int> &all_unhit_id_continue_speeding_for_motor,
                                                 std::set<int> &all_unhit_id_continue_speeding_for_non_motor) {
    if (cur_id_lane.count(id) and cur_id_lane[id] > 0){//if the id car is in the lane.
        double max_v = cur_id_lane.count(id) > 0 ? lane_info[cur_id_lane[id]]->max_v : param["max_v"] / 3.6;
        if (history_speed_info.count(id) > 0 and history_speed_info[id]->moving_average_speed[history_speed_info[id]->moving_average_speed.size() - 1] > max_v){
            history_speed_info[id]->speeding_count += 1;
            if (history_speed_info[id]->speeding_count > 20){
                history_speed_info[id]->speeding_count = 20;
            }
            if (id_count.count(id) > 0 and id_count[id]->times > 20 and history_speed_info[id]->unspeeding_count > 0){
                history_speed_info[id]->unspeeding_count -= 1;
            }
        }
        else
        {
            history_speed_info[id]->unspeeding_count += 1;
            if (history_speed_info[id]->unspeeding_count > 20){
                history_speed_info[id]->unspeeding_count = 20;//yk:wrong speeding_count->unspeeding_count
            }
            if (id_count.count(id) > 0 and id_count[id]->times > 20 and history_speed_info[id]->speeding_count > 0){
                history_speed_info[id]->speeding_count += 1;
            }
        }
        if (id_count.count(id)){
            if (id_count[id]->times < 5){
                history_speed_info[id]->speeding_rate = 0;
            }
            else
            {
                history_speed_info[id]->speeding_rate = float(history_speed_info[id]->speeding_count) / 20;
                //yk:history_speed_info[id]->speeding_count change to float(history_speed_info[id]->speeding_count)
            }
        }
        if (!(history_speed_info.count(id) > 0 and history_speed_info[id]->moving_average_speed.size() < 50)){
//            double mean_v = std::accumulate(history_speed_info[id]->moving_average_speed.begin(),
//                                            history_speed_info[id]->moving_average_speed.end(), 0) /
//                                                    history_speed_info[id]->moving_average_speed.size();

            double mean_v = SumVector(history_speed_info[id]->moving_average_speed) /
                            history_speed_info[id]->moving_average_speed.size();
            if (history_speed_info[id]->moving_average_speed[history_speed_info[id]->moving_average_speed.size() - 1] > max_v and
                mean_v > max_v and std::abs(history_speed_info[id]->acceleration[history_speed_info[id]->acceleration.size() - 1]) < 2){//one of the speeding judge condition
                std::vector<double> temp;
                for (auto & i : history_speed_info[id]->acceleration) {
//                    if (int (i) == 0){   //yk:wrong int ->
//                        temp.push_back(i);
//                    }
                    if (i == 0){
                        temp.push_back(i);
                    }
                }
                if (temp.size() <= 8){ //accleration size bigger than 8,continue
                    double coincidence = history_speed_info[id]->speeding_rate;
                    id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->coincidence[Abnormal_Class_Index::speeding_for_motor] = coincidence;
                    if (coincidence >= 0.5){ //coincidence less than0.8 continue
                        id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->final_coincidence[Abnormal_Class_Index::speeding_for_motor] = coincidence;
                        if (std::find(classType->motor_type.begin(), classType->motor_type.end(), int (info->class_id)) != classType->motor_type.end()){
                            update_now_and_history(id, coincidence, id_continue_speeding_info_for_motor, history_speeding_for_motor, all_unhit_id_continue_speeding_for_motor);
                        }
                        else
                        {
                            update_now_and_history(id, coincidence, id_continue_speeding_info_for_motor, history_speeding_for_non_motor, all_unhit_id_continue_speeding_for_non_motor);
                        }
                    }
                }
            }
        }
    }
}
//wrong in global_frame_time = float 10163803 and global_frame_id = int 0 event_no
//更新最新事件连续出现的帧数和时间段，并保存历史时间段中
void Traffic_Flow::update_now_and_history(int &idx, double &coincidence, std::map<int, std::shared_ptr<Event_Continue_Info>> &idx_continue_info,std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> &history_event,
                                          std::set<int> &unhit_idxs, const float &start_forward_time) {

    if (!idx_continue_info.count(idx))
    { // if this id is not in idx_continue_info, initialize idx_continue_info
        idx_continue_info[idx] = std::make_shared<Event_Continue_Info>(start_forward_time, cur_time, coincidence); //cur_time is this frame time
//        cout<<"idx:"<<idx<<endl;
        idx_continue_info[idx]->Assignment(cur_id_info[idx]);

        if (!history_event.count(idx))
        { // if this id not appear in the history_event,do if
            event_cnt += 1;
            event_cnt %= 65000;  //xcb add
            idx_continue_info[idx]->event_no = event_cnt;//yk:the event_no ans not same with python
            history_event[idx] = {idx_continue_info[idx]}; //yk:no wrong
        }
        else
        {
            auto this_history_event = history_event[idx];
            if (idx_continue_info[idx]->start_time - this_history_event[this_history_event.size() - 1]->end_time <= 60 * 1000)
            {
//            if (idx_continue_info[idx]->start_time - this_history_event[this_history_event.size() - 1]->end_time < 60 * 1000){ //yk:wrong end_time is using the lidar info
                this_history_event[this_history_event.size() - 1]->end_time = idx_continue_info[idx]->end_time;
                idx_continue_info[idx]->event_no = this_history_event[this_history_event.size() - 1]->event_no;
            }
            else
            {
                event_cnt += 1;
                event_cnt %= 65000;
                idx_continue_info[idx]->event_no = event_cnt;
                history_event[idx] = {idx_continue_info[idx]};
            }
        }
    }
    else // this id in idx_continue_info, hit_count+1
    {
        unhit_idxs.erase(idx);
        idx_continue_info[idx]->hit_count += 1;
//        printf("weiting.hit_count+1");
        idx_continue_info[idx]->coincidence = ((idx_continue_info[idx]->hit_count - 1) * idx_continue_info[idx]->coincidence + coincidence) / idx_continue_info[idx]->hit_count;
        idx_continue_info[idx]->end_time = cur_time;

        if (!history_event.count(idx)){
            event_cnt += 1;
            event_cnt %= 65000;
            idx_continue_info[idx]->event_no = event_cnt;
            history_event[idx] = {idx_continue_info[idx]};
        }
        else
        {
            history_event[idx][history_event[idx].size() - 1]->end_time = idx_continue_info[idx]->end_time;
            history_event[idx][history_event[idx].size() - 1]->coincidence = idx_continue_info[idx]->coincidence;
        }
    }
    if(del_flag)
    {
        del_long_time_miss_history(history_event);
    }
    
    else if(history_event.size() > 30)
    {
        del_long_time_miss_history(history_event);
    }
    
}



//void Traffic_Flow::update_now_and_history(int &idx, double &coincidence, std::map<int, std::shared_ptr<Event_Continue_Info>> &idx_continue_info,
//                                          std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> &history_event,
//                                          std::set<int> &unhit_idxs, const float &start_forward_time) {
//
//    if (!idx_continue_info.count(idx)){ // if this id is not in idx_continue_info, initialize idx_continue_info
//        idx_continue_info[idx] = std::make_shared<Event_Continue_Info>(start_forward_time, cur_time, coincidence); //cur_time is this frame time
//        idx_continue_info[idx]->Assignment(cur_id_info[idx]);
//        printf("weiting.hit_count_1");
//        if (!history_event.count(idx)){ // if this id not appear in the history_event,do if
//            event_cnt += 1;
//            idx_continue_info[idx]->event_no = event_cnt;//yk:the event_no ans not same with python
//            history_event[idx] = {idx_continue_info[idx]}; //yk:no wrong
//        }
//        else
//        {
//            auto this_history_event = history_event[idx];
//            if (idx_continue_info[idx]->start_time - this_history_event[this_history_event.size() - 1]->end_time <= 60){
////            if (idx_continue_info[idx]->start_time - this_history_event[this_history_event.size() - 1]->end_time < 60 * 1000){ //yk:wrong end_time is using the lidar info
//                this_history_event[this_history_event.size() - 1]->end_time = idx_continue_info[idx]->end_time;
//                idx_continue_info[idx]->event_no = this_history_event[this_history_event.size() - 1]->event_no;
//            }
//            else
//            {
//                event_cnt += 1;
//                idx_continue_info[idx]->event_no = event_cnt;
//                history_event[idx] = {idx_continue_info[idx]};
//            }
//        }
//    }
//    else // this id in idx_continue_info, hit_count+1
//    {
//        unhit_idxs.erase(idx);
//        idx_continue_info[idx]->hit_count += 1;
//        printf("weiting.hit_count+1");
//        idx_continue_info[idx]->coincidence = ((idx_continue_info[idx]->hit_count - 1) * idx_continue_info[idx]->coincidence + coincidence) / idx_continue_info[idx]->hit_count;
//        idx_continue_info[idx]->end_time = cur_time;
//
//        if (!history_event.count(idx)){
//            event_cnt += 1;
//            idx_continue_info[idx]->event_no = event_cnt;
//            history_event[idx] = {idx_continue_info[idx]};
//        }
//        else
//        {
//            history_event[idx][history_event[idx].size() - 1]->end_time = idx_continue_info[idx]->end_time;
//            history_event[idx][history_event[idx].size() - 1]->coincidence = idx_continue_info[idx]->coincidence;
//        }
//    }
//}
//初始化异常事件检测的信息
void Traffic_Flow::init_abnormal_event_info() {
    cur_occupy_dedicated_lane.clear();
    cur_people_occupy_motor_lane.clear();
    cur_non_motor_occupy_motor_lane.clear();
    cur_id_pix_v.clear();
    cur_id_pix_v_by_angle.clear();
    cur_retrograde_for_motor.clear();
    cur_retrograde_for_non_motor.clear();
    cur_speeding_for_motor.clear();
    cur_speeding_for_non_motor.clear();
    cur_stroll.clear();
    cur_cross_line.clear();
    cur_illegal_stop.clear();
    cur_cross_lane_for_non_motor.clear();
    cur_cross_lane_for_people.clear();
    cur_cross_lane_for_motor.clear();
    cur_run_the_red_light_for_motor.clear();
    cur_run_the_red_light_for_people.clear();
    cur_occupy_bus_lane.clear();
    cur_change_lanes_illegal.clear();
    cur_construction_sign.clear();
    cur_spills.clear();
    cur_accident.clear();
    cur_roadwork.clear();
    cur_person_not_walk_zebra_stripe.clear();
    cur_occupy_zebra_stripe.clear();
    cur_not_following_lane_guide.clear();
    cur_stop_over_terminateline.clear();
    cur_stop_over_terminateline_for_motor.clear();
    cur_occupy_emergency_lane.clear();
    cur_congestion.clear();
    cur_big_car_in_dedicated_lane.clear();
    cur_traverse.clear();
    lane2zone.clear();
    for (auto it : lane_fix_info)
    {
        lane2zone[it.first] = int(it.first/1000);
    }
    zone2bus = {{1,0},{2,0},{3,0},{4,0}};
}

void Traffic_Flow::clear_id_info() {
    std::set<int> all_set;
    // for (auto & it : id_dist_list) {
    //     all_set.insert(it.first);
    // }
    for (auto & it : id_trajectory_eval) {
        all_set.insert(it.first);
    }
    for (auto & idx_ : all_set) {

        // repeat delete cjm 0831
        // if (id_dist_list.count(idx_) != 0 and id_dist_list[idx_].size() > 0 and 
        //     cur_time - id_dist_list[idx_][id_dist_list[idx_].size() - 1][0] >= 3 * 1000){
        //     id_dist_list.erase(idx_);
        //     malloc_trim(0);
        // }
        if (id_trajectory_eval.count(idx_) != 0 and cur_time - id_trajectory_eval[idx_][id_trajectory_eval[idx_].size() - 1]->time >= 10 * 1000){
            id_trajectory_eval.erase(idx_);
            malloc_trim(0);
        }
    }
}

//set id_trajectory_eval, id_dist_list
void Traffic_Flow::trajectory_evalute(int &id, std::shared_ptr<Detect_Info> &info) {
#if 1
    std::vector< double > center = {info->x, info->y};
    int nn = 1;
    std::vector<double> distVec;
    auto res = pcd_tree->nearest_point(center); //yk: the return of pcd_tree is not same
    for (double b : res) {
        distVec.push_back(b);
    }

    double pos_coincidence = 0;
    double time_gap_each = 0;
    if (res.size()!=0)// yk:(!nn) -> (res.size()!=0)
    {
        double dis_gap = 0;
        double dist = std::sqrt(std::pow((distVec[0] - center[0]), 2) + std::pow((distVec[1] - center[1]), 2));  //yk:ans not same

        if (distVec[0] - center[0] > 0)
        {
            dist *= -1;
        }
        if (!id_dist_list.count(id))
        {
            id_dist_list[id] = {{double (cur_time), dist, dis_gap, pos_coincidence, time_gap_each, double(center[0]), info->l, info->pre_frame}};
        }
        else
        {
            time_gap_each = std::abs(double (cur_time) - id_dist_list[id][id_dist_list[id].size() - 1][0]) / 1000;//yk:time_gap_each ms transfer second
            dis_gap = std::abs(dist - id_dist_list[id][id_dist_list[id].size() - 1][1]);
            //两帧之间的距离置信度
            double dis_gap_coincidence;
            if (dis_gap <= 0.15)
            {
                dis_gap_coincidence = 1 - dis_gap * 4 / 3;
            }
            else
            {
//                dis_gap_coincidence = std::exp(-2 * std::pow(dis_gap- 0.15, 2) / 0.25 / (0.5 * std::sqrt(2 * M_PI)));//yk:not same with python,which wrong? C++code is wrong
                dis_gap_coincidence = std::exp(-2 * std::pow(dis_gap- 0.15, 2) / 0.25) / (0.5 * std::sqrt(2 * M_PI));//yk:9.13-16:49
            }
            //位置置信度的计算
            double dis_gap_vel = 0.1 * dis_gap / time_gap_each;
            if (dis_gap_vel <= 0.15)
            {
                pos_coincidence = 1 - dis_gap_vel * 4 / 3;
            }
            else
            {
                pos_coincidence = std::exp(-2 * std::pow(dis_gap_vel- 0.15, 2) / 0.25) / (0.5 * std::sqrt(2 * M_PI));//yk:wrong with the gaussian function
            }
            // 当前该目标的时间戳  当前帧和上一帧的距离 距离置信度 位置置信度 时间差 中心点x坐标 目标长度
            id_dist_list[id].push_back({double (cur_time), dist, dis_gap_coincidence, pos_coincidence, time_gap_each, double (center[0]), info->l, info->pre_frame});
            //yk:dis_gap_coincidence, pos_coincidence error is little big. Time_gap_each is wrong 9.13-16.24 to be modified
            //id_dist_list contains id,time,dist,dist_gap_coincidence,poscoincidence,time_gap,x,y,l,pre_frame.
        }
    } //yk:no wrong 9.13-18:36
    // 同一个id评估50次，删除掉第一条记录
    if (id_dist_list[id].size() > 50)
    {
        id_dist_list[id].erase(id_dist_list[id].begin());
        malloc_trim(0);
    }
    int lane = -1;
    if (cur_id_lane.count(id))
    {
        lane = cur_id_lane[id];    //get the lane of the cur id car
    }
    if (!id_trajectory_eval.count(id))
    {
        id_trajectory_eval[id] = {std::make_shared<History_Continue_Trajectory_Eval>(lane, cur_time, center[0], center[1])};
    }
    else
    {
        id_trajectory_eval[id].push_back(std::make_shared<History_Continue_Trajectory_Eval>(lane, cur_time, center[0], center[1]));
        if (id_trajectory_eval[id].size() > 50)// more than 50 frame,delete the first one
        {
            id_trajectory_eval[id].erase(id_trajectory_eval[id].begin());
            malloc_trim(0);
        }
        if (history_speed_info.count(id) and history_speed_info[id]->x_diff.size() >= 10 and id_dist_list[id].size() >= 20)
        {
            std::vector<double> pos_coin = id_dist_list[id][id_dist_list[id].size() - 1];

//            double temp0 = std::accumulate(history_speed_info[id]->car_l.begin(), history_speed_info[id]->car_l.end(), 0) / history_speed_info[id]->car_l.size();
            double temp0 = SumVector(history_speed_info[id]->car_l) / history_speed_info[id]->car_l.size();
            double length_std = 0;
            for (int i = 0; i < history_speed_info[id]->car_l.size(); ++i) {
                length_std += std::pow(history_speed_info[id]->car_l[i] - temp0, 2);
            }
            length_std = std::sqrt(length_std / history_speed_info[id]->car_l.size());  // standard deviation

            double pose_x_std = 0;
            temp0 = SumVector(history_speed_info[id]->x) / history_speed_info[id]->x.size();
//            temp0 = std::accumulate(history_speed_info[id]->x.begin(), history_speed_info[id]->x.end(), 0) / history_speed_info[id]->x.size();
            for (int i = 0; i < history_speed_info[id]->x.size(); ++i) {
                pose_x_std += std::pow(history_speed_info[id]->x[i] - temp0, 2);
            }
            pose_x_std = std::sqrt(pose_x_std / history_speed_info[id]->x.size());
            double pose_x_diff_mean = SumVector(history_speed_info[id]->x_diff) / history_speed_info[id]->x_diff.size();

            double vel_std = 0;
            temp0 = SumVector(history_speed_info[id]->v) / history_speed_info[id]->v.size();
            for (int i = 0; i < history_speed_info[id]->v.size(); ++i) {
                vel_std += std::pow(history_speed_info[id]->v[i] - temp0, 2);
            }
            vel_std = std::sqrt(vel_std / history_speed_info[id]->v.size());
            std::vector<std::vector<double>> pos_dist_list_np = id_dist_list[id]; //save the newest 20 frame
            if (pos_dist_list_np.size() > 20){
                pos_dist_list_np.erase(pos_dist_list_np.begin(), pos_dist_list_np.begin() + pos_dist_list_np.size() - 20);
            } //save the newest 20 frame

            float k = 0;
            for (int i = 0; i < pos_dist_list_np.size(); ++i) {
                if (pos_dist_list_np[i][7] > 0){
                    k += 1;
                }
            }
            double pre_cnt_rate = k / pos_dist_list_np.size(); //yk:wrong ans = 0.15

            id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_coin = pos_coin[2];  //id_trajectory_eval contains id, lanes, time, std
            id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pos_anno_vel_coin = pos_coin[3];
            id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->vel_std = vel_std;
            id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->length_std = length_std;
            id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pose_x_std = pose_x_std;
            id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pose_x_diff_mean = pose_x_diff_mean;
            id_trajectory_eval[id][id_trajectory_eval[id].size() - 1]->pre_cnt_rate = pre_cnt_rate;
            //yk:暂无融合速率 nowrong 9.14-12:03
        }
    }
#endif
}

std::map<int, int> Traffic_Flow::input_land_box_process() {
    std::map<int, int> lane_box_sorted;
    for (decltype(cur_lane_box_info)::const_iterator it = cur_lane_box_info.cbegin(); it != cur_lane_box_info.cend(); ++it) {
    }
    return lane_box_sorted;
}

//chaosupanduan
void Traffic_Flow::judge_speeding(std::set<int> &all_unhit_id_continue_speeding_for_motor,
                                  std::set<int> &all_unhit_id_continue_speeding_for_non_motor) {
    for (auto & id : all_unhit_id_continue_speeding_for_motor) {
        id_continue_speeding_info_for_motor.erase(id);
    }
    for (auto & id : all_unhit_id_continue_speeding_for_non_motor) {
        id_continue_speeding_info_for_non_motor.erase(id);
    }
    for (auto & it : id_continue_speeding_info_for_motor) {  // directly use the content of variable-id_continue_speeding_info_for_motor
        if (it.second->hit_count < 10){
            continue;
        }

        cur_speeding_for_motor.push_back(it.first);

    }
    for (auto & it : id_continue_speeding_info_for_non_motor)
    {
        if (it.second->hit_count < 10){
            continue;
        }
        cur_speeding_for_non_motor.push_back(it.first);
    }
}
//manxingpanduan
void Traffic_Flow::judge_stroll(std::set<int> &all_unhit_id_continue_stroll,
                                std::map<int, int> &lane_box_sorted) {

    for (auto & id : all_unhit_id_continue_stroll) {
        id_continue_stroll_info.erase(id);
    }

    std::map<int, int> lane_id_stroll;
    if (id_continue_stroll_info.size()!=0){
        lane_id_stroll.clear();
        for (auto &it : lane_info){
            lane_id_stroll[it.first] = 0;
        }
    }
    for (auto & it : id_continue_stroll_info){
        if (it.second->hit_count < 10){
            continue;
        }
        if (cur_id_lane.count(it.first)){
            int lane = cur_id_lane[it.first];
            auto this_id_info = cur_id_info[it.first];
            double pc_id = this_id_info->pc_id;
            if (lane_local_static_congestion_state[lane]["null"][int (pc_id)] < Congestion_Level::slightly_congested and
                lane_mean_v_info_by_all_v[lane]["pc_id"][int (pc_id)]->congestion < Congestion_Level::slightly_congested and
                lane_mean_v_info_by_all_v[lane]["pc_id"][int (pc_id)]->cur_occupancy <= 0.4){
                if (std::find(cur_stroll.begin(), cur_stroll.end(), it.first) == cur_stroll.end()){
                    for (auto & exist_id : cur_stroll){
                        if (lane == cur_id_lane[exist_id] and lane_id_stroll.count(cur_id_lane[int (exist_id)])){
                            lane_id_stroll[cur_id_lane[exist_id]] += cur_id_info[exist_id]->l;
                        }
                    }
                    auto stroll_car_on_lane = lane_id_stroll[lane];
                    if (stroll_car_on_lane <30){
                        cur_stroll.push_back(it.first);
                    }
                }
            }
        }
    }
}




//void Traffic_Flow::judge_stroll(std::set<int> &all_unhit_id_continue_stroll,
//                                std::map<int, int> &lane_box_sorted) {
//    for (auto & id : all_unhit_id_continue_stroll)
//    {
//        id_continue_stroll_info.erase(id);
//    }
//    std::map<int, int> lane_id_stroll;
//    if (id_continue_stroll_info.size())
//    {
//        lane_id_stroll.clear();
//        for (auto & it : lane_info){
//            lane_id_stroll[it.first] = 0;
//        }
//    }
//
//    for (auto & it : id_continue_stroll_info)
//    {
//        if (it.second->hit_count < 10){
//            continue;
//        }
//        if (cur_id_lane.count(it.first))
//        {
//            int lane = cur_id_lane[it.first];
//            auto this_id_info = cur_id_info[it.first];
//            double pc_id = this_id_info->pc_id;
////            std::map<int, std::map<std::string, std::map<int, T1>>>
//            if (lane_local_static_congestion_state[lane]["null"][int (pc_id)] < Congestion_Level::slightly_congested and
//                    lane_mean_v_info_by_all_v[lane]["pc_id"][int (pc_id)]->congestion < Congestion_Level::slightly_congested and
//                    lane_mean_v_info_by_all_v[lane]["pc_id"][int (pc_id)]->cur_occupancy <= 0.4)
//            {
//                if (std::find(cur_stroll.begin(), cur_stroll.end(), it.first) == cur_stroll.end())
//                {
//                    for (auto & exist_id : cur_stroll)   //no use?  //yk:wrong in logic wrong
//                    {
//                        if (lane == cur_id_lane[int(exist_id)] and lane_id_stroll.count(cur_id_lane[int (exist_id)]))
//                        {
//                            lane_id_stroll[cur_id_lane[int (exist_id)]] += cur_id_info[int (exist_id)]->l;  //lane length = sum(every stroll car.l)
//                        }
//                    }
//                    if (lane_id_stroll[lane] < 30)   //if car whole length smaller than 30
//                    {
//                        cur_stroll.push_back(it.first);
//                        std::cout<<"cur_stroll"<<std::endl;
//                        for (int i = 0; i < cur_stroll.size(); ++i) {
//                            std::cout<<cur_stroll[i]<<std::endl;
//
//                        }
//                    }
//                }
//            }
//            else
//            {
////                if (lane_box_sorted.count(lane))
////                {
////
////                }
//            }
//
//        }
//    }
//}
//panduannixing
void Traffic_Flow::judge_retrograde(std::set<int> &all_unhit_id_continue_retrograde_for_motor,
                                    std::set<int> &all_unhit_id_continue_retrograde_for_non_motor,
                                    std::map<int, int> &lane_box_sorted) {
    for (auto & id : all_unhit_id_continue_retrograde_for_motor)
    {
        id_continue_retrograde_info_for_motor.erase(id);
    }
    for (auto & id : all_unhit_id_continue_retrograde_for_non_motor)
    {
        id_continue_retrograde_info_for_non_motor.erase(id);
    }
    for (auto & it : id_continue_retrograde_info_for_non_motor) {
        if (it.second->hit_count < 12){
            continue;
        }
        //  if (cur_id_info[it.first]->coincidence < 0.4)
        // {
        //     continue;
        // }
//        int lane = cur_id_lane[it.first];
//        double pc_id = cur_id_info[it.first]->pc_id;
//        auto this_count = lane_mean_v_info_by_all_v[lane]["pc_id"][int (pc_id)]->statistice_congestion_count;
//        /** 此处python程序有误，map无法排序，这两行的含义模糊不清。无法实现 */
//
//        if (lane_mean_v_info_by_all_v[lane]["pc_id"][int (pc_id)]->congestion >= Congestion_Level::slightly_congested)
//        {
//            continue;
//        }
        cur_retrograde_for_non_motor.push_back(it.first);
    }
    for (auto & it : id_continue_retrograde_info_for_motor) {
        if (it.second->hit_count < 40) {
            continue;
        }
        if (cur_id_info[it.first]->coincidence < 0.4)
        {
            continue;
        }
        cur_retrograde_for_motor.push_back(it.first);
    }
}


//biandaopanduan

void Traffic_Flow::judge_change_lanes(std::set<int> &all_unhit_id_continue_change_lanes,
                                      std::map<int, int> &lane_box_sorted) {
    for (auto & id : all_unhit_id_continue_change_lanes)
    {
        id_continue_change_lanes_info.erase(id);
    }

    for (auto & it : id_continue_change_lanes_info)
    {
        if (cur_id_info.count(it.first) and config_matrix(cur_id_info[it.first]->center_px, cur_id_info[it.first]->center_py, Config_Matrix_Index::FORBID_CROSS_LINE) < 1)
        {
            continue;
        }
        if (it.second->hit_count < 5){
            continue;
        }
        if (it.second->end_time - it.second->start_time < 0.3 * 1000)
        {
            continue;
        }
        int lane  = id_last_lane[it.first][id_last_lane[it.first].size() - 1]->lane;
        if (lane2zone.count(lane))
        {
            int zone = lane2zone[lane];
            if (zone2bus[zone] == 1)
            {
                continue;
            }
        }
        if (static_cast<int>(std::to_string(lane)[1] - '0') == 0)
        {
            continue;
        }
        if(lane_info[lane]->is_non_motorway)
        {
            continue;
        }
        cur_change_lanes_illegal.push_back(it.first);  //yk: can not come in
        std::vector<double> point_new = id_dist_list[it.first][id_dist_list[it.first].size() - 1]; //yk:缺少一行代码 compared with python code
        id_dist_list.erase(it.first);
        id_dist_list[it.first] = {point_new}; //yk:no wrong 9.15-11:50
    }
}


//yongdupanduan
void Traffic_Flow::judge_congestion_detect(std::set<int> &all_unhit_congestion_continue){
    for (auto & id : all_unhit_congestion_continue)
    {
        congestion_continue_info.erase(id);
    }
    for (decltype(congestion_continue_info)::const_iterator it = congestion_continue_info.cbegin();
         it != congestion_continue_info.cend(); ++it) {
        if (it->second->hit_count < 8){
            continue;
        }
        cur_congestion.push_back(it->first);
    }
}


void Traffic_Flow::judge_big_car_in_dedicated_lane(std::set<int> &all_unhit_big_car_in_dedicated_lane_continue){
    for (auto & id : all_unhit_big_car_in_dedicated_lane_continue)
    {
        big_car_in_dedicated_lane_continue_info.erase(id);
    }
    for (decltype(big_car_in_dedicated_lane_continue_info)::const_iterator it = big_car_in_dedicated_lane_continue_info.cbegin();
         it != big_car_in_dedicated_lane_continue_info.cend(); ++it) {
//    for(auto & it : congestion_continue_info){
        if (it->second->hit_count < 8){
            continue;
        }
        cur_big_car_in_dedicated_lane.push_back(it->first);
    }
}

void Traffic_Flow::judge_traverse(std::set<int> &all_unhit_traverse_continue){
    for (auto & id : all_unhit_traverse_continue)
    {
        traverse_continue_info.erase(id);
    }
    for (decltype(traverse_continue_info)::const_iterator it = traverse_continue_info.cbegin();
         it != traverse_continue_info.cend(); ++it) {
        if (it->second->hit_count < 2){
            continue;
        }
        cur_traverse.push_back(it->first);
    }
}


//panduanpaosaowu
void Traffic_Flow::spills_detect(std::set<int> &all_unhit_spills_coordinate_continue) {
    for (auto & spills_coordinate : all_unhit_spills_coordinate_continue)
    {
        spills_coordinate_continue_info.erase(spills_coordinate);
    }
    for (auto & it : spills_coordinate_continue_info) {
        if (it.second->hit_count < 20) {
            continue;
        }
        cur_spills.push_back(it.first);
    }
}

void Traffic_Flow::judge_construction(std::set<int> &all_unhit_construction){
    for(auto & construction:all_unhit_construction){
        construction_continue_info.erase(construction);
    }
    for(auto & it : construction_continue_info){
        if (it.second->hit_count < 20){
            continue;
        }
        cur_construction.push_back(it.first);
    }

}


//panduanfeijidongche
void Traffic_Flow::detect_occupy_dedicated_lane(std::set<int> &all_unhit_id_continue_occupy_dedicated_lane,
                                                std::set<int> &all_unhit_id_continue_people_occupy_motor_lane,
                                                std::set<int> &all_unhit_id_continue_non_motor_occupy_motor_lane) {
    for (auto & id : all_unhit_id_continue_occupy_dedicated_lane){
        id_continue_occupy_dedicated_lane_info.erase(id);
    }
    for (auto & id : all_unhit_id_continue_people_occupy_motor_lane){
        id_continue_people_occupy_motor_lane_info.erase(id);
    }
    for (auto &id : all_unhit_id_continue_non_motor_occupy_motor_lane){
        id_continue_non_motor_occupy_motor_lane_info.erase(id);
    }

    for (decltype(id_continue_occupy_dedicated_lane_info)::const_iterator it = id_continue_occupy_dedicated_lane_info.cbegin();
         it != id_continue_occupy_dedicated_lane_info.cend(); ++it) {
        if (it->second->hit_count < 30){
            continue;
        }
        cur_occupy_dedicated_lane.push_back(it->first);
    }
    for (decltype(id_continue_people_occupy_motor_lane_info)::const_iterator it = id_continue_people_occupy_motor_lane_info.cbegin();
         it != id_continue_people_occupy_motor_lane_info.cend(); ++it) {
        if (it->second->hit_count < 10){
            continue;
        }
        cur_people_occupy_motor_lane.push_back(it->first);
    }
    for (decltype(id_continue_non_motor_occupy_motor_lane_info)::const_iterator it = id_continue_non_motor_occupy_motor_lane_info.cbegin();
         it != id_continue_non_motor_occupy_motor_lane_info.cend(); ++it){
        if (it->second->hit_count < 10){
            continue;
        }
        cur_non_motor_occupy_motor_lane.push_back(it->first);
    }
}

//panduanfeijidongche
void Traffic_Flow::detect_occupy_bus_lane(std::set<int> &all_unhit_id_continue_occupy_bus_lane) 
{
    for (auto & id : all_unhit_id_continue_occupy_bus_lane){
        id_continue_occupy_bus_lane_info.erase(id);
    }
    for (decltype(id_continue_occupy_bus_lane_info)::const_iterator it = id_continue_occupy_bus_lane_info.cbegin();
         it != id_continue_occupy_bus_lane_info.cend(); ++it) 
    {
        if (cur_id_lane.count(it->first) == 0)
        {
            continue;
        }
        if (cur_id_lane[it->first] <= 0){
            continue;
        }
        if (lane_info[cur_id_lane[it->first]]->is_bus_lane == 0)
        {
            continue;
        }
        if (it->second->hit_count < 5)
        {
            continue;
        }
        cur_occupy_bus_lane.push_back(it->first);
    }
}

//xcb add panduan zunxun chedao zhiyin 
void Traffic_Flow::detect_car_not_following_lane_guide(std::set<int> &all_unhit_id_continue_not_following_lane_guide)
{
    for (auto & id : all_unhit_id_continue_not_following_lane_guide){
        id_continue_not_following_lane_guide.erase(id);
    }
    for (decltype(id_continue_not_following_lane_guide)::const_iterator it = id_continue_not_following_lane_guide.cbegin();
         it != id_continue_not_following_lane_guide.cend(); ++it)
    {
        if (confirm_not_following_info.count(it->first) == 0)
        {
            confirm_not_following_info[it->first] = history_not_following_lane_guide[it->first];
            confirm_not_following_info_cur_time[it->first] = cur_time;
            confirm_not_following_info_lane[it->first] = id_last_lane[it->first][id_last_lane[it->first].size() - 1]->lane;
        }
        else{
            confirm_not_following_info_cur_time[it->first] = cur_time;
        }
    }
    if (confirm_not_following_info.size() != 0)
    {
        std::map<int, std::vector<int>> new_lane_match_pair = lane_match_pair;
        // for (decltype(confirm_not_following_info)::iterator it = confirm_not_following_info.begin();
        //      it != confirm_not_following_info.end(); ++it)
        std::vector<int>confirm_not_following_info_delete;
        std::vector<int>confirm_not_following_info_cur_time_delete;
        std::vector<int>confirm_not_following_info_lane_delete;
        std::vector<int>not_follow_lane_status_delete;
        for(auto it : confirm_not_following_info)
        {
            bool idHasDelet = false;
            if (id_last_lane.count(it.first) != 0 and cur_id_info.count(it.first) != 0)
            {
                int lane = id_last_lane[it.first][id_last_lane[it.first].size() - 1]->lane;

                if (lane_fix_info.count(lane) != 0 and lane != confirm_not_following_info_lane[it.first])
                {
                    std::vector<int>::iterator iter = std::find(new_lane_match_pair[confirm_not_following_info_lane[it.first]].begin(),
                                                                new_lane_match_pair[confirm_not_following_info_lane[it.first]].end(),
                                                                lane);
                    // cjm 1225 車輛左轉容易駛入目標方向進口道
                    bool not_following = true;
                    for (auto & lane_id : new_lane_match_pair[confirm_not_following_info_lane[it.first]]){
                        if (std::to_string(lane)[0] == std::to_string(lane_id)[0]){
                            not_following = false;
                            break;
                        }
                    }
                    if (new_lane_match_pair.count(confirm_not_following_info_lane[it.first]) != 0 and 
                        new_lane_match_pair[confirm_not_following_info_lane[it.first]].size() > 0 and 
                        iter == new_lane_match_pair[confirm_not_following_info_lane[it.first]].end() and not_following) 
                    {
                        
                        cur_not_following_lane_guide.push_back(it.first);

                        if (not_follow_lane_status.count(it.first) == 0)
                        {
                            not_follow_lane_status[it.first] = cur_time; // cjm 1201 避免只觸發一幀，在軟件在丟數據
                        }

                        if (history_not_following_lane_guide.count(it.first) == 0)
                        {
                            history_not_following_lane_guide[it.first] = confirm_not_following_info[it.first];
                        }
                        
                        // cjm 0902 change end_time
                        history_not_following_lane_guide[it.first][0]->end_time = cur_time;
                    }

                    if (not_follow_lane_status.count(it.first) != 0 and cur_time - not_follow_lane_status[it.first] < 1.5 *1000)
                    {

                    }
                    else{
                        confirm_not_following_info_delete.push_back(it.first);
                        confirm_not_following_info_cur_time_delete.push_back(it.first);
                        confirm_not_following_info_lane_delete.push_back(it.first);
                        idHasDelet = true;
                        if (not_follow_lane_status.count(it.first) != 0 and cur_time - not_follow_lane_status[it.first] > 3 *1000)
                        {
                            not_follow_lane_status_delete.push_back(it.first);
                        }
                    }
                }
            }
            if (!idHasDelet)
            {
                if ((cur_time - confirm_not_following_info_cur_time[it.first]) > 60 * 1000)
                {
                    confirm_not_following_info_delete.push_back(it.first);
                    confirm_not_following_info_cur_time_delete.push_back(it.first);
                    confirm_not_following_info_lane_delete.push_back(it.first);
                }
            }
        }

        for (int key : confirm_not_following_info_delete)
        {
            confirm_not_following_info.erase(key);
        }

        for (int key : confirm_not_following_info_cur_time_delete)
        {
            confirm_not_following_info_cur_time.erase(key);
        }

        for (int key : confirm_not_following_info_lane_delete)
        {
            confirm_not_following_info_lane.erase(key);
        }

        for (int key : not_follow_lane_status_delete)
        {
            not_follow_lane_status.erase(key);
        }
    }
}
//panduanzhanyongjinji
void Traffic_Flow::detect_occupy_emergency_lane(std::set<int> &all_unhit_id_continue_occupy_emergency_lane) {
    for (auto & id : all_unhit_id_continue_occupy_emergency_lane){
        id_continue_occupy_emergency_lane_info.erase(id);
    }
    for (decltype(id_continue_occupy_emergency_lane_info)::const_iterator it = id_continue_occupy_emergency_lane_info.cbegin();
         it != id_continue_occupy_emergency_lane_info.cend(); ++it) {
        if (it->second->hit_count < 10){
            continue;
        }
        cur_occupy_emergency_lane.push_back(it->first);
    }
}
//xcb add
void Traffic_Flow::detect_stop_over_terminateline(std::set<int> &all_unhit_id_continue_stop_over_terminateline,std::set<int> &all_unhit_id_continue_stop_over_terminateline_for_motor)
{
    for (auto &id : all_unhit_id_continue_stop_over_terminateline)
    {
        id_continue_stop_over_terminateline.erase(id);
    }

    for (auto &id : all_unhit_id_continue_stop_over_terminateline_for_motor)
    {
        id_continue_stop_over_terminateline_for_motor.erase(id);
    }
    for (decltype(id_continue_stop_over_terminateline)::const_iterator it = id_continue_stop_over_terminateline.begin();
         it != id_continue_stop_over_terminateline.end(); ++it)
    {

        if (cur_id_info.count(it->first) and cur_id_info[it->first]->v > 0.3)
        {
            continue;
        }
        if (it->second->hit_count < 10)
        {
            continue;
        }
        cur_stop_over_terminateline.push_back(it->first);
    }

    // for motor
    for (decltype(id_continue_stop_over_terminateline_for_motor)::const_iterator it = id_continue_stop_over_terminateline_for_motor.begin();
         it != id_continue_stop_over_terminateline_for_motor.end(); ++it)
    {

        if (cur_id_info.count(it->first) and cur_id_info[it->first]->v > 0.3)
        {
            continue;
        }
        if (it->second->hit_count < 10)
        {
            continue;
        }
        cur_stop_over_terminateline_for_motor.push_back(it->first);
    }
}
//xcb panduanweiting
void Traffic_Flow::judge_illegal_stop(std::set<int> &all_unhit_id_continue_stop, std::map<int, int> &lane_box_sorted) {
    for (auto & id : all_unhit_id_continue_stop){
        id_continue_stop_info.erase(id);
    }
    // std::map<int, int> lane_id_stop_length_before;
    // if (id_continue_stop_info.size())
    // {
    //     lane_id_stop_length_before.clear();
    //     for (auto & it : lane_info){
    //         lane_id_stop_length_before[it.first] = 0;
    //     }
    // }

    for (auto & it : id_continue_stop_info)
    {

        

        if (cur_id_lane.count(it.first) and int(cur_id_lane[it.first]) > 0)
        {
            int lane = cur_id_lane[it.first];
            auto this_id_info = cur_id_info[it.first];
            if (std::find(classType->motor_type.begin(), classType->motor_type.end(), this_id_info->class_id) == classType->motor_type.end())
            {
                continue;
            }
            double pc_id = this_id_info->pc_id;
            auto this_count = lane_mean_v_info_by_all_v[lane]["pc_id"][int(pc_id)]->statistice_congestion_count;
            int this_count_sum = 0;
            std::map<int, int>::iterator item = this_count.begin();
            int firstValue = item->second;
            ++item;
            int secondValue = item->second;
            for (auto & iter : this_count){
                this_count_sum += iter.second;
            }
            if (this_count_sum < 30 and lane_local_static_congestion_state[lane]["null"][int(pc_id)] >= Congestion_Level::moderately_congested)
            {
                continue;
            }
            else if((float(this_count_sum - firstValue - secondValue)) / float(this_count_sum) > 0.3)
            {
                continue;
            }
            int c_px = cur_id_info[it.first]->center_px;
            int c_py = cur_id_info[it.first]->center_py;
            if (config_matrix(c_px, c_py, Config_Matrix_Index::FORBID_CROSS_LINE) == 1 and std::to_string(lane)[1] == '1')
            {
                continue;
            }
            if (cur_statistics[lane].count("queue_id") != 0 and std::find(cur_statistics[lane]["queue_id"].begin(),
                cur_statistics[lane]["queue_id"].end(), static_cast<float>(it.first)) != cur_statistics[lane]["queue_id"].end() and (!lane_info[lane]->is_non_motorway))
            {
                continue;
            }
            if (it.second->hit_count > 80 and std::find(classType->motor_type.begin(), classType->motor_type.end(), cur_id_info[it.first]->class_id) != classType->motor_type.end())
            {
                if (std::to_string(lane)[1] == '1' and !lane_info[lane]->is_non_motorway and it.second->hit_count < 500)
                {
                    continue;
                }
                if (cur_id_info[it.first]->v > 1)
                {
                    continue;
                }
                // cur_time
                cur_illegal_stop.push_back(it.first);
            }
        }
        // else
        // {
        //     if (std::find(classType->motor_type.begin(), classType->motor_type.end(), cur_id_info[it.first]->class_id) != classType->motor_type.end() and it.second->hit_count > 80)
        //     {
        //         int c_px = cur_id_info[it.first]->center_px;
        //         int c_py = cur_id_info[it.first]->center_py;
        //         int center_area =  config_matrix(c_px, c_py, Config_Matrix_Index::CENTER_AREA);
        //         int sidewalk = config_matrix(c_px, c_py, Config_Matrix_Index::SIDEWALK);
        //         if (center_area > 0 and it.second->hit_count > 80)
        //         {
        //             int waiting_area = config_matrix(c_px, c_py, Config_Matrix_Index::CENTER_AREA);
        //             if (waiting_area != 11)
        //             {
        //                 cur_illegal_stop.push_back(it.first);
        //             }
        //         }
        //         else if (center_area == 0 and sidewalk > 0 and it.second->hit_count > 80)
        //         {
        //             cur_illegal_stop.push_back(it.first);
        //         }
        //         else if (it.second->hit_count > 80)
        //         {
        //             cur_illegal_stop.push_back(it.first);
        //         }
        //     }
        // }
    }
}

void Traffic_Flow::accident_detect() {
    get_accident_coordinate_continue_info();
    for (auto & idx : accident_coordinate_continue_info)
    {
        cur_accident.push_back(idx.first);
    }
}
//保存当前帧的id信息，用于下一帧的雷达bmp像素速度计算
void Traffic_Flow::save_id_info() {

    last_id_info_0.push_back(cur_id_info);
    last_id_info_1.push_back(cur_time);
    for (int i = 0; i < last_id_info_1.size(); ++i){
        if (last_id_info_1.at(i) < cur_time - 15 * 1000){
            std::vector<std::map<int, std::shared_ptr<Detect_Info>>>::iterator it_0 = last_id_info_0.begin() + i;
            std::vector<double>::iterator it_1 = last_id_info_1.begin() + i;
            last_id_info_0.erase(it_0);
            last_id_info_1.erase(it_1);
        }else{
            break;
        }
    }
}
//事故检测
void Traffic_Flow::get_accident_coordinate_continue_info() {
    std::set<int> all_unhit_accident_coordinate_continue;
    std::vector<int> stop_list;
    if (cur_illegal_stop.size() >= 2 and cur_people_occupy_motor_lane.size() >= 1)
    {
        xt::xarray<float> det_xy_feature = xt::zeros<float>({int (cur_illegal_stop.size()), 4});
        xt::xarray<float> trk_xy_feature = xt::zeros<float>({int (cur_illegal_stop.size()), 4});
        // # 机动车静止状态(后续需要与违停区分开)
        for (int i = 0; i < cur_illegal_stop.size(); ++i)
        {
            int id = cur_illegal_stop[i];
            int class_id = cur_id_info[id]->class_id;
            double pc_id = cur_id_info[id]->pc_id;
            float px = cur_id_info[id]->x, py = cur_id_info[id]->y;
            xt::view(det_xy_feature, i) = xt::xarray<float>({px, py, float(pc_id), float (class_id)});
            xt::view(trk_xy_feature, i) = xt::xarray<float>({px, py, float(pc_id), float (class_id)});
        }
        xt::xarray<float> det_xy = xt::view(det_xy_feature, xt::all(), xt::range(0, 2));
        xt::xarray<float> trk_xy = xt::view(trk_xy_feature, xt::all(), xt::range(0, 2));
        xt::xarray<float> distance_matrix = xt::sqrt(-2 * xt::linalg::dot(det_xy, xt::transpose(trk_xy)) + xt::sum(xt::square(trk_xy), 1) + xt::transpose(xt::sum(xt::square(det_xy), 1)));

        int dis_stop;
        for (int i = 0; i < distance_matrix.shape(0); ++i) {
            for (int j = i + 1; j < distance_matrix.shape(1); ++j) {
                if (std::find(classType->car.begin(), classType->car.end(), int (cur_id_info[cur_illegal_stop[i]]->class_id)) != classType->car.end() and
                    std::find(classType->car.begin(), classType->car.end(), int (cur_id_info[cur_illegal_stop[j]]->class_id)) != classType->car.end())
                {
                    dis_stop = 7;
                }
                else if (std::find(classType->big_car.begin(), classType->big_car.end(), int (cur_id_info[cur_illegal_stop[i]]->class_id)) != classType->big_car.end() and std::find(classType->big_car.begin(), classType->big_car.end(), int (cur_id_info[cur_illegal_stop[j]]->class_id)) != classType->big_car.end())
                {
                    dis_stop = 25;
                }
                else
                {
                    dis_stop = 15;
                }
                if (distance_matrix(i, j) > 0 and distance_matrix(i, j) < dis_stop)
                {
                    float px = (det_xy(i, 0) + det_xy(j, 0)) / 2;
                    float py = (det_xy(i, 1) + det_xy(j, 1)) / 2;
                    xt::xarray<int> accident_xy = bmp_to_accident_coordinate({int (px), int(py)});
                    accident_matrix(accident_xy(0), accident_xy(1)) += 1;
                    if (accident_matrix(accident_xy(0), accident_xy(1)) > 20)
                    {
                        double pc_id_i = det_xy_feature(i, 2);
                        double pc_id_j = det_xy_feature(j, 2);
                        bool accident_happen = false;
                        for (auto & id : cur_people_occupy_motor_lane)
                        {
                            if (cur_id_info.count(id))
                            {
                                if (std::abs(pc_id_i - cur_id_info[id]->pc_id) <= 1 or std::abs(pc_id_j - cur_id_info[id]->pc_id) <= 1)
                                {
                                    accident_happen = true;
                                    break;
                                }
                            }
                        }
                        if (accident_happen)
                        {
                            if (std::find(stop_list.begin(), stop_list.end(), int (cur_illegal_stop[i])) == stop_list.end())
                            {
                                stop_list.push_back(int (cur_illegal_stop[i]));
                            }
                            if (std::find(stop_list.begin(), stop_list.end(), int (cur_illegal_stop[j])) == stop_list.end())
                            {
                                stop_list.push_back(int (cur_illegal_stop[j]));
                            }
                        }

                    }
                }
            }
        }
        if (stop_list.size())
        {
            for (auto & id : stop_list)
            {
                double coincidence = 0.5;
                update_now_and_history(id, coincidence, accident_coordinate_continue_info, history_accident, all_unhit_accident_coordinate_continue);
            }
        }
    }
    else
    {
        accident_matrix *= 0;
    }
    for (auto & accident_coordinate : all_unhit_accident_coordinate_continue)
    {
        accident_coordinate_continue_info.erase(accident_coordinate);
    }
}

/*
void Traffic_Flow::get_B3(std::shared_ptr<Event_Output> &event_res) {
    long t_now = ops::getTimeStamp();
    B3_cnt += 1;
    // ghj
    std::vector<std::vector<double>> B3_context;

    int normal_lane_num = lane_info.size() - turn_left_info.size();

    for (auto & it : lane_info)
    {
        if (std::to_string(it.first).size() != normal_lane_str_len)
        {
            continue;
        }
        std::shared_ptr<Lane_Info> this_lane_info = lane_info[it.first];
        std::map<std::string, std::vector<float>> this_cur_statistics = cur_statistics[it.first];
        std::map<int, int> this_cur_lane_classes_num = cur_lane_classes_num[it.first];
        std::map<std::string , int> this_cur_lane_classes_num2 = cur_lane_classes_num2[it.first];
        B3_context.push_back({
                                     lane_fix_info[it.first]->lane_output_name,
                                     double (logical_calculation(lane_info[it.first]->is_emergency_lane, lane_info[it.first]->is_non_motorway)),
                                     this_lane_info->angle_to_north,
                                     this_lane_info->max_v * 3.6,
                                     cur_lane_mean_v[it.first],
                                     this_cur_statistics["queue_length"][0] * 100,
//            double (std::accumulate(this_cur_statistics["space_headway"].begin(), this_cur_statistics["space_headway"].end(), 0)),//yk:wrong
                                     double (SumVector(this_cur_statistics["space_headway"])/this_cur_statistics["space_headway"].size()), //yk:mean space headway
                                     this_cur_statistics["lane_occupancy"][0] * 100,
                                     0,
                                     double (this_cur_lane_classes_num2["motor_type"]),
                                     double (this_cur_lane_classes_num[classType->car[0]]),
                                     0,
                                     double (this_cur_lane_classes_num[classType->bus[0]]),
                                     double (this_cur_lane_classes_num[classType->minibus[0]]),
                                     0,
                                     double (this_cur_lane_classes_num[classType->people[0]]),
                                     double (this_cur_lane_classes_num2["non_motor_type"]),
                                     double (this_cur_lane_classes_num[classType->truck[0]]),
                                     0
                             });
    }
    event_res = std::make_shared<Event_Output>(0, 0, 0, 0, B3_cnt, normal_lane_num, 0, B3_context, 0);
}
*/

// cjm 0920 add 
void Traffic_Flow::get_B3(std::shared_ptr<Event_Output> &event_res) {
    long t_now = ops::getTimeStamp();
    B3_cnt += 1;
    // ghj
    std::vector<std::vector<double>> B3_context;
    std::vector<std::vector<double>> B3_context_for_save;

    int normal_lane_num = lane_info.size() - turn_left_info.size();

    for (auto & it : lane_info)
    {
        if (std::to_string(it.first).size() != normal_lane_str_len or std::to_string(it.first)[1] == '0')
        {
            continue;
        }

        std::shared_ptr<Lane_Info> this_lane_info = lane_info[it.first];
        std::map<std::string, std::vector<float>> this_cur_statistics = cur_statistics[it.first];
        std::map<int, int> this_cur_lane_classes_num = cur_lane_classes_num[it.first];
        std::map<std::string , int> this_cur_lane_classes_num2 = cur_lane_classes_num2[it.first];
        int queueState = -1;
        if (this_cur_statistics["queue_length"][0] < 50)
        {
            queueState = 1;
        }
        else if (this_cur_statistics["queue_length"][0] >= 50 and this_cur_statistics["queue_length"][0] < 100)
        {
            queueState = 2;
        }
        else if (this_cur_statistics["queue_length"][0] >= 100)
        {
            queueState = 3;
        }
        // double (lane_fix_info[it.first]->lane_output_name),

        B3_context.push_back({
                                double (lane_num_switch(double(it.first))),
                                double (logical_calculation(lane_info[it.first]->is_emergency_lane, lane_info[it.first]->is_non_motorway)),
                                double (int(it.first)/1000),
                                this_lane_info->max_v * 3.6,
                                cur_lane_mean_v[it.first],
                                this_cur_statistics["queue_length"][0] * 100,
                                double (queueState),
                                this_cur_statistics["space_headway"].size() == 0 ? 0 : double (SumVector(this_cur_statistics["space_headway"])/this_cur_statistics["space_headway"].size()), //yk:mean space headway
                                this_cur_statistics["lane_occupancy"][0] * 100,
                                double (this_cur_lane_classes_num2["motor_type"]),
                                double (this_cur_lane_classes_num2["non_motor_type"]),
                                double (dir_num_switch(double(int(it.first)/100))),
                                this_cur_statistics["waitingTime"][0] // cjm 0920 not achieve
                             });
        B3_context_for_save.push_back({
                                     double (it.first),
                                     double (logical_calculation(lane_info[it.first]->is_emergency_lane, lane_info[it.first]->is_non_motorway)),
                                     double (int(it.first)/1000),
                                     this_lane_info->max_v * 3.6,
                                     cur_lane_mean_v[it.first],
                                     this_cur_statistics["queue_length"][0] * 100,
                                     double (queueState),
                                     this_cur_statistics["space_headway"].size() == 0 ? 0 : double (SumVector(this_cur_statistics["space_headway"])/this_cur_statistics["space_headway"].size()), //yk:mean space headway
                                     this_cur_statistics["lane_occupancy"][0] * 100,
                                     double (this_cur_lane_classes_num2["motor_type"]),
                                     double (this_cur_lane_classes_num2["non_motor_type"]),
                                     double (dir_num_switch(double(int(it.first)/100))),
                                     this_cur_statistics["waitingTime"][0] // cjm 0920 not achieve
                             });
    }

    // cjm 0920 not achieve no signal input
    // std::vector<std::vector<double>> B3_crosswalk;
    // for (auto & it : cur_crosswalk)
    // {

    // }



    event_res = std::make_shared<Event_Output>(0, 0, 0, 0, B3_cnt, normal_lane_num, 0, B3_context, 0);


    if (event_sign["save_event"])
    {
        std::string otherStyleTimeNameNow = ops::GetLocalTimeWithMs().substr(0, 10);
        std::string otherStyleTimeName = ops::TimeStampToString_sec(long (cur_time)).substr(0, 10);

        std::string path = m_config_path + "/save_event_data/";
        if (access(path.c_str(), 0) == -1) {
            mkdir(path.c_str(), S_IRWXU);
        }


        std::string name = path + "save_b3_rawdata_laneID_" + otherStyleTimeName + "_saved_in_" + otherStyleTimeNameNow + ".json";   // event data
        json savejson;

        for (auto info : B3_context_for_save)
        {
            savejson[std::to_string(info[0])] = info;
        }

        std::ofstream output(name, std::ios::app);
        output << savejson << std::endl;
    }
}



void Traffic_Flow::get_B4(std::shared_ptr<Event_Output> &event_res) {
    long t_now = ops::getTimeStamp();
    B4_cnt += 1;

    std::shared_ptr<B4_res> b4_res = std::make_shared<B4_res>();
   if (event_sign["save_event"])
   {
       std::string otherStyleTime = ops::GetLocalTimeWithMs().substr(0, 19);
       std::string otherStyleTimeBef = ops::GetLocalTimeWithMs(60).substr(0, 19);
       b4_res->startTimeStamp = otherStyleTimeBef;
       b4_res->endTimeStamp = otherStyleTime;
       b4_res->laneNum = lane_info.size();
   }

    // std::vector<std::vector<double>> B4_context;
    B4_context.clear(); // cjm 0927
    std::vector<std::vector<double>> B4_context_for_save;

    int normal_lane_num = lane_info.size() - turn_left_info.size();
    for (auto & it : lane_info) {
        if (std::to_string(it.first).size() != normal_lane_str_len) {
            continue;
        }
        std::shared_ptr<Lane_Info> this_lane_info = it.second;
        std::map<std::string, std::map<int, int>> this_lane_local_static_congestion_state = lane_local_static_congestion_state[it.first];
        std::map<std::string, std::map<int, int>> this_total_statistics = total_statistics[it.first];//yk:wrong in here, map<int, int> gaiwei map<int, double>
        std::map<int, int> this_lane_flow = total_statistics[it.first]["lane_flow"];//yk:wrong in here too, map<int, int> gaiwei map<int, double>
        std::map<std::string, std::shared_ptr<Mean_V_Info>> this_lane_mean_v_info_by_section = lane_mean_v_info_by_section[it.first];//yk:wrong in here also

        std::shared_ptr<EventList> event = std::make_shared<EventList>();
        if (event_sign["save_event"])
        {
            event->laneNo = lane_fix_info[it.first]->lane_output_name;
            event->laneType = logical_calculation(this_lane_info->is_emergency_lane, this_lane_info->is_non_motorway);
            event->laneDirection = this_lane_info->angle_to_north;
            event->laneSpeedLimit = this_lane_info->max_v * 3.6;
            event->stationNum = pc_num;

        //    std::map<std::string, int> temp = {};
            for (int i = 1; i <  + 1; ++i) {
                event->stationCongestionList.push_back({i, this_lane_local_static_congestion_state["null"][i]});
            }

            event->followPercent = this_total_statistics["follow_car_percent"][0] * 100;
            event->timeOccupancy = this_total_statistics["time_occupancy"][0] * 100;
            event->headSpaceAvg = lane_mean_space_headway_info[it.first]->mean_sh;
            event->carFlow = this_lane_flow[classType->car[0]];
            event->truckFlow = this_lane_flow[classType->truck[0]];
            event->busFLow = this_lane_flow[classType->bus[0]];
            event->mediumBusFlow = this_lane_flow[classType->minibus[0]];
            event->carSpeedAvg = this_lane_mean_v_info_by_section[std::to_string(classType->car[0])]->mean_v;
            event->truckSpeedAvg = this_lane_mean_v_info_by_section[std::to_string(classType->truck[0])]->mean_v;
            event->busSpeedAvg = this_lane_mean_v_info_by_section[std::to_string(classType->bus[0])]->mean_v;
            event->mediumBusSpeedAvg = this_lane_mean_v_info_by_section[std::to_string(classType->minibus[0])]->mean_v;
            b4_res->eventList.push_back(event);
        }
        std::vector<std::vector<double>> temp1;
        for (int it = 0;it <= pc_num;++it)
        {
            temp1.push_back({double(it),this_lane_local_static_congestion_state["null"][it],0});
        }

        // B4_context.push_back({
        //                         lane_fix_info[it.first]->lane_output_name,
        //                         double (logical_calculation(this_lane_info->is_emergency_lane, this_lane_info->is_non_motorway)),
        //                         this_lane_info->angle_to_north,
        //                         this_lane_info->max_v * 3.6,
        //                         static_cast<double>(pc_num),
        //                         0,
        //                         0, //temp1  6
        //                         double (this_total_statistics["follow_car_percent"][0]),
        //                         double (this_total_statistics["time_occupancy"][0]),
        //                         lane_mean_space_headway_info[it.first]->mean_sh,  //b4wrong
        //                         double (this_lane_flow[classType->car[0]]), //10
        //                         double (this_lane_flow[classType->largetruck[0]]),
        //                         double (this_lane_flow[classType->bus[0]]),
        //                         double (this_lane_flow[classType->minibus[0]]),
        //                         0, //14
        //                         double (this_lane_mean_v_info_by_section[std::to_string(classType->car[0])]->mean_v),
        //                         double (this_lane_mean_v_info_by_section[std::to_string(classType->largetruck[0])]->mean_v), //16
        //                         double (this_lane_mean_v_info_by_section[std::to_string(classType->bus[0])]->mean_v),
        //                         double (this_lane_mean_v_info_by_section[std::to_string(classType->minibus[0])]->mean_v),
        //                         double (Mean_V_Info::default_mean_v),
        //                         double (this_lane_flow[classType->truck[0]]), //20
        //                         double (this_lane_flow[classType->littletruck[0]]),
        //                         double (this_lane_mean_v_info_by_section[std::to_string(classType->truck[0])]->mean_v),
        //                         double (this_lane_mean_v_info_by_section[std::to_string(classType->littletruck[0])]->mean_v),
        //                         double (dir_num_switch(double(int(it.first)/100))),
        //                      });

        B4_context.push_back(std::make_shared<B4_context_info>(
                        lane_fix_info[it.first]->lane_output_name,
                        double (logical_calculation(this_lane_info->is_emergency_lane, this_lane_info->is_non_motorway)),
                        this_lane_info->angle_to_north,
                        this_lane_info->max_v * 3.6,
                        static_cast<double>(pc_num),
                        0,
                        temp1, //temp1  6 need to do
                        double (this_total_statistics["follow_car_percent"][0]),
                        double (this_total_statistics["time_occupancy"][0]),
                        lane_mean_space_headway_info[it.first]->mean_sh,  //b4wrong
                        double (this_lane_flow[classType->car[0]]), //10
                        double (this_lane_flow[classType->largetruck[0]]),
                        double (this_lane_flow[classType->bus[0]]),
                        double (this_lane_flow[classType->minibus[0]]),
                        0, //14
                        double (this_lane_mean_v_info_by_section[std::to_string(classType->car[0])]->mean_v),
                        double (this_lane_mean_v_info_by_section[std::to_string(classType->largetruck[0])]->mean_v), //16
                        double (this_lane_mean_v_info_by_section[std::to_string(classType->bus[0])]->mean_v),
                        double (this_lane_mean_v_info_by_section[std::to_string(classType->minibus[0])]->mean_v),
                        double (Mean_V_Info::default_mean_v),
                        double (this_lane_flow[classType->truck[0]]), //20
                        double (this_lane_flow[classType->littletruck[0]]),
                        double (this_lane_mean_v_info_by_section[std::to_string(classType->truck[0])]->mean_v),
                        double (this_lane_mean_v_info_by_section[std::to_string(classType->littletruck[0])]->mean_v),
                        double (dir_num_switch(double(int(it.first)/100)))
                        ));

        B4_context_for_save.push_back({
                                        double (it.first),
                                        double (this_lane_flow[classType->car[0]]),
                                        double (this_lane_flow[classType->car[0]] + this_lane_flow[classType->car[1]]),
                                        double (this_lane_flow[classType->truck[0]] + this_lane_flow[classType->truck[1]] + this_lane_flow[classType->truck[2]]),
                                        double (this_lane_flow[classType->car[0]] + this_lane_flow[classType->bus[1]] + this_lane_flow[classType->bus[1]] + this_lane_flow[classType->truck[0]] + this_lane_flow[classType->truck[1]] + this_lane_flow[classType->truck[2]])
                                        });
    }

    save_b4[B4_cnt] = b4_res;
    event_res = std::make_shared<Event_Output>(0, 0, 0, 0, 0, 0, B4_cnt, normal_lane_num, 0, B4_context);
//    B4_context_for_interface=B4_context;
//    B4_context_congestion_state_for_interface=B4_context_congestion_state;
//    B4_cnt_for_interface=B4_cnt;
//    normal_lane_num_for_interface=normal_lane_num;

    if (event_sign["save_event"])
    {
        std::string otherStyleTimeNameNow = ops::GetLocalTimeWithMs().substr(0, 10);
        std::string otherStyleTimeName = ops::TimeStampToString_sec(cur_time).substr(0, 10);

        std::string path = m_config_path + "/save_event_data/";
        if (access(path.c_str(), 0) == -1) {
            mkdir(path.c_str(), S_IRWXU);
        }   
        std::string name = path + "save_b4_rawdata_" + otherStyleTimeName + "_saved_in_" + otherStyleTimeNameNow + ".json";
        json savejson;

        for (auto info : B4_context_for_save)
        {
            savejson[std::to_string(info[0])] = info;
        }

        std::ofstream output(name, std::ios::app);
        output << savejson << std::endl;
    }
}

//void Traffic_Flow::get_B4(std::shared_ptr<Event_Output> &event_res) {
//    long t_now = ops::getTimeStamp();
//    B4_cnt += 1;
//
//    std::shared_ptr<B4_res> b4_res = std::make_shared<B4_res>();
//    if (event_sign["save_event"])
//    {
//        std::string otherStyleTime = ops::GetLocalTimeWithMs().substr(0, 19);
//        std::string otherStyleTimeBef = ops::GetLocalTimeWithMs(60).substr(0, 19);
//        b4_res->startTimeStamp = otherStyleTimeBef;
//        b4_res->endTimeStamp = otherStyleTime;
//        b4_res->laneNum = lane_info.size();
//    }
//    std::vector<std::vector<double>> B4_context;
//    std::vector<std::vector<std::vector<double>>> B4_context_congestion_state;
//
//    int normal_lane_num = lane_info.size() - turn_left_info.size();
//    for (auto & it : lane_info) {
//        if (std::to_string(it.first).size() != normal_lane_str_len) {
//            continue;
//        }
//        std::shared_ptr<Lane_Info> this_lane_info = it.second;
//        std::map<std::string, std::map<int, int>> this_lane_local_static_congestion_state = lane_local_static_congestion_state[it.first];
//        std::map<std::string, std::map<int, int>> this_total_statistics = total_statistics[it.first];//yk:wrong in here
//        std::map<int, int> this_lane_flow = total_statistics[it.first]["lane_flow"];//yk:wrong in here too
//        std::map<std::string, std::shared_ptr<Mean_V_Info>> this_lane_mean_v_info_by_section = lane_mean_v_info_by_section[it.first];//yk:wrong in here also
//
//        std::shared_ptr<EventList> event = std::make_shared<EventList>();
//        if (event_sign["save_event"])
//        {
//            event->laneNo = lane_fix_info[it.first]->lane_output_name;
//            event->laneType = logical_calculation(this_lane_info->is_emergency_lane, this_lane_info->is_non_motorway);
//            event->laneDirection = this_lane_info->angle_to_north;
//            event->laneSpeedLimit = this_lane_info->max_v * 3.6;
//            event->stationNum = ;
//
//            std::map<std::string, int> temp = {};
//            for (int i = 1; i <  + 1; ++i) {
//                event->stationCongestionList.push_back({i, this_lane_local_static_congestion_state["null"][i]});
//            }
//            event->followPercent = this_total_statistics["follow_car_percent"][0] * 100;
//            event->timeOccupancy = this_total_statistics["time_occupancy"][0] * 100;
//            event->headSpaceAvg = lane_mean_space_headway_info[it.first]->mean_sh;
//            event->carFlow = this_lane_flow[classType->car[0]];
//            event->truckFlow = this_lane_flow[classType->truck[0]];
//            event->busFLow = this_lane_flow[classType->bus[0]];
//            event->mediumBusFlow = this_lane_flow[classType->minibus[0]];
//            event->carSpeedAvg = this_lane_mean_v_info_by_section[std::to_string(classType->car[0])]->mean_v;
//            event->truckSpeedAvg = this_lane_mean_v_info_by_section[std::to_string(classType->truck[0])]->mean_v;
//            event->busSpeedAvg = this_lane_mean_v_info_by_section[std::to_string(classType->bus[0])]->mean_v;
//            event->mediumBusSpeedAvg = this_lane_mean_v_info_by_section[std::to_string(classType->minibus[0])]->mean_v;
//            b4_res->eventList.push_back(event);
//        }
//
//        B4_context.push_back({
//           lane_fix_info[it.first]->lane_output_name,
//           double (logical_calculation(this_lane_info->is_emergency_lane, this_lane_info->is_non_motorway)),
//           this_lane_info->angle_to_north,
//           this_lane_info->max_v * 3.6,
//           static_cast<double>(),
//           0,
//           0,
////            [[pc_id, this_lane_local_static_congestion_state[pc_id].value, 0] for pc_id in range(1, self. + 1)]
//           double (this_total_statistics["follow_car_percent"][0] * 100),
//           double (this_total_statistics["time_occupancy"][0] * 100),
//           lane_mean_space_headway_info[it.first]->mean_sh,
//           double (this_lane_flow[classType->car[0]]),
//           double (this_lane_flow[classType->largetruck[0]]),
//           double (this_lane_flow[classType->bus[0]]),
//           double (this_lane_flow[classType->minibus[0]]),
//           0,
//           double (this_lane_mean_v_info_by_section[std::to_string(classType->car[0])]->mean_v),
//           double (this_lane_mean_v_info_by_section[std::to_string(classType->largetruck[0])]->mean_v),
//           double (this_lane_mean_v_info_by_section[std::to_string(classType->bus[0])]->mean_v),
//           double (this_lane_mean_v_info_by_section[std::to_string(classType->minibus[0])]->mean_v),
//           double (Mean_V_Info::default_mean_v),
//           double (this_lane_flow[classType->truck[0]]),
//           double (this_lane_flow[classType->littletruck[0]]),
//           double (this_lane_mean_v_info_by_section[std::to_string(classType->truck[0])]->mean_v),
//           double (this_lane_mean_v_info_by_section[std::to_string(classType->littletruck[0])]->mean_v),
//         });
//
//        std::vector<std::vector<double>> temp;
//        for (int i = 1; i <  + 1; ++i) {
//            temp.push_back({static_cast<double>(i), static_cast<double>(this_lane_local_static_congestion_state["null"][i]), 0});//yk:no wrong in B4_context_congestion_state.
//        }
//        B4_context_congestion_state.push_back(temp);
//    }
//    event_res = std::make_shared<Event_Output>(0, 0, 0, 0, 0, 0, B4_cnt, normal_lane_num, 0, B4_context, B4_context_congestion_state);
//    if (event_sign["save_event"])
//    {
//        std::string otherStyleTimeNameNow = ops::GetLocalTimeWithMs().substr(0, 10);
//        std::string otherStyleTimeName = ops::TimeStampToString_sec(cur_time).substr(0, 10);
//        std::string name = "/data/event-alg/save_b4_rawdata_" + otherStyleTimeName + "_saved_in_" + otherStyleTimeNameNow + ".yaml";    // JSON --> YAML
////        std::ofstream yaml_out(name);
//        // TODO: save to json
//    }
//}

void Traffic_Flow::get_event_output_for_B5() {
    all_event_output_for_B5.clear();
    has_new_event = false;
    judge_new_event(cur_illegal_stop, history_illegal_stop);
    one_event_info_for_B5(cur_illegal_stop, history_illegal_stop, Abnormal_Class_Index::illegal_stop);

    judge_new_event(cur_retrograde_for_non_motor, history_retrograde_for_non_motor);
    one_event_info_for_B5(cur_retrograde_for_non_motor, history_retrograde_for_non_motor, Abnormal_Class_Index::retrograde_for_non_motor);


    judge_new_event(cur_retrograde_for_motor, history_retrograde_for_motor);
    one_event_info_for_B5(cur_retrograde_for_motor, history_retrograde_for_motor, Abnormal_Class_Index::retrograde_for_motor);

    judge_new_event(cur_occupy_dedicated_lane, history_occupy_dedicated_lane);
    one_event_info_for_B5(cur_occupy_dedicated_lane, history_occupy_dedicated_lane, Abnormal_Class_Index::occupy_dedicated_lane);

    // judge_new_event(cur_people_occupy_motor_lane, history_people_occupy_motor_lane);
    // one_event_info_for_B5(cur_people_occupy_motor_lane, history_people_occupy_motor_lane, Abnormal_Class_Index::people_occupy_motor_lane);

    // judge_new_event(cur_non_motor_occupy_motor_lane, history_non_motor_occupy_motor_lane);
    // one_event_info_for_B5(cur_non_motor_occupy_motor_lane, history_non_motor_occupy_motor_lane, Abnormal_Class_Index::non_motor_occupy_motor_lane);

    judge_new_event(cur_not_following_lane_guide, history_not_following_lane_guide);
    one_event_info_for_B5(cur_not_following_lane_guide, history_not_following_lane_guide, Abnormal_Class_Index::not_following_lane_guide);

    judge_new_event(cur_occupy_emergency_lane, history_occupy_emergency_lane);
    one_event_info_for_B5(cur_occupy_emergency_lane, history_occupy_emergency_lane, Abnormal_Class_Index::illegal_stop);

    judge_new_event(cur_speeding_for_motor, history_speeding_for_motor);
    one_event_info_for_B5(cur_speeding_for_motor, history_speeding_for_motor, Abnormal_Class_Index::speeding_for_motor);

    judge_new_event(cur_speeding_for_non_motor, history_speeding_for_non_motor);
    one_event_info_for_B5(cur_speeding_for_non_motor, history_speeding_for_non_motor, Abnormal_Class_Index::speeding_for_non_motor);

    judge_new_event(cur_stroll, history_stroll);
    one_event_info_for_B5(cur_stroll, history_stroll, Abnormal_Class_Index::stroll);

    judge_new_event(cur_cross_line, history_cross_line);
    one_event_info_for_B5(cur_cross_line, history_cross_line, Abnormal_Class_Index::cross_line);

    judge_new_event(cur_cross_lane_for_non_motor, history_cross_lane_for_non_motor);
    one_event_info_for_B5(cur_cross_lane_for_non_motor, history_cross_lane_for_non_motor, Abnormal_Class_Index::cross_lane_for_non_motor);

    judge_new_event(cur_cross_lane_for_people, history_cross_lane_for_people);
    one_event_info_for_B5(cur_cross_lane_for_people, history_cross_lane_for_people, Abnormal_Class_Index::cross_lane_for_people);

    judge_new_event(cur_cross_lane_for_motor, history_cross_lane_for_motor);
    one_event_info_for_B5(cur_cross_lane_for_motor, history_cross_lane_for_motor, Abnormal_Class_Index::cross_lane_for_motor);

    judge_new_event(cur_run_the_red_light_for_motor, history_run_the_red_light_for_motor);
    one_event_info_for_B5(cur_run_the_red_light_for_motor, history_run_the_red_light_for_motor, Abnormal_Class_Index::run_the_red_light_for_motor);

    judge_new_event(cur_run_the_red_light_for_people, history_run_the_red_light_for_people);
    one_event_info_for_B5(cur_run_the_red_light_for_people, history_run_the_red_light_for_people, Abnormal_Class_Index::run_the_red_light_for_people);

    judge_new_event(cur_occupy_bus_lane, history_occupy_bus_lane);
    one_event_info_for_B5(cur_occupy_bus_lane, history_occupy_bus_lane, Abnormal_Class_Index::occupy_bus_lane);

    // judge_new_event(cur_change_lanes_illegal, history_change_lanes);
    // one_event_info_for_B5(cur_change_lanes_illegal, history_change_lanes, Abnormal_Class_Index::change_lanes);

    judge_new_event(cur_spills, history_spills);
    one_event_info_for_B5(cur_spills, history_spills, Abnormal_Class_Index::spills);

    judge_new_event(cur_congestion, history_congestion);
    one_event_info_for_B5(cur_congestion, history_congestion, Abnormal_Class_Index::congestion);

    judge_new_event(cur_accident, history_accident);
    one_event_info_for_B5(cur_accident, history_accident, Abnormal_Class_Index::accident);

    judge_new_event(cur_big_car_in_dedicated_lane, history_big_car_in_dedicated_lane);
    one_event_info_for_B5(cur_big_car_in_dedicated_lane, history_big_car_in_dedicated_lane, Abnormal_Class_Index::big_car_in_dedicated_lane);

    judge_new_event(cur_traverse, history_traverse);
    one_event_info_for_B5(cur_traverse, history_traverse, Abnormal_Class_Index::traverse_the_diversion_zone);

    judge_new_event(cur_stop_over_terminateline, history_stop_over_terminateline);
    one_event_info_for_B5(cur_stop_over_terminateline, history_stop_over_terminateline, Abnormal_Class_Index::stop_over_terminate_line);

    // for motor
    // judge_new_event(cur_stop_over_terminateline_for_motor, history_stop_over_terminateline_for_motor);
    // one_event_info_for_B5(cur_stop_over_terminateline_for_motor, history_stop_over_terminateline_for_motor, Abnormal_Class_Index::stop_over_terminate_line_for_motor);

}
std::vector<double> Traffic_Flow::find_right_video_time(const int &flag, const int &frame_id, const int &id, const int &pc_id) {
    int index = -1;
    std::vector<double> result;
    int length_info = last_id_info_0.size();
    if (length_info >= 5)
    {
        if (flag)
        {
            for (int i = 0; i < length_info; ++i)
            {
                if (last_id_info_0[i].count(id))
                {
                    auto temp = std::abs(frame_id - last_id_info_0[i][id]->cur_global_frameid);
                    if (temp > 0 and temp <= 2)
                    {
                        int cur_cam_id = last_id_info_0[i][id]->cam_id;
                        if (last_id_info_0[i][id]->cam_id_time_dict.count(cur_cam_id) and last_id_info_0[i][id]->cam_id_time_dict[cur_cam_id] > 1262275200)
                        {
                            index = i;
                            if (int (std::abs(frame_id - last_id_info_0[i][id]->cur_global_frameid)) == 1)
                            {
                                break;
                            }
                        }
                    }
                }
            }
        }
        else
        {
            int dist_to_base = 10000;
            for (int i = 0; i < length_info; ++i)
            {
                if (last_id_info_0[i].count(id))
                {
                    int cur_pc_id = last_id_info_0[i][id]->pc_id;
                    if (pc_id - cur_pc_id > 1)
                    {
                        continue;
                    }
                    double cur_global_frameid = last_id_info_0[i][id]->cur_global_frameid;
                    if (cur_global_frameid - frame_id > 0)
                    {
                        break;
                    }
                    int cur_cam_id = last_id_info_0[i][id]->cam_id;
                    if (cur_cam_id == 3 and last_id_info_0[i][id]->cam_id_time_dict[cur_cam_id] > 1262275200)
                    {
                        double pc_x = last_id_info_0[i][id]->pc_x;
                        double pc_y = last_id_info_0[i][id]->pc_y;
                        double dist = std::sqrt(pc_x * pc_x + pc_y * pc_y);
                        double deltDist = std::abs(dist - 20);
                        if (deltDist <= dist_to_base)
                        {
                            dist_to_base = deltDist;
                            index = i;
                        }
                    }
                }
            }
        }
        if (index != -1)
        {
            result.push_back(flag);
            result.push_back(last_id_info_0[index][id]->longitude);
            result.push_back(last_id_info_0[index][id]->latitude);
            result.push_back(last_id_info_0[index][id]->left_top_px);
            result.push_back(last_id_info_0[index][id]->left_top_py);
            result.push_back(last_id_info_0[index][id]->right_bottom_px);
            result.push_back(last_id_info_0[index][id]->right_bottom_py);
            result.push_back(last_id_info_0[index][id]->pc_id);
            result.push_back(last_id_info_0[index][id]->cam_id);
            result.push_back(last_id_info_0[index][id]->head_camera_time);
            result.push_back(last_id_info_0[index][id]->body_camera_time);
            result.push_back(last_id_info_0[index][id]->tail_camera_time);
            result.push_back(last_id_info_0[index][id]->cur_global_frameid);
            result.push_back(last_id_info_0[index][id]->cur_id_time);

        }
    }
    return result;
}
void Traffic_Flow::one_event_info_for_B5(std::vector<int> &cur_event,
                                         std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> &history_infos,
                                         const int &abnormal_class_index) {
    B5_event.clear();
    if (has_new_event)
    {
        for (auto & id : cur_event)
        {
            if (!history_infos.count(id))
            {
                continue;
            }
            std::shared_ptr<Event_Continue_Info> this_history_info = history_infos[id].at(history_infos[id].size() - 1);


            if (history_infos[id].size() - 1 < 0) return;
            if (!this_history_info->eventInfoList.size())
            {
                double cur_x = this_history_info->x;
                double cur_y = this_history_info->y;
                double event_head_Hcamera_time = -1;
                double event_head_Bcamera_time = -1;
                double event_head_Tcamera_time = -1;
                double event_head_lon = -1;
                double event_head_lat = -1;
                double event_head_pc_id = -1;
                double event_head_camera_id = -1;
                double event_head_global_frame_time = -1;
                double event_head_global_frame_id = -1;
                double event_head_left_top_px = -1;
                double event_head_left_top_py = -1;
                double event_head_right_bottom_px = -1;
                double event_head_right_bottom_py = -1;
                double delt_x = -1;
                double delt_y = -1;
                double dist_last_cur = -1;
                double event_tail_Hcamera_time = -1;
                double event_tail_Bcamera_time = -1;
                double event_tail_Tcamera_time = -1;
                double event_tail_lon = -1;
                double event_tail_lat = -1;
                double event_tail_pc_id = -1;
                double event_tail_camera_id = -1;
                double event_tail_global_frame_time = -1;
                double event_tail_global_frame_id = -1;
                double event_tail_left_top_px  = -1;
                double event_tail_left_top_py  = -1;
                double event_tail_right_bottom_px = -1;
                double event_tail_right_bottom_py = -1;


                for (int i = last_id_info_0.size() - 1; i >= 0; --i) {
                    if (last_id_info_0[i].count(id))
                    {
                        double event_start_time = cur_time - this_history_info->global_frame_time < 5 * 1000 ? this_history_info->global_frame_time : cur_time - 5 * 1000;
                        if (last_id_info_0[i][id]->cur_id_time >= event_start_time)
                        {
                            continue;
                        }
                        event_head_Hcamera_time = last_id_info_0[i][id]->head_camera_time;
                        event_head_Bcamera_time = last_id_info_0[i][id]->body_camera_time;
                        event_head_Tcamera_time = last_id_info_0[i][id]->tail_camera_time;
                        event_head_lon = last_id_info_0[i][id]->longitude;
                        event_head_lat = last_id_info_0[i][id]->latitude;
                        event_head_pc_id = last_id_info_0[i][id]->pc_id;
                        event_head_camera_id = last_id_info_0[i][id]->cam_id;
                        event_head_global_frame_time = last_id_info_0[i][id]->cur_id_time;
                        event_head_global_frame_id = last_id_info_0[i][id]->cur_global_frameid;
                        event_head_left_top_px = last_id_info_0[i][id]->left_top_px;
                        event_head_left_top_py = last_id_info_0[i][id]->left_top_py;
                        event_head_right_bottom_px = last_id_info_0[i][id]->right_bottom_px;
                        event_head_right_bottom_py = last_id_info_0[i][id]->right_bottom_py;
                        delt_x = last_id_info_0[i][id]->x - cur_x;
                        delt_y = last_id_info_0[i][id]->y - cur_y;
                        dist_last_cur = std::sqrt(delt_x * delt_x + delt_y * delt_y);
                        if (dist_last_cur > 15)
                        {
                            break;
                        }
                    }
                }


                if (int (event_head_lon) == 0)
                {
                    continue;
                }

                double change_start_time = this_history_info->start_time;
                double change_end_time = this_history_info->end_time;
                double change_middle_time = (change_start_time + change_end_time) / 2;
                if (change_end_time - change_start_time > 10 * 1000)
                {
                    change_end_time += 10 * 1000;
                    change_middle_time = (change_start_time + change_end_time) / 2;
                }

                std::map<int, double> event_head;
                event_head[1] = event_head_Hcamera_time;
                event_head[2] = event_head_Bcamera_time;
                event_head[3] = event_head_Tcamera_time;

                // std::vector<double> event_head_info =
                //         {0, event_head_lon, event_head_lat, event_head_left_top_px, event_head_left_top_py,
                //          event_head_right_bottom_px, event_head_right_bottom_py, event_head_pc_id,
                //          event_head_camera_id, event_head_Hcamera_time, event_head_Bcamera_time,
                //          event_head_Tcamera_time, event_head_global_frame_id, event_head_global_frame_time};

                //suzhousanqi
                //  std::vector<double> event_head_info =
                //         {0, event_head_lon, event_head_lat, event_head_left_top_px, event_head_left_top_py,
                //          event_head_right_bottom_px, event_head_right_bottom_py, event_head_pc_id,
                //          event_head_camera_id, change_start_time, change_start_time,
                //          change_start_time, event_head_global_frame_id, change_start_time};


                //quanxilukou
                 std::vector<double> event_head_info =
                        {0, event_head_lon, event_head_lat, event_head_left_top_px, event_head_left_top_py,
                         event_head_right_bottom_px, event_head_right_bottom_py, event_head_pc_id,
                         event_head_camera_id, event_head_Hcamera_time, event_head_Bcamera_time,
                         event_head_Tcamera_time, event_head_global_frame_id, change_start_time};

                // std::vector<double> event_head_result = find_right_video_time(0, int (event_head_global_frame_id), id, int(event_head_pc_id));
                // if (event_head_result.size() > 0)
                // {
                //     event_head_info = event_head_result;
                // }
                event_tail_Hcamera_time = cur_id_info[id]->head_camera_time;
                event_tail_Bcamera_time = cur_id_info[id]->body_camera_time;
                event_tail_Tcamera_time = cur_id_info[id]->tail_camera_time;
                std::map<int, double> event_tail;
                event_tail[1] = event_tail_Hcamera_time;
                event_tail[2] = event_tail_Bcamera_time;
                event_tail[3] = event_tail_Tcamera_time;

                event_tail_lon = cur_id_info[id]->longitude;
                event_tail_lat = cur_id_info[id]->latitude;
                event_tail_pc_id = cur_id_info[id]->pc_id;
                event_tail_camera_id = cur_id_info[id]->cam_id;
                event_tail_global_frame_time = cur_id_info[id]->cur_id_time;
                event_tail_global_frame_id = cur_id_info[id]->cur_global_frameid;
                event_tail_left_top_px = cur_id_info[id]->left_top_px;
                event_tail_left_top_py = cur_id_info[id]->left_top_py;
                event_tail_right_bottom_px = cur_id_info[id]->right_bottom_px;
                event_tail_right_bottom_py = cur_id_info[id]->right_bottom_py;

                std::map<int, double> event_mid;
                event_tail[1] = this_history_info->event_mid_Hcamera_time;
                event_tail[2] = this_history_info->event_mid_Bcamera_time;
                event_tail[3] = this_history_info->event_mid_Tcamera_time;
                // std::vector<double> event_mid_info = {
                //         1,
                //         this_history_info->event_mid_lon,
                //         this_history_info->event_mid_lat,
                //         this_history_info->left_top_px,
                //         this_history_info->left_top_py,
                //         this_history_info->right_bottom_px,
                //         this_history_info->right_bottom_py,
                //         double (this_history_info->pc_id),
                //         double (this_history_info->camera_id),
                //         this_history_info->event_mid_Hcamera_time,
                //         this_history_info->event_mid_Bcamera_time,
                //         this_history_info->event_mid_Tcamera_time,
                //         double (this_history_info->global_frame_id),
                //         double (this_history_info->global_frame_time)
                // };

                // suzhousanqi
                // std::vector<double> event_mid_info = {
                //         1,
                //         this_history_info->event_mid_lon,
                //         this_history_info->event_mid_lat,
                //         this_history_info->left_top_px,
                //         this_history_info->left_top_py,
                //         this_history_info->right_bottom_px,
                //         this_history_info->right_bottom_py,
                //         double (this_history_info->pc_id),
                //         double (this_history_info->camera_id),
                //         change_middle_time,
                //         change_middle_time,
                //         change_middle_time,
                //         double (this_history_info->global_frame_id),
                //         change_middle_time
                // };

                // quanxilukou
                std::vector<double> event_mid_info = {
                        1,
                        this_history_info->event_mid_lon,
                        this_history_info->event_mid_lat,
                        this_history_info->left_top_px,
                        this_history_info->left_top_py,
                        this_history_info->right_bottom_px,
                        this_history_info->right_bottom_py,
                        double (this_history_info->pc_id),
                        double (this_history_info->camera_id),
                        this_history_info->event_mid_Hcamera_time,
                        this_history_info->event_mid_Bcamera_time,
                        this_history_info->event_mid_Tcamera_time,
                        double (this_history_info->global_frame_id),
                        change_middle_time
                };
                // if ((this_history_info->camera_id == 1 or this_history_info->camera_id == 2 or this_history_info->camera_id == 3) and
                //     event_mid[this_history_info->camera_id] < 1262275200)
                // {
                //     std::vector<double> event_mid_result = find_right_video_time(1, this_history_info->global_frame_id, id, this_history_info->pc_id);
                //     if (event_mid_result.size())
                //     {
                //         event_mid_info = event_mid_result;
                //     }
                // }
                // std::vector<double> event_tail_info = {
                //         2,
                //         event_tail_lon,
                //         event_tail_lat,
                //         event_tail_left_top_px,
                //         event_tail_left_top_py,
                //         event_tail_right_bottom_px,
                //         event_tail_right_bottom_py,
                //         event_tail_pc_id,
                //         event_tail_camera_id,
                //         event_tail_Hcamera_time,
                //         event_tail_Bcamera_time,
                //         event_tail_Tcamera_time,
                //         event_tail_global_frame_id,
                //         event_tail_global_frame_time
                // };

                // suzhousanqi
                // std::vector<double> event_tail_info = {
                //         2,
                //         event_tail_lon,
                //         event_tail_lat,
                //         event_tail_left_top_px,
                //         event_tail_left_top_py,
                //         event_tail_right_bottom_px,
                //         event_tail_right_bottom_py,
                //         event_tail_pc_id,
                //         event_tail_camera_id,
                //         change_end_time,
                //         change_end_time,
                //         change_end_time,
                //         event_tail_global_frame_id,
                //         change_end_time
                // };

                //quanxilukou
                std::vector<double> event_tail_info = {
                        2,
                        event_tail_lon,
                        event_tail_lat,
                        event_tail_left_top_px,
                        event_tail_left_top_py,
                        event_tail_right_bottom_px,
                        event_tail_right_bottom_py,
                        event_tail_pc_id,
                        event_tail_camera_id,
                        event_tail_Hcamera_time,
                        event_tail_Bcamera_time,
                        event_tail_Tcamera_time,
                        event_tail_global_frame_id,
                        change_end_time
                };
                // if ((int (event_tail_camera_id) == 1 or int (event_tail_camera_id) == 2 or int (event_tail_camera_id) == 3) and event_tail[int (event_tail_camera_id)] < 1262275200)
                // {
                //     std::vector<double> event_tail_result = find_right_video_time(2, int(event_head_global_frame_id), id, int (event_tail_pc_id));
                //     if (event_tail_result.size())
                //     {
                //         event_tail_info = event_tail_result;
                //     }
                // }
                this_history_info->eventInfoList = {event_head_info, event_mid_info, event_tail_info};
            }
            long second = static_cast<long>(this_history_info->start_time);
            int miscros = (this_history_info->start_time - second) * 1000000;
            int eventInfoNum = this_history_info->eventInfoList.size();
            double end_time_change = this_history_info->end_time - this_history_info->start_time >10 * 1000 ? this_history_info->start_time + 10 * 1000 : this_history_info->end_time;
            // double coincidence = this_history_info->coincidence;
//            double eventInfoList = this_history_info->eventInfoList;
            // std::vector<std::vector<double>> eventInfoList = this_history_info->eventInfoList;
            // if (coincidence < 0)
            // {
            //     coincidence = 0;
            // }
            if (abnormal_class_index == 16)
            {
                // float lane = -1;
                // if (cur_id_lane.count(id)){
                //     if (cur_id_lane[id] >  0){
                //         lane = lane_fix_info[cur_id_lane[id]]->lane_output_name;
                //     }
                // }
//                print('事件编号', '事件类型', '基站号', '目标车道', '事件涉及目标个数', 'id', '经度', '纬度', '开始时间s', '开始时间ms', 0)
//                 B5_event.push_back(std::make_shared<B5_event_info>(this_history_info->event_no,   //yk:有待优化
//                                                                    abnormal_class_index,
//                                                                    cur_id_info[id]->pc_id,
//                                                                    cur_id_info[id]->w,
//                                                                    cur_id_info[id]->l,
//                                                                    cur_id_info[id]->h,
//                                                                    cur_id_lane.count(id) ? lane_num_switch(cur_id_lane[id]) : lane_num_switch(0),
//                                                                    cur_id_lane.count(id) ? dir_num_switch(int(cur_id_lane[id] / 100)) : dir_num_switch(0),
//                                                                    1,
//                                                                    std::vector<std::vector<double>>{{(double)id, cur_id_info[id]->class_id, this_history_info->coincidence,  (double)eventInfoNum}},//yk:有待优化 id need to revise
// //                                                                   std::vector<std::vector<double>>{{double (id), coincidence, double (eventInfoNum)}},//yk:有待优化 id need to revise

//                                                                    this_history_info->longitude,
//                                                                    this_history_info->latitude,
//                                                                 //    second,//yk:有待优化
//                                                                 //    miscros,//yk:有待优化
//                                                                 //    0,

//                                                                    this_history_info->start_time,//yk:有待优化
//                                                                    end_time_change,
//                                                                    this_history_info->eventInfoList));
//                 history_infos.erase(id);
                continue;
            }
            else
            {
    
                // float lane = -1;
                // if (cur_id_lane.count(id)){
                //     if (cur_id_lane[id] > 0){
                //         lane = lane_fix_info[cur_id_lane[id]]->lane_output_name;
                //     }

                // }

                        B5_event.push_back(std::make_shared<B5_event_info>(
                        this_history_info->event_no,
                        abnormal_class_index,
                        cur_id_info[id]->pc_id,
                        cur_id_info[id]->w,
                        cur_id_info[id]->l,
                        cur_id_info[id]->h,
                        cur_id_lane.count(id) ? lane_num_switch(cur_id_lane[id]) : lane_num_switch(0),
                        cur_id_lane.count(id) ? dir_num_switch(int(cur_id_lane[id] / 100)) : dir_num_switch(0),
                        1,
                        // std::vector<std::vector<double>>{{(double)id, (double)eventInfoNum}},
                        std::vector<std::vector<double>>{{double (id), cur_id_info[id]->class_id, this_history_info->coincidence, double (eventInfoNum)}},
                        this_history_info->longitude,
                        this_history_info->latitude,
                        this_history_info->start_time,
                        end_time_change,
                        this_history_info->eventInfoList)); //yk:eventInfoList 有待优化
                }

        }
    }
    if (B5_event.size()) {
        all_event_output_for_B5.insert(all_event_output_for_B5.end(), B5_event.begin(), B5_event.end());  //yk:insert all_event_output_for_B5
    }
    has_new_event = false;

}
void Traffic_Flow::judge_new_event(std::vector<int> &cur_event,
                                   std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> &history_infos) {
    for (auto & id : cur_event)
    {
        if (history_infos[id].size() == 0)
        {
            return;
        }
        std::shared_ptr<Event_Continue_Info>this_history_info = history_infos[id].at(history_infos[id].size() - 1);
        int event_num = this_history_info->event_no;
        if (event_num % 100 == 0)
        {
            xt::xarray<int> temp = {event_num};
            xt::dump_npy(m_config_path + "/event_number.npy", temp);
            // xt::dump_npy(m_config_path + "/event_number_new.npy", temp);
        }

        // cjm 0925
        has_new_event = true;
        history_events[event_num] = this_history_info->end_time;
        history_all_event_output_for_B5[event_num] = std::make_shared<history_all_event_output_for_B5_info>(this_history_info->end_time, std::vector<double>{});

        // if (!history_events.count(event_num))
        // {
        //     has_new_event = true;
        //     history_events[event_num] = this_history_info->end_time;
        //     history_all_event_output_for_B5[event_num] = std::make_shared<history_all_event_output_for_B5_info>(this_history_info->end_time, std::vector<double>{});
        // }
        // else
        // {
        //     if (this_history_info->end_time - history_events[event_num] >= 5.0 * 1000)
        //     {

        //         has_new_event = true;
        //         history_events[event_num] = this_history_info->end_time;
        //         history_all_event_output_for_B5[event_num] = std::make_shared<history_all_event_output_for_B5_info>(this_history_info->end_time, std::vector<double>{});
        //     }
        // }
    }

    decltype(history_events)::iterator it;
    bool erase_flag = false;
    for (it=history_events.begin();it!=history_events.end();)  //?
    {
        if (cur_time - history_events[it->first] > 300 * 1000)   //ms -> s
        {
            history_events.erase(it++);
        }else{
            it++;
        }
    }
    del_long_time_miss_history(history_infos);
}
//删除太长时间未出现的历史记录
void Traffic_Flow::del_long_time_miss_history(std::map<int, std::vector<std::shared_ptr<Event_Continue_Info>>> & history_event) {
    std::set<int> long_time_miss_idxs;
    for (auto & it : history_event)
    {
        if (cur_time - it.second[it.second.size() - 1]->end_time > 10 * 1000)  
        {
            long_time_miss_idxs.insert(it.first);
        }
    }
    for(std::set<int> :: iterator it = long_time_miss_idxs.begin();it!=long_time_miss_idxs.end();it++){//set只能通过迭代器访问
        history_event.erase(*it);
    }
}

//std::shared_ptr<Event_Output> Traffic_Flow::get_B5(){
//    get_event_output_for_B5();
//    for (auto & event : all_event_output_for_B5){
//        if (history_all_event_output_for_B5.count(event->item0)){
//            history_all_event_output_for_B5[event->item0]->event = event;
//        }
//    }
//    std::vector<int> to_del;
//    decltype(history_all_event_output_for_B5)::iterator iter = history_all_event_output_for_B5.begin();
//    if (iter->first)
//    {
//        to_del.push_back(iter->first);
//    }
//    for (auto & event_no : to_del){
//        if (history_all_event_output_for_B5[event_no]->m_list.size() and history_all_event_output_for_B5[event_no]->m_list[1] == 16){
//            history_all_event_output_for_B5.erase(event_no);
//            continue;
//        }
//        if (cur_time - history_all_event_output_for_B5[event_no]->end_time >7*1){
//            if (history_all_event_output_for_B5[event_no]->m_list.size()){
//                history_all_event_output_for_B5[event_no]->m_list[history_all_event_output_for_B5[event_no]->m_list.size()-1] = cur_time;
////                all_event_output_for_B5.insert(all_event_output_for_B5.end(), history_all_event_output_for_B5[event_no]->m_list.begin(), history_all_event_output_for_B5[event_no]->m_list.end());
//                all_event_output_for_B5.push_back();
//            }
//            history_all_event_output_for_B5.erase(event_no);
//        }
//    }
//    std::shared_ptr<Event_Output> res = std::make_shared<Event_Output>(0, 0, 0, 0, all_event_output_for_B5.size(), 0, all_event_output_for_B5, 0);
//    return res;
//}

std::shared_ptr<Event_Output> Traffic_Flow::get_B5() {
    get_event_output_for_B5();

    for (auto & event : all_event_output_for_B5)
    {
        if (history_all_event_output_for_B5.count(event->item0))
        {
            history_all_event_output_for_B5[event->item0]->event = event;
        }
        else  
        {
            history_all_event_output_for_B5[event->item0] = std::make_shared<history_all_event_output_for_B5_info>();
            history_all_event_output_for_B5[event->item0]->event = event; // ?cjm
        }
    }
    std::vector<int> to_del;
        
    for (auto & it : history_all_event_output_for_B5)
    {
        //xcb note
        // if (it.second->m_list.size() > 1 and it.second->m_list[1] == 16)
        // {
        //     to_del.push_back(it.first);
        //     continue;
        // }

        // cjm 0824
        if (history_all_event_output_for_B5_all.count(it.first))
        {
            history_all_event_output_for_B5_all[it.first]->end_time = it.second->end_time;
            history_all_event_output_for_B5_all[it.first]->m_list = it.second->m_list;
            history_all_event_output_for_B5_all[it.first]->event = it.second->event;
        }
        else
        {
            history_all_event_output_for_B5_all[it.first] = std::make_shared<history_all_event_output_for_B5_info>();
            history_all_event_output_for_B5_all[it.first]->end_time = it.second->end_time;
            history_all_event_output_for_B5_all[it.first]->m_list = it.second->m_list;
            history_all_event_output_for_B5_all[it.first]->event = it.second->event;
        }


        // cjm 0829 Align the logic of the Python version
        if (it.second->event != NULL and it.second->event->item1 ==16) // bug 
        {
            to_del.push_back(it.first);
            continue;
        }


        // cjm 0829 Align the logic of the Python version
        if (cur_time - it.second->end_time > 7.0 * 1000) // 
        {   
            // cjm 0925 note
            // if (it.second->event != NULL)
            // {
            //     history_all_event_output_for_B5_all[it.first]->event->item13 = cur_time; // cjm 0824
            //     it.second->event->item13 = cur_time;
            //     all_event_output_for_B5.push_back(it.second->event);
            // }
            to_del.push_back(it.first);
        }
    }

    for (auto & i : to_del)
    {
        history_all_event_output_for_B5.erase(i);
    }
    std::shared_ptr<Event_Output> res = std::make_shared<Event_Output>(0, 0, 0, 0, all_event_output_for_B5.size(), 0, all_event_output_for_B5, 0);

    if (event_sign["save_event"])
    {
        std::string path = m_config_path + "/save_event_data/";
        if (access(path.c_str(), 0) == -1) {
            mkdir(path.c_str(), S_IRWXU);
        }
        std::string otherStyleTimeNameNow = ops::GetLocalTimeWithMs().substr(0, 13);
        std::string otherStyleTimeName = ops::TimeStampToString_sec(long (cur_time)).substr(0, 13);
        std::string name = path + "save_b5_rawdata_" + otherStyleTimeName + "_saved_in_" + otherStyleTimeNameNow + ".json";   // event data
        std::string name2 = path + "save_cur_lane_id_rawdata_" + otherStyleTimeName + "_saved_in_" + otherStyleTimeNameNow + ".json";
        json savejson2;
        json savejson;
        for (const auto& pair : cur_lane_id)
        {
            savejson2[std::to_string(pair.first)] = pair.second;
        }
        if (res->out_input4 != 0)
        {
            // std::vector<std::tuple<int,int, int, int, int, int, double, double, double, double, std::vector<std::vector<double>>, std::vector<double>>> json_result;
            // std::tuple<int,int, int, int, int, int, double, double, double, double, std::vector<std::vector<double>>, std::vector<double>>temp_tuple;


            std::vector<std::tuple<int,int, int, int, int, int, int, int, double, int, std::vector<std::vector<double>>, std::vector<double>>> json_result;
            std::tuple<int,int, int, int, int, int, int, int, double, int, std::vector<std::vector<double>>, std::vector<double>>temp_tuple;

            for(auto event : res->out_all_event_output_for_B5)
            {
                if (event->item1 != 0)
                {
                    std::vector<double>lwh = {event->item3, event->item4, event->item5};
                    temp_tuple = std::make_tuple(
                                                 event->item0,event->item1, event->item2, event->item6, event->item7, event->item8,
                                                 event->item9[0][0],event->item9[0][1],event->item9[0][2],event->item9[0][3],
                                                 event->eventInfoList,lwh);
                    json_result.push_back(temp_tuple);
                }
            }

            /*
            item0:event NO
            item1:event class
            item2:jizhan ID
            item3-5:l w h
            item6:lane no
            item7:big direction
            item8:1
            item9:[[object id, object class, zhixindu, 3]]
            item10:lontitude
            item11:latitude
            item12:event start time,
            item13:event end time

            */

            // void Traffic_Flow::detect_occupy_bus_lane(std::set<int> &all_unhit_id_continue_occupy_bus_lane) 
            // {
            //     for (auto & id : all_unhit_id_continue_occupy_bus_lane){
            //         id_continue_occupy_bus_lane_info.erase(id);
            //     }
            //     for (decltype(id_continue_occupy_bus_lane_info)::const_iterator it = id_continue_occupy_bus_lane_info.cbegin();
            //         it != id_continue_occupy_bus_lane_info.cend(); ++it) 
            //     {
            //         if (cur_id_lane.count(it->first) == 0)
            //         {
            //             continue;
            //         }
            //         if (lane_info[cur_id_lane[it->first]]->is_bus_lane == 0)
            //         {
            //             continue;
            //         }
            //         if (it->second->hit_count < 5)
            //         {
            //             continue;
            //         }
            //         cur_occupy_bus_lane.push_back(it->first);
            //     }
            // }
            
            if (json_result.size() != 0)
            {
                savejson["0"] = ops::TimeStampToString_sec(long (cur_time)).substr(0, 19);
                savejson["1"] = json_result;
                savejson["2"] = cur_lane_id;
                std::ofstream output(name, std::ios::app);
                output << savejson <<std::endl;
                std::ofstream output2(name2, std::ios::app);
                output2 << savejson2 << std::endl;
            }
        }

        // 0824 cjm
        // simple version
        int dict_pos = history_all_event_output_for_B5_all.size();
        if (dict_pos > 0)
        {
            std::string name3 = path + "history_all_event_output_for_B5_all"+ B5_all +".json";
            std::ofstream output3(name3, std::ios::ate);
            for(auto & key : history_all_event_output_for_B5_all)
            {
                if (key.second->event != NULL)
                {
                    std::vector<std::tuple<string,int, double, double, int, string, string>> json_result1;
                    std::tuple<string,int, double, double, int, string, string>temp_tuple1;
                    json savejson3;
                    temp_tuple1 = std::make_tuple(
                                                ops::TimeStampToString_sec(long (key.second->end_time)).substr(0, 19),
                                                key.second->event->item1, 
                                                key.second->event->item9[0][0],
                                                key.second->event->item9[0][1],
                                                key.second->event->item6,
                                                ops::TimeStampToString_sec(long (key.second->event->item12)).substr(0, 19),
                                                ops::TimeStampToString_sec(long (key.second->event->item13)).substr(0, 19)
                                                );
                    json_result1.push_back(temp_tuple1);
                    savejson3[std::to_string(key.second->event->item0)] = json_result1;
                    std::ofstream output4(name3, std::ios::app); // std::ios::app zhuijia; std::ios::out fugai
                    output4 << savejson3 << std::endl;
                }
            }
        }

        /*
        if (++frame >= 4997)
        {
            int dict_pos = history_all_event_output_for_B5_all.size();
            if (dict_pos > 0)
            {
                std::string name3 = path + "history_all_event_output_for_B5_all.json";

                for(auto & key : history_all_event_output_for_B5_all)
                {
                    std::vector<std::tuple<double,int,int, int, int, int, int, double, double, double, double, 
                        std::vector<std::vector<double>>, std::vector<double>, double, double, double, double>> json_result1;
                    std::tuple<double,int,int, int, int, int, int, double, double, double, double, 
                        std::vector<std::vector<double>>, std::vector<double>, double, double, double, double>temp_tuple1;
                    json savejson3;

                    // if (key.second->event->item1 != 0)
                    // {
                    std::vector<double>lwh = {key.second->event->item3, 
                                            key.second->event->item4, 
                                            key.second->event->item5};
                    temp_tuple1 = std::make_tuple(key.second->end_time,
                                                key.second->event->item0,
                                                key.second->event->item1, 
                                                key.second->event->item2, 
                                                key.second->event->item6, 
                                                key.second->event->item7, 
                                                key.second->event->item8,
                                                key.second->event->item9[0][0],
                                                key.second->event->item9[0][1],
                                                key.second->event->item9[0][2],
                                                key.second->event->item9[0][3],
                                                key.second->event->eventInfoList,
                                                lwh,
                                                key.second->event->item10,
                                                key.second->event->item11,
                                                key.second->event->item12,
                                                key.second->event->item13);
                    json_result1.push_back(temp_tuple1);
                    // }
                    // if (json_result1.size() != 0)
                    // {
                        // savejson3[key.second->event->item1] = json_result1;
                    savejson3[std::to_string(key.second->event->item0)] = json_result1;
                    std::ofstream output3(name3, std::ios::app);
                    output3 << savejson3 << std::endl;
                    // }
                }
            }
        }*/

    }
    return res;
}



Traffic_Flow::~Traffic_Flow() {

}

Event_Output::Event_Output() {}

void Detect_Info::fill_data(const xt::xarray<double> &origin_data, xt::xarray<int> &px_data,
                            const xt::xarray<double> &boxdata_info) {
    x = origin_data(0);
    y = origin_data(1);
    z = origin_data(2);
    w = origin_data(3);
    l = origin_data(4);
    h = origin_data(5);
    angle = origin_data(6);
    class_id = origin_data(7);
    v = origin_data(8);
    if (origin_data(8) >= 0){v_director = 1;}
    // all_area_id = origin_data[9];
    coincidence = origin_data(10);
    src = origin_data(11);
    origin_id = origin_data[12];
    pc_id = 1;   // 1
    longitude = origin_data(15);
    latitude = origin_data(16);
    pc_x = origin_data(17);
    pc_y = origin_data(18);
    cam_id = origin_data(19);

    center_px = px_data(0);
    center_py = px_data(1);
    front_right_px = px_data(2);
    front_right_py = px_data(3);
    front_left_px = px_data(4);
    front_left_py = px_data(5);
    behind_left_px = px_data(6);
    behind_left_py = px_data(7);
    behind_right_px = px_data(8);
    behind_right_py = px_data(9);

    cur_id_time = boxdata_info(0);
    cur_global_frameid = boxdata_info(1);
    left_top_px = boxdata_info(2);
    left_top_py = boxdata_info(3);
    right_bottom_px = boxdata_info(4);
    right_bottom_py = boxdata_info(5);
    head_camera_time = boxdata_info(6);
    body_camera_time = boxdata_info(7);
    tail_camera_time = boxdata_info(8);
    lane_no = boxdata_info(9);
    pre_frame = boxdata_info(10);

    cam_id_time_dict[1] = head_camera_time;
    cam_id_time_dict[2] = body_camera_time;
    cam_id_time_dict[2] = tail_camera_time;
    // single_info = Single_Info;
}

void Lane_Info::fill_data(const std::vector<double> &origin_data,
                          const xt::xarray<int> &Congestion_Level_highway_table,
                          const xt::xarray<int> &Congestion_Level_main_branch_table) {

    lane_direction = origin_data[1];
    lane_class = origin_data[2];  // 0
    is_non_motorway = origin_data[3];
    is_bus_lane = origin_data[4];
    is_emergency_lane = origin_data[5];
    min_v = origin_data[6] / 3.6;    //is to change the unit to m/s?  origin_data is m/s?
    max_v = origin_data[7] / 3.6;   //yk:pay attention to unit of velocity of car and lane demand.
    length = origin_data[8];
    width = origin_data[9];
    start_px = origin_data[10];
    start_py = origin_data[11];
    end_px = origin_data[12];
    end_py = origin_data[13];
    // radar_direct = origin_data[17];
    get_v_for_congestion_level(Congestion_Level_highway_table, Congestion_Level_main_branch_table);
}
//获取用于判断道路拥堵情况的速度边界
void Lane_Info::get_v_for_congestion_level(const xt::xarray<int> &Congestion_Level_highway_table,
                                           const xt::xarray<int> &Congestion_Level_main_branch_table) {
    xt::xarray<int> this_table =
            lane_class == Lane_Class::highway ? Congestion_Level_highway_table : Congestion_Level_main_branch_table;

    if (max_v * 3.6 < this_table(this_table.shape(0) - 1, 0)) {
        unblocked_min_v = this_table(this_table.shape(0) - 1, 1);
        slightly_congested_min_v = this_table(this_table.shape(0) - 1, 2);
        slightly_congested_max_v = this_table(this_table.shape(0) - 1, 3);
        // moderately_conupdate_versionested_min_v = this_table(this_table.shape(0) - 1, 6); // cjm bianyibutongguo
        seriously_congested_max_v = this_table(this_table.shape(0) - 1, 7);
    } else if (max_v * 3.6 >= this_table(0, 0)) {
        unblocked_min_v = this_table(0, 1);
        slightly_congested_min_v = this_table(0, 2);
        slightly_congested_max_v = this_table(0, 3);
        moderately_congested_min_v = this_table(0, 4);
        moderately_congested_max_v = this_table(0, 5);
        seriously_congested_min_v = this_table(0, 6);
        seriously_congested_max_v = this_table(0, 7);
    } else {
        for (int i = 0; i < this_table.shape(0) - 2; ++i) {
            if (max_v * 3.6 >= this_table(i + 1, 0) and max_v * 3.6 < this_table(i, 0)) {
                xt::xarray<double> temp = xt::view(this_table, i, xt::range(1, this_table.shape(1)));
                temp -= xt::view(this_table, i + 1, xt::range(1, this_table.shape(1)));
                temp = temp * (max_v * 3.6 - this_table(0, i + 1)) / (this_table(0, i) - this_table(0, i + 1));
                temp += xt::view(this_table, i + 1, xt::range(1, this_table.shape(1)));
                unblocked_min_v = temp(0);
                slightly_congested_min_v = temp(1);
                slightly_congested_max_v = temp(2);
                moderately_congested_min_v = temp(3);
                moderately_congested_max_v = temp(4);
                seriously_congested_min_v = temp(5);
                seriously_congested_max_v = temp(6);
                break;
            }
        }

    }
}

//填充非交汇处的额外信息 theta:y轴与正北方向夹角
void Lane_Info::fill_plus_data_for_non_converge(const std::vector<double> &origin_data,
                                                const xt::xarray<int> &negative_y_axis_xy, const double &theta) {
    std::string temp = std::to_string(int(origin_data[0])).substr(1, 1);
    out_in = std::atoi(temp.c_str());  // 是进口还是出口

    v_px = (end_px - start_px) / std::sqrt(std::pow((end_px - start_px), 2) + std::pow((end_py - start_py), 2));
    v_py = (end_py - start_py) / std::sqrt(std::pow((end_px - start_px), 2) + std::pow((end_py - start_py), 2));


    std::vector<int> this_lane_vec_xy = {end_px - start_px, end_py - start_py};
    float angle = std::acos(
            (negative_y_axis_xy(0) * this_lane_vec_xy[0] + negative_y_axis_xy(1) * this_lane_vec_xy[1]) /
            std::sqrt((std::pow(this_lane_vec_xy[0], 2) + std::pow(this_lane_vec_xy[1], 2)) * (
                    std::pow(negative_y_axis_xy[0], 2) + std::pow(negative_y_axis_xy[1], 2))));
    angle *= (180 / M_PI);
    angle_in = this_lane_vec_xy[0] < 0 ? angle : 360.0 - angle;
    angle_out = angle_in > 180.0 ? angle_in - 180 : angle_in + 180;
    int times = int(angle_in + 180.0 - theta) / 360;
    angle_to_north = angle_in + 180.0 - theta - times * 360.0;
}

//xcb add
//填充交汇处的额外信息
void Lane_Info::fill_plus_data_for_converge(std::shared_ptr<Lane_Info>start_lane_info,
                                     std::shared_ptr<Lane_Info>end_lane_info)
{
    out_in = 0;
    int reduce_angle = 15;
    float small_lane_angle, big_lane_angle;
    if(start_lane_info->angle_out < end_lane_info->angle_in)
    {
        small_lane_angle = start_lane_info->angle_out;
        big_lane_angle = end_lane_info->angle_in; 
    }
    else
    {
        small_lane_angle = end_lane_info->angle_in;
        big_lane_angle = start_lane_info->angle_out;
    }
    if((big_lane_angle - small_lane_angle) < 180)
    {
        small_angle = small_lane_angle + reduce_angle;
        big_angle = big_lane_angle - reduce_angle;
        // is_angle_in_this_turn_left_lane = is_angle_in_this_turn_left_lane_normal;
    }
    else
    {
        float angle_near_90 = small_lane_angle - reduce_angle;
        if(angle_near_90 < 0){
            angle_near_90 += 360;
        }
        float angle_near_270 = big_lane_angle + reduce_angle;
        if(angle_near_270 > 360){
            angle_near_270 -= 360;
        }
        if (abs(angle_near_90 - angle_near_270) < 180)
        {
            small_angle = angle_near_270;
            big_angle = angle_near_90;
            // is_angle_in_this_turn_left_lane = is_angle_in_this_turn_left_lane_normal;
        }
        else{
            small_angle = angle_near_90;
            big_angle = angle_near_270;
            // is_angle_in_this_turn_left_lane = is_angle_in_this_turn_left_lane_near_0;
        }
    }
    

}

//xcb add
bool Lane_Info::is_angle_in_this_turn_left_lane_normal(float angle)
{
    return small_angle <= angle && angle <= big_angle;
}
//xcb add
bool Lane_Info::is_angle_in_this_turn_left_lane_near_0(float angle)

{
    return (0 <= angle && angle <=small_angle) || (big_angle <= angle && angle <=360);
}

