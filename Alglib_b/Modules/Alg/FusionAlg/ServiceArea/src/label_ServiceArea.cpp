/*******************************************************
 文件名：label_ServiceArea.cpp
 作者：HuaHuan
 描述：标签内容
 版本：v1.0
 日期：2024-01-11
 *******************************************************/
#include "label_ServiceArea.h"

/*******************************************************
融合类别:
 "car", "person","little_bus","middle_bus", "large_bus", "little_truck",
 "middle_truck","large_truck","motorbicycle","tricycle",  "food_bicycle",
 "eleme", "meituan", "express_deliver_car","dangerous_car", "remaining",
 "ambulance", "fire_car","kfc","MacDonald","shunfeng","shentong","yuantong",
 "yunda","other"
 *******************************************************/


int label_ServiceArea::lidar_class_transform(int lidar_id) 
{
    int output_class;
    if (lidar_id == 0)  // car
        output_class = 0;
    else if (lidar_id == 1)  // bicycle
        output_class = 1;
    else if (lidar_id == 2)  // bus
        output_class = 5;
    else if (lidar_id == 3)  // tricycle
        output_class = 3;
    else if (lidar_id == 4)  // person
        output_class = 4;
    else if (lidar_id == 5)  // semi
        output_class = 6;
    else  // truck
        output_class = 6;

    return output_class;
}


int label_ServiceArea::video_class_transform(int video_id) {
    int output_class;
    if (video_id == 0)  // person
        output_class = 4;
    else if (video_id == 1)  // bicycle
        output_class = 1;
    else if (video_id == 2)  // eleme
        output_class = 7;
    else if (video_id == 3)  // meituan
        output_class = 8;
    else if (video_id == 4)  // other food bicycle
        output_class = 2;
    else if (video_id == 5)  // tricycle
        output_class = 3;
    else if (video_id == 6)  // express_deliver_car
        output_class = 9;
    else if (video_id == 7)  // car
        output_class = 0;
    else if (video_id == 8)  // bus
        output_class = 5;
    else                     // truck
        output_class = 6;

    return output_class;
}

// 相机转换至雷达  不同类别的取值
std::string label_ServiceArea::class2size(int class_id) {
    if (class_id == 0 || class_id == 2)
        return "car";
    else if (class_id == 8 || class_id == 9 || class_id == 10 || class_id == 11 || class_id == 12 || class_id == 13)
        return "bicycle";
    else if (class_id == 1)
        return "person";
    else if (class_id == 3 || class_id == 4)
        return "bus";
    else if (class_id == 5 || class_id == 6 || class_id == 7)
        return "truck";
    else
        return "car";
}

bool label_ServiceArea::judge_large_target(int target_class)
{
    if (target_class == 0 || target_class == 2 || target_class == 3 || target_class == 4 || target_class == 5 || target_class == 6 || target_class == 7 || target_class == 14
    || target_class == 16 || target_class == 17)
        return true;
    else
        return false;
}

void label_ServiceArea::lidar_class_judge( xt::xarray<float> &data_temp,float &lidar_class)
{

    float lidar_area = data_temp(3) * data_temp(4);
    float lidar_class_tmp = data_temp(7);

    if (data_temp(4) > 8)
        lidar_class = 4.0; //large_bus
    else if (data_temp(4) <= 8 && data_temp(4) > 6)
    {
//        if (lidar_class_tmp != 5 && lidar_class_tmp != 6)
//            lidar_class = 6.0;
//        else
//            lidar_class = lidar_class_tmp;
        lidar_class = 5.0; //little_truck
    }
    else if (data_temp(4) <= 6 && data_temp(4) > 4)
    {
        lidar_class = 0.0; //car
    }
//    else if (lidar_area <= 15 && lidar_area > 6)
//        lidar_class = 0.0;
    else if (data_temp(4)<=4 && data_temp(4) > 1.3)
    {
        if (lidar_class_tmp != 9 &&
            lidar_class_tmp != 10 && lidar_class_tmp != 11 &&
            lidar_class_tmp != 12 && lidar_class_tmp != 13 && lidar_class_tmp != 1)//not small target
            lidar_class = 8.0;  //motorbicycle
        else
            lidar_class = lidar_class_tmp;
    }
    else if (data_temp(4)<=1.3 && data_temp(4)>0)
    {
        if (lidar_class_tmp != 1 && lidar_class_tmp != 2 && lidar_class_tmp != 7 && lidar_class_tmp != 8)
            lidar_class = 2.0; //person
        else

            lidar_class = lidar_class_tmp;
    }
    else
    {
        std::cout<<"possible"<<std::endl;
        lidar_class = lidar_class_tmp;
    }
}

int label_ServiceArea::fusion_class(int lidar_id, int video_id) {
    int final_class;
    if (video_id == lidar_id)
        final_class = video_id;
    else if (video_id == 0){
    //car

//        if (lidar_id == 1 || lidar_id == 2 || lidar_id == 3)
//            final_class = lidar_id;
//        else
        final_class = video_id;
    }
    else if (video_id == 8 || video_id == 9 || video_id == 10 || video_id == 11 || video_id == 12)
    {
        //motorbicycle
        if (lidar_id == 1)
            final_class = lidar_id;
        else
            final_class = video_id;
    }

    else if (video_id == 2 || video_id == 3 || video_id == 4 || video_id == 5 || video_id == 6 || video_id == 7) {
        //bus & truck
        final_class = video_id;
    }
    else if (video_id == 4) {
        //person
//        if (lidar_id == 1 || lidar_id == 2)
//            final_class = video_id;
//        else
        final_class = video_id;
    }
    else
        final_class = video_id;

    return final_class;

}