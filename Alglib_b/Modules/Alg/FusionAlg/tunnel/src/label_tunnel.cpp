//
// Created by root on 1/6/22.
//

#include "label_tunnel.h"


//point-cloud 0:car 1:bicycle 2:bus 3:tricycle 4:people 5:semi 6:truck
//video 0:person 1:bicycle 2: eleme 3: meituan 4: other food bicycle 5:tricycle 6:express_deliver_car 7:car 8:bus 9:little_truck 10:large_truck
//result 0:car 1:bicycle 2: other food bicycle 3:tricycle 4:people 5:bus 6:truck 7:eleme 8:meituan 9:express_deliver_car

int label_tunnel::lidar_class_transform(int lidar_id) {
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

int label_tunnel::video_class_transform(int video_id) {
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
std::string label_tunnel::class2size(int class_id) {
    if (class_id == 0)
        return "car";
    else if (class_id == 1 || class_id == 2 || class_id == 7 || class_id == 8)
        return "bicycle";
    else if (class_id == 3 || class_id == 9)
        return "motorcycle";
    else if (class_id == 4)
        return "person";
    else if (class_id == 5)
        return "bus";
    else if (class_id == 6)
        return "truck";
    else
        return "car";
}

void label_tunnel::lidar_class_judge( xt::xarray<float> &data_temp,float &lidar_class)
{

    float lidar_area = data_temp(3) * data_temp(4);
    float lidar_class_tmp = data_temp(7);

    if (data_temp(4) > 8)
        lidar_class = 5.0;
    else if (data_temp(4) <= 8 && data_temp(4) > 6)
    {
//        if (lidar_class_tmp != 5 && lidar_class_tmp != 6)
//            lidar_class = 6.0;
//        else
//            lidar_class = lidar_class_tmp;
        lidar_class = 6.0;
    }
    else if (data_temp(4) <= 6 && data_temp(4) > 4)
    {
        lidar_class = 0.0;
    }
//    else if (lidar_area <= 15 && lidar_area > 6)
//        lidar_class = 0.0;
    else if (data_temp(4)<=4 && data_temp(4) > 1.3)
    {
        if (lidar_class_tmp != 4 &&
            lidar_class_tmp != 2 && lidar_class_tmp != 7 &&
            lidar_class_tmp != 8 && lidar_class_tmp != 3 && lidar_class_tmp != 9)//not small target
            lidar_class = 1.0;
        else
            lidar_class = lidar_class_tmp;
    }
    else if (data_temp(4)<=1.3 && data_temp(4)>0)
    {
        if (lidar_class_tmp != 1 && lidar_class_tmp != 2 && lidar_class_tmp != 7 && lidar_class_tmp != 8)
            lidar_class = 4.0;
        else

            lidar_class = lidar_class_tmp;
    }
    else
    {
        std::cout<<"possible"<<std::endl;
        lidar_class = lidar_class_tmp;
    }
}

int label_tunnel::fusion_class(int lidar_id, int video_id) {
    int final_class;
    if (video_id == lidar_id)
        final_class = video_id;
    else if (video_id == 0) {
//        if (lidar_id == 1 || lidar_id == 2 || lidar_id == 3)
//            final_class = lidar_id;
//        else
        final_class = video_id;
    }
    else if (video_id == 1 || video_id == 2 || video_id == 7 || video_id == 8)
    {
        if (lidar_id == 4)
            final_class = lidar_id;
        else
            final_class = video_id;
    }

    else if (video_id == 3 || video_id == 9) {
        final_class = video_id;
    }
    else if (video_id == 4) {
//        if (lidar_id == 1 || lidar_id == 2)
//            final_class = video_id;
//        else
        final_class = video_id;
    }
    else
        final_class = video_id;

    return final_class;

}
