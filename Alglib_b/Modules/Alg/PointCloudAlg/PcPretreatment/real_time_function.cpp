//
// Created by root on 5/7/21.
//

#include "real_time_function.h"

/* 保存BMP图，
* param：
*   pc_data : 点云数据
*   im_size : 保存的bmp图的大小，默认为3000
*   bmp_path ： 保存的路径
*/
void save_bmp(xt::xarray<float> pc_data, int im_size, std::string bmp_path){
    int img_corner = im_size / 2;
    int dist2pixel = 10;
    cv::Mat img = cv::Mat::zeros(im_size, im_size, CV_8UC3);
//    cv::Mat img = cv::Mat(cv::Size(im_size,im_size), CV_8UC3, cv::Scalar(0, 0, 0));
    int point_num = pc_data.shape(0);
    if (point_num == 0)
        printf("there is no data");
    for (int i = 0; i < point_num; ++i) {
        int tempx = int (std::round(pc_data(i, 0) * dist2pixel));
        int tempy = int (std::round(pc_data(i, 1) * dist2pixel));
        if (tempx >= img_corner || tempy >= img_corner || tempx <= -1 * img_corner || tempy <= -1 * img_corner)
            continue;

        tempx = img_corner - 1 + tempx;
        tempy = img_corner - 1 - tempy;

        img.at<cv::Vec3b>(tempy, tempx)[0] = 0;
        img.at<cv::Vec3b>(tempy, tempx)[1] = 255;
        img.at<cv::Vec3b>(tempy, tempx)[2] = 255;

    }
    cv::imwrite(bmp_path, img);
    // cv::namedWindow("Example1",cv::WINDOW_AUTOSIZE);
    // cv::imshow("Example1",img);
    // cv::waitKey(0);
    // cv::destroyWindow("Example1");

}


void pc_process::m_read_bmp(const std::string & bmp_path)
{
    cv::Mat img = cv::imread(bmp_path);
    if (img.empty())
        return;
    cv::Mat img_;
    cv::cvtColor(img, img_, cv::COLOR_BGR2RGB);
    background_img = img_ * 0 + 1;

    int height = background_img.size[0];
    for (int i = 0; i < height; ++i) {
        int i_ = height - i - 1;
        for (int j = 0; j < background_img.size[1]; ++j) {
            background_img.at<cv::Vec3b>(i, j)[0] = img_.at<cv::Vec3b>(i_, j)[0];
            background_img.at<cv::Vec3b>(i, j)[1] = img_.at<cv::Vec3b>(i_, j)[1];
            background_img.at<cv::Vec3b>(i, j)[2] = img_.at<cv::Vec3b>(i_, j)[2];
        }
    }
}

std::vector<int> pc_process::crop_bboxes2(xt::xarray<float> &bboxes){

   
    int im_size = background_img.size[0];
    int img_corner = im_size / 2;
    int dist2pixel = 5;
    int bboxes_num = bboxes.shape(0);

   std::vector<int> bb_index;

    int count_in = 0;

    for (int i = 0; i < bboxes_num; ++i) {
      
        int xindex = std::ceil(bboxes(i, 0) * dist2pixel) + img_corner - 1;
        int line_index = std::ceil(bboxes(i, 1) * dist2pixel) + img_corner - 1;
        if (xindex >= im_size || line_index >= im_size || xindex <= 0 || line_index <= 0){
           continue;
        }
        else if (background_img.ptr<cv::Vec3b>(line_index)[xindex][2] == 255){
          count_in++;
        }
        else{
            bb_index.push_back(i);

        }
    }
   // std::cout<<count_in<<"\n";
    return bb_index;
}

// pc_process::pc_process(const std::string & bmp_path):
//     grid_size_small(1),
//     grid_size_big(5),
//     thresh_small(0.2),
//     thresh_big(0.4),
//     h_thresh_x(-7),
//     h_thresh_y(-7),
//     dis_thresh(80)
// {
//     if (!bmp_path.empty())
//     {
//         m_read_bmp(bmp_path);
//     }
// }

pc_process::pc_process():
    grid_size_small(1),
    grid_size_big(5),
    thresh_small(0.2),
    thresh_big(0.4),
    h_thresh_x(-7),
    h_thresh_y(-7),
    dis_thresh(80)
{
}

/***************************************************************
 * @file       real_time_function.cpp
 * @brief      点云处理对象的构造函数
 * @input      AlgParams: 参数结构体指针
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
pc_process::pc_process(TSelfPcAlgParam * AlgParams):
    grid_size_small(AlgParams->m_stPcAlgParam.m_nGridSizeSmall),
    grid_size_big(AlgParams->m_stPcAlgParam.m_nGridSizeBig),
    thresh_small(AlgParams->m_stPcAlgParam.m_fThreshSmall),
    thresh_big(AlgParams->m_stPcAlgParam.m_fThreshBig),
    h_thresh_x(AlgParams->m_stPcAlgParam.m_fHThreshX),
    h_thresh_y(AlgParams->m_stPcAlgParam.m_fHThreshY),
    dis_thresh(AlgParams->m_stPcAlgParam.m_fDisThresh)
{
    YAML::Node pc_process_cfg = YAML::LoadFile(AlgParams->m_strRootPath + AlgParams->m_stPcAlgParam.m_strCfgPath);
    AlgParams->m_stPcAlgParam.m_strBmpPath = AlgParams->m_strRootPath + pc_process_cfg["BACK_IMG_PATH"].as<std::string>();
    std::cout << "--zqj-debug-init_bmp_path:" << AlgParams->m_stPcAlgParam.m_strBmpPath  << std::endl;
    if (!AlgParams->m_stPcAlgParam.m_strBmpPath.empty()){
        m_read_bmp(AlgParams->m_stPcAlgParam.m_strBmpPath);
    }
}

xt::xarray<float> pc_process::m_crop_pcdata(xt::xarray<float> &frame_data)
{
    int im_size = background_img.size[0];
    int img_corner = im_size / 2;
    int dist2pixel = 5;
    int point_num = frame_data.shape(0);

    xt::xarray<float> pc_inside = xt::zeros<float>({point_num, 4});

    int count_in = 0;

    for (int i = 0; i < point_num; ++i) {

        int xindex = std::ceil(frame_data(i, 0) * dist2pixel) + img_corner - 1;
        int line_index = std::ceil(frame_data(i, 1) * dist2pixel) + img_corner - 1;
        if (xindex >= im_size || line_index >= im_size || xindex <= 0 || line_index <= 0){

        }
        else if (background_img.ptr<cv::Vec3b>(line_index)[xindex][2] == 255){

        }
        else{
            pc_inside(count_in, 0) = frame_data(i, 0);
            pc_inside(count_in, 1) = frame_data(i, 1);
            pc_inside(count_in, 2) = frame_data(i, 2);
            pc_inside(count_in, 3) = frame_data(i, 3);
            count_in += 1;
        }
    }
    return xt::view(pc_inside, xt::range(0, count_in));

}

void pc_process::m_fused_back(xt::xarray<float> & backdata)
{
    double grid_small_inv = 1.0 / grid_size_small;
    int im_size_small = std::round(300 * grid_small_inv);

    double grid_big_inv = double (1.0 / double (grid_size_big));

    int im_size_big = std::round(300.0 * grid_big_inv);
    double min_x = -100.0;
    double min_y = -100.0;

    img_small = xt::zeros<float>({im_size_small, im_size_small, 4});
    img_big = xt::zeros<float>({im_size_big, im_size_big, 4});
    int point_num = backdata.shape(0);
    if (point_num == 0)
    {
        printf("there is no data\n");
    }

    for (int i = 0; i < point_num; ++i) {

        int tempx_small = floor((double (backdata(i, 0)) - min_x) * grid_small_inv);
        int tempy_small = floor((double (backdata(i, 1)) - min_y) * grid_small_inv);

        int tempx_big = floor((double (backdata(i, 0)) - min_x) * grid_big_inv);
        int tempy_big = floor((double (backdata(i, 1)) - min_y) * grid_big_inv);
        float tempz = backdata(i,2);


        if ((tempx_small > im_size_small-1) or (tempx_small<0) or (tempy_small > im_size_small-1) or (tempy_small<0)){
            continue;
        }

        else if(tempz < img_small(tempx_small, tempy_small, 3)){
            img_small(tempx_small, tempy_small, 3) = tempz;
        }

        else if (tempz < img_big(tempx_big, tempy_big, 3)){
            img_big(tempx_big, tempy_big, 3) = tempz;
        }
    }
}

xt::xarray<float> pc_process::m_fused_crop_back(xt::xarray<float> &PC_data){
    double grid_small_inv = 1.0 / grid_size_small;
    int im_size_small = std::round(300 * grid_small_inv);

    double grid_big_inv = 1.0 / grid_size_big;

    double min_x = -100.0, min_y = -100.0;
    int point_num = PC_data.shape(0);
    xt::xarray<float> in_points = xt::zeros<float>({point_num, 4});
    xt::xarray<float> out_points = xt::zeros<float>({point_num, 4});

    int count_in = 0, count_out = 0;
    if (point_num == 0)
        printf("there is no data\n");

    for (int i = 0; i < point_num; ++i) {
        int tempx_small = floor((double (PC_data(i, 0)) - min_x) * grid_small_inv);
        int tempy_small = floor((double (PC_data(i, 1)) - min_y) * grid_small_inv);

        int tempx_big = floor((double (PC_data(i, 0)) - min_x) * grid_big_inv);
        int tempy_big = floor((double (PC_data(i, 1)) - min_y) * grid_big_inv);

        if ((tempx_small > im_size_small -1) or (tempx_small < 0) or (tempy_small > im_size_small - 1) or (tempy_small < 0)){
            continue;
        }
        else if ((PC_data(i, 2) - img_small(tempx_small, tempy_small, 3)) > thresh_small){
            in_points(count_in, 0) = PC_data(i, 0);
            in_points(count_in, 1) = PC_data(i, 1);
            in_points(count_in, 2) = PC_data(i, 2);
            in_points(count_in, 3) = PC_data(i, 3);
            count_in += 1;
        }
        else if (std::abs(PC_data(i,0)) < dis_thresh and std::abs(PC_data(i,1)) < dis_thresh){
            if ((PC_data(i,2) - img_big(tempx_big, tempy_big, 3)) > thresh_big){
                in_points(count_in, 0) = PC_data(i, 0);
                in_points(count_in, 1) = PC_data(i, 1);
                in_points(count_in, 2) = PC_data(i, 2);
                in_points(count_in, 3) = PC_data(i, 3);
                count_in += 1;
            }
            else{
                out_points(count_out, 0) = PC_data(i, 0);
                out_points(count_out, 1) = PC_data(i, 1);
                out_points(count_out, 2) = PC_data(i, 2);
                out_points(count_out, 3) = PC_data(i, 3);
                count_out += 1;
            }

        }
        else if (std::abs(PC_data(i, 0)) > dis_thresh){
            if (PC_data(i, 2) > h_thresh_x){
                in_points(count_in, 0) = PC_data(i, 0);
                in_points(count_in, 1) = PC_data(i, 1);
                in_points(count_in, 2) = PC_data(i, 2);
                in_points(count_in, 3) = PC_data(i, 3);
                count_in += 1;
            }
            else {
                out_points(count_out, 0) = PC_data(i, 0);
                out_points(count_out, 1) = PC_data(i, 1);
                out_points(count_out, 2) = PC_data(i, 2);
                out_points(count_out, 3) = PC_data(i, 3);
                count_out += 1;
            }
        }
        else if (std::abs(PC_data(i, 1)) > dis_thresh){
            if (PC_data(i, 2) > h_thresh_y){
                in_points(count_in, 0) = PC_data(i, 0);
                in_points(count_in, 1) = PC_data(i, 1);
                in_points(count_in, 2) = PC_data(i, 2);
                in_points(count_in, 3) = PC_data(i, 3);
                count_in += 1;
            }
            else {
                out_points(count_out, 0) = PC_data(i, 0);
                out_points(count_out, 1) = PC_data(i, 1);
                out_points(count_out, 2) = PC_data(i, 2);
                out_points(count_out, 3) = PC_data(i, 3);
                count_out += 1;
            }
        }
        else{
            out_points(count_in, 0) = PC_data(i, 0);
            out_points(count_in, 1) = PC_data(i, 1);
            out_points(count_in, 2) = PC_data(i, 2);
            out_points(count_in, 3) = PC_data(i, 3);
            count_out += 1;
        }

    }
    in_points = xt::view(in_points, xt::range(0, count_in));
    out_points = xt::view(out_points, xt::range(0, count_out));
    return in_points;
}

bool pc_process::m_save_bmp(xt::xarray<float> & pc_data, const std::string & bmp_path, const int & im_size){
    int img_corner = im_size / 2;
    // int dist2pixel = im_size / 300;
    int dist2pixel = 10;
    cv::Mat img = cv::Mat::zeros(im_size, im_size, CV_8UC3);
    int point_num = pc_data.shape(0);
    if (point_num == 0)
        printf("there is no data");
    for (int i = 0; i < point_num; ++i) {
        int tempx = int (std::round(pc_data(i, 0) * dist2pixel));
        int tempy = int (std::round(pc_data(i, 1) * dist2pixel));
        if (tempx >= img_corner || tempy >= img_corner || tempx <= -1 * img_corner || tempy <= -1 * img_corner)
            continue;

        tempx = img_corner - 1 + tempx;
        tempy = img_corner - 1 - tempy;

        img.at<cv::Vec3b>(tempy, tempx)[0] = 0;
        img.at<cv::Vec3b>(tempy, tempx)[1] = 255;
        img.at<cv::Vec3b>(tempy, tempx)[2] = 255;

    }
    cv::imwrite(bmp_path, img);
    return true;
}


void pc_process::set_background(const std::string &bmp_path)
{
    m_read_bmp(bmp_path);
}

/***************************************************************
 * @file       real_time_function.cpp
 * @brief      点云预处理(裁剪与地面店过滤)
 * @input      points:          输入点云, 维度为2; 
 *             crop:            裁剪参数，true为裁剪，false为不裁剪; 
 *             filter_ground:   地面点过滤，true为过滤，false为不过滤;
 *             单位:             米.
 * @return     背景图为空时输出位false，其他情况下输出为true
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
bool pc_process::filter_points(xt::xarray<float> &points, const bool &crop, const bool & filter_ground)
{
    if (background_img.empty())
    {
        return false;
    }
    if (crop and filter_ground) 
    {
        xt::xarray<float> res = m_crop_pcdata(points);
        m_fused_back(res);
        points = m_fused_crop_back(res);
        return true;
    }
    else if (crop and !filter_ground) 
    {
        points = m_crop_pcdata(points);
        // std::cout << points << "\n";
        return true;
    }
    else if (!crop and filter_ground) 
    {
        return true;
    }
    else 
    {
        return true;
    }
    return true;
}

void pc_process::set_rotate_params(const std::vector<float> &angles, const std::vector<float> &translations)
{
    rotate_angle = angles;
    rotate_trans = translations;
}

/***************************************************************
 * @file       real_time_function.cpp
 * @brief      角度修正 --隧道添加，其他场景可能没用，需要甄别
 * @input      boxes_lidar: 输入点云检测结果, 尺寸N * 9 (x, y, z, w, l, h, yaw, cls, conf) 
 *             单位:         米、弧度
 * @author     陈向阳
 * @date       2022.10.12
 **************************************************************/
void pc_process::correct_angle(xt::xarray<float> &boxes_lidar)
{
    std::vector<std::size_t> shape = {vecAngle.size() / 2, 2 };
    xt::xarray<float> l_xarrAngle = xt::adapt(vecAngle, shape);
    fix_angle(boxes_lidar, l_xarrAngle);

    if(RotationTranslation)
    {
        xt::xarray<float> rotate = cal_trans(0, 0, rotate_angle[1]);  // z需要是弧度
        xt::xarray<float> rotate_inv = xt::linalg::inv(rotate);
        xt::xarray<float> boxes_lidar_temp = xt::view(boxes_lidar, xt::all(), xt::range(0, 3));
        boxes_lidar_temp = xt::transpose(boxes_lidar_temp);

        // 添加该判断，解决该错：ldb must be >= MAX(N,1): ldb=0 N=0ldc must be >= MAX(N,1): ldc=0 N=0Parameter 11 to routine cblas_sgemm was incorrect
        if (boxes_lidar_temp.shape(1) != 0)
        {
            xt::xarray<float> A = xt::linalg::dot(rotate_inv, boxes_lidar_temp);
            xt::view(boxes_lidar, xt::all(), xt::range(0, 3)) = xt::transpose(A);
        }
        xt::view(boxes_lidar, xt::all(), 6) += rotate_angle[1];    // z需要是弧度
    }
}

void pc_process::fix_angle(xt::xarray<float> &boxes_lidar, xt::xarray<float> &npAngle) {
    if (boxes_lidar.shape().size() < 2 or npAngle.shape().size() < 2) {
        return;
    }
    for (int num = 0; num < boxes_lidar.shape(0); ++num) {
        for (int i = 0; i < npAngle.shape(0); ++i) {
            if (boxes_lidar(num, 0) < npAngle(i, 0)) {
                boxes_lidar(num, 6) = npAngle(i, 1) * M_PI / 180.0;
                break;
            }
        }
    }
}

xt::xarray<float> pc_process::cal_trans(const float & x, const float & y, const float & z) {
    // 输入需要是弧度
    xt::xarray<float> R_x = {{1.0, 0.0,         0.0},
                             {0.0, std::cos(x), -1 * std::sin(x)},
                             {0.0, std::sin(x), std::cos(x)}};
    xt::xarray<float> R_y = {{std::cos(y),      0.0, std::sin(y)},
                             {0.0,              1.0, 0.0},
                             {-1 * std::sin(y), 0.0, std::cos(y)}};
    xt::xarray<float> R_z = {{std::cos(z), -1 * std::sin(z), 0.0},
                             {std::sin(z), std::cos(z),      0.0},
                             {0.0,         0.0,              1.0}};
//    std::cout<<R_x<<"\n";
//    std::cout<<R_y<<"\n";
//    std::cout<<R_z<<"\n";
    xt::xarray<float> rotate = xt::linalg::dot(R_z, R_y);
    rotate = xt::linalg::dot(rotate, R_x);
//    std::cout<<rotate<<"\n";
    return rotate;
}









xt::xarray<float> corner_to_surfaces_3d2(xt::xarray<float> corners){
    xt::xarray<float> surfaces = xt::zeros<float>({6, 4, (int)corners.shape(0), 3});
    xt::xarray<float> a= {{1.0,2.0},{2.0,3.0}};
    xt::xarray<int> index = {{0,1,2,3},{7,6,5,4},{0,3,7,4},{1,5,6,2},{0,4,5,1},{3,2,6,7}};
    for(int i=0; i<6;i++)
    {
        for(int j=0; j<4; j++)
        {
            xt::view(surfaces,i,j,xt::all(),xt::all()) = xt::view(corners,xt::all(), index(i,j), xt::all());
        }
    }
    surfaces = xt::transpose(surfaces,{2,0,1,3});
    return surfaces;
}

void surface_equ_3d_jitv3(xt::xarray<float> surfaces, xt::xarray<float>& normal_vec, xt::xarray<float>& d){
    int num_polygon = surfaces.shape(0);
    int max_num_surfaces = surfaces.shape(1);
    xt::xarray<float> sv0 = xt::zeros<float>({3});
    xt::xarray<float> sv1 = xt::zeros<float>({3});
    for(int  i=0; i<num_polygon; i++){
        for(int j=0; j<max_num_surfaces; j++ ){
            sv0(0)  = surfaces(i, j, 0, 0) -surfaces(i, j, 1, 0);
            sv0(1)  = surfaces(i, j, 0, 1) -surfaces(i, j, 1, 1);
            sv0(2)  = surfaces(i, j, 0, 2) -surfaces(i, j, 1, 2);
            sv1(0)  = surfaces(i, j, 1, 0) -surfaces(i, j, 2, 0);
            sv1(1)  = surfaces(i, j, 1, 1) -surfaces(i, j, 2, 1);
            sv1(2)  = surfaces(i, j, 1, 2) -surfaces(i, j, 2, 2);
            normal_vec(i, j, 0) = sv0(1) * sv1(2) - sv0(2) * sv1(1);
            normal_vec(i, j, 1) = sv0(2) * sv1(0) - sv0(0) * sv1(2);
            normal_vec(i, j, 2) = sv0(0) * sv1(1) - sv0(1) * sv1(0);
            d(i, j) = -surfaces(i, j, 0, 0) * normal_vec(i, j, 0) - \
            surfaces(i, j, 0, 1) * normal_vec(i, j, 1) - \
            surfaces(i, j, 0, 2) * normal_vec(i, j, 2);
        }

    }

}

std::pair<xt::xarray<int>, xt::xarray<float>> _points_count_convex_polygon_3d_jit2(xt::xarray<float> points,xt::xarray<float> polygon_surfaces,
                                         xt::xarray<float> normal_vec, xt::xarray<float> d,
                                         xt::xarray<float> num_surfaces,xt::xarray<float> out_box,xt::xarray<float> MBR){
    int max_num_surfaces = polygon_surfaces.shape(1);
//    int max_num_points_of_surface = polygon_surfaces.shape(2);
    int num_points = points.shape(0);
    int num_polygons = polygon_surfaces.shape(0);

    std::pair<xt::xarray<int>, xt::xarray<float>> number_height;
    xt::xarray<float> h_area = xt::zeros<float>({num_polygons, 6});
    for(int i=0; i<num_polygons; i++){
        h_area(i, 0) = 0;
        h_area(i, 1) = -100;
        h_area(i, 2) = 0;
        h_area(i, 3) = -100;
        h_area(i, 4) = 0;
        h_area(i, 5) = -100;
    }
    xt::xarray<int> ret = xt::zeros<int>({num_polygons});
    // for (int i=0; i<num_polygons; i++){
    //     ret(i) = num_points;
    // }
    //bool flag =true;
    for(int i=0; i<num_points; i++){
        for(int j=0; j<num_polygons; j++){
            bool flag =true;
            //××××××××××××× to do
             auto out_box_j = xt::view(out_box, j, xt::all(),  xt::all());
            //  std::cout<<"out_box_j"<<out_box_j<<std::endl;
            //  std::cout<<"MBR(j)"<<xt::view(MBR, j, xt::all())<<std::endl;
            // xt::xarray<float> con_range_max = xt::amax(out_box_i, {0});//out_box(i,xt::all(),0);
            // xt::xarray<float> con_range_min = xt::amin(out_box_i, {0});
            // if (points(i, 0)<con_range_max(0) && points(i, 0)>con_range_min(0) && points(i, 1)<con_range_max(1) && points(i, 1)>con_range_min(1))
           
            if(((points(i,0)>MBR(j,0)) and (points(i,0) < MBR(j,2))) and ((points(i,1)>MBR(j,1)) and (points(i,1) < MBR(j,3))))
            {
            
            for(int k=0;k<max_num_surfaces; k++){
                if(k>num_surfaces(j))
                    break;
                float sign = points(i, 0) * normal_vec(j, k, 0) + points(i, 1) * normal_vec(j, k, 1) + points(i, 2) * normal_vec(j, k, 2) + d(j, k);
               // std::cout<<"sign:"<<sign<<std::endl;
                if (sign >= 0){
                    //ret(j) -= 1;
                    flag = false;
                    break;
                }
            }
            if(flag)
            {
                //flag=false;
               
                ret(j) += 1;
              //  std::cout<<":"<<points(i, 2)<<":"<<std::min(h_area(j, 0), points(i, 2))<<":"<<std::max(h_area(j, 1), points(i, 2))<<":"<<std::endl; 
                h_area(j, 0) = std::min(h_area(j, 0), points(i, 2));
                h_area(j, 1) = std::max(h_area(j, 1), points(i, 2));
                h_area(j, 2) = std::min(h_area(j, 2), points(i, 0));
                h_area(j, 3) = std::max(h_area(j, 3), points(i, 0));
                h_area(j, 4) = std::min(h_area(j, 4), points(i, 1));
                h_area(j, 5) = std::max(h_area(j, 5), points(i, 1));
            }
            else
            {
                flag=true;
            }

            
        }
        }
    }
   // std::cout<<"ret:"<<ret<<std::endl;
    number_height.first=ret;
    number_height.second=h_area;
    return number_height;
}

std::pair<xt::xarray<int>, xt::xarray<float>> points_count_convex_polygon_3d_jit2(xt::xarray<float> points, xt::xarray<float> polygon_surfaces,xt::xarray<float> out_box,xt::xarray<float> MBR)
{
//    int max_num_surfaces = polygon_surfaces.shape(1);
//    int max_num_points_of_surfaces = polygon_surfaces.shape(2);
//    int num_points = points.shape(0);

    int num_polygons = polygon_surfaces.shape(0);
    xt::xarray<int> num_surfaces = xt::zeros<int>({num_polygons});
    for(int i=0; i<num_polygons; i++)
    {
        num_surfaces(i) =  (int)99999;
    }
    xt::xarray<float> normal_vec = xt::zeros<float>({(int)polygon_surfaces.shape(0), (int)polygon_surfaces.shape(1), 3});
    xt::xarray<float> d = xt::zeros<float>({(int)polygon_surfaces.shape(0), (int)polygon_surfaces.shape(1)});
    surface_equ_3d_jitv3(xt::view(polygon_surfaces,xt::all(), xt::all(), xt::range(0,3), xt::all()), normal_vec, d);
    std::pair<xt::xarray<int>, xt::xarray<float>> num_point_per_boxes_height_area;
//    xt::xarray<int> num_point_per_boxes = _points_count_convex_polygon_3d_jit(points, polygon_surfaces, normal_vec, d, num_surfaces);
    
     clock_t start_3d_jit = clock(); 
    num_point_per_boxes_height_area = _points_count_convex_polygon_3d_jit2(points, polygon_surfaces, normal_vec, d, num_surfaces,out_box,MBR);
    double polygon_3d_jitTimes = ((double) clock() -start_3d_jit) / CLOCKS_PER_SEC;
    std::cout<<"polygon_3d_jit: "<<polygon_3d_jitTimes<<std::endl;
    return num_point_per_boxes_height_area;
}



std::pair<xt::xarray<int>, xt::xarray<float>> points_count_rbbox2(xt::xarray<float> &points, xt::xarray<float> box3d)
{
    clock_t start_corner_to_surfaces_3d = clock(); 
    xt::xarray<float> out_box = xt::zeros<float>({(int)box3d.shape(0),8, 3});
    for (int i=0; i<box3d.shape(0); i++)
    {
        xt::xarray<float> center = xt::view(box3d,i, xt::range(0,3));
        xt::xarray<float> dims = xt::view(box3d,i, xt::range(3,6));
        dims = dims.reshape({1,3});
        float angles = box3d(i,6);
        xt::view(out_box,i,xt::all(),xt::all()) =xt::view(box_ops2::center_to_corner_box3d(center,dims,angles),0, xt::all(), xt::all());
    }

    xt::xarray<float> surfaces = corner_to_surfaces_3d2(out_box);

    double corner_to_surfaces_3dTimes = ((double) clock() -start_corner_to_surfaces_3d) / CLOCKS_PER_SEC;
    // std::cout<<"corner_to_surfaces_3d: "<<corner_to_surfaces_3dTimes<<std::endl;

    clock_t start_mbr = clock(); 
    // std::cout <<"out_box"<< xt::view(out_box,0,xt::all(),xt::all())<<std::endl;
    xt::xarray<float> min_1 = xt::view(xt::amin(xt::view(out_box,xt::all(),xt::all(),0),-1),xt::all(), xt::newaxis());
    //std::cout <<"min_1"<< min_1<<std::endl;
    xt::xarray<float> min_2 = xt::view(xt::amin(xt::view(out_box,xt::all(),xt::all(),1),-1),xt::all(), xt::newaxis());

    xt::xarray<float> max_1 = xt::view(xt::amax(xt::view(out_box,xt::all(),xt::all(),0),-1),xt::all(), xt::newaxis());
    xt::xarray<float> max_2 = xt::view(xt::amax(xt::view(out_box,xt::all(),xt::all(),1),-1),xt::all(), xt::newaxis());

    xt::xarray<float> MBR_m = xt::concatenate(xt::xtuple(min_1,min_2),1);

    xt::xarray<float> MBR_m2 = xt::concatenate(xt::xtuple(MBR_m,max_1),1);

    xt::xarray<float> MBR = xt::concatenate(xt::xtuple(MBR_m2,max_2),1);

    //std::cout << MBR<<std::endl;
    double run_MBRTimes = ((double) clock() -start_mbr) / CLOCKS_PER_SEC;
   // std::cout<<"MBR: "<<run_MBRTimes<<std::endl;
    //std::cout <<"MBR"<< MBR<<std::endl;

    return points_count_convex_polygon_3d_jit2(xt::view(points, xt::all(), xt::range(0,3)), surfaces, out_box,MBR);

}
