#include "Camera_reflect.h"

Camera_reflect::Camera_reflect()
{
}

Camera_reflect::~Camera_reflect()
{
}

Camera_reflect::Camera_reflect(const Camera_reflect &ob)
{
    m_in_param_matrix = ob.m_in_param_matrix;  // 相机内参  load_camera_param
    m_rotate_matrix = ob.m_rotate_matrix;    // 相机旋转参数  load_camera_param
    m_translation_matrix = ob.m_translation_matrix;   // 相机平移参数  load_camera_param
    m_dist_matrix = ob.m_dist_matrix;     // 相机畸变系数  load_camera_param

    m_lidar_rotate_matrix = ob.m_lidar_rotate_matrix;  // 辅雷达向主雷达的旋转矩阵 load_csv
    m_lidar_trans_vec = ob.m_lidar_trans_vec;   // 辅雷达向主雷达的平移矩阵  load_csv

    m_filter_image_flag = ob.m_filter_image_flag;  // 过滤图像标志位  load_filter_image

    m_reflect_points = ob.m_reflect_points;  // 反映射点 

    m_pixel_xyz = ob.m_pixel_xyz; // 加载的反映射存放在这里  load_npy

}

void Camera_reflect::load_camera_param(int camera_index, TCameraParam *_camera_param)
{
    m_in_param_matrix = {{_camera_param->m_vecCameraDev[camera_index-1].m_vecInParameter[0], 
                          _camera_param->m_vecCameraDev[camera_index-1].m_vecInParameter[1],
                          _camera_param->m_vecCameraDev[camera_index-1].m_vecInParameter[2]},{
                          _camera_param->m_vecCameraDev[camera_index-1].m_vecInParameter[3],
                          _camera_param->m_vecCameraDev[camera_index-1].m_vecInParameter[4],
                          _camera_param->m_vecCameraDev[camera_index-1].m_vecInParameter[5]},{
                          _camera_param->m_vecCameraDev[camera_index-1].m_vecInParameter[6],
                          _camera_param->m_vecCameraDev[camera_index-1].m_vecInParameter[7],
                          _camera_param->m_vecCameraDev[camera_index-1].m_vecInParameter[8]}};

    m_rotate_matrix = {{_camera_param->m_vecCameraDev[camera_index-1].m_vecRotateMatrix[0], 
                        _camera_param->m_vecCameraDev[camera_index-1].m_vecRotateMatrix[1],
                        _camera_param->m_vecCameraDev[camera_index-1].m_vecRotateMatrix[2]}};

    m_translation_matrix = {{_camera_param->m_vecCameraDev[camera_index-1].m_vecTranslationMatrix[0],
                             _camera_param->m_vecCameraDev[camera_index-1].m_vecTranslationMatrix[1],
                             _camera_param->m_vecCameraDev[camera_index-1].m_vecTranslationMatrix[2]}};

    m_dist_matrix ={{_camera_param->m_vecCameraDev[camera_index-1].m_vecDistMatrix[0],
                     _camera_param->m_vecCameraDev[camera_index-1].m_vecDistMatrix[1],
                     _camera_param->m_vecCameraDev[camera_index-1].m_vecDistMatrix[2],
                     _camera_param->m_vecCameraDev[camera_index-1].m_vecDistMatrix[3],
                     _camera_param->m_vecCameraDev[camera_index-1].m_vecDistMatrix[4]}};
    
}

void Camera_reflect::load_filter_image(std::string _file_path)
{
    cv::Mat imag = cv::imread(_file_path, cv::IMREAD_COLOR);
	if (imag.empty()){
		std::cout << "can't open the image:"<< _file_path << std::endl;
		// return -1;
	}
	cv::Size size = imag.size();
	int width = size.width;
	int height = size.height;
	
	// auto &filter_image_flag = *_filter_image_flag;
	m_filter_image_flag = cv::Mat::ones(height, width, CV_32S);
	for(int i = 0; i<height; i++){
		for(int j = 0; j<width; j++){
			cv::Vec3b pixel = imag.at<cv::Vec3b>(i, j);
			if(pixel[0] == 255 && pixel[1] == 0 && pixel[2] == 0){
				m_filter_image_flag.at<int>(i, j) = 0;
			}
		}
	}
	// std::cout << filter_image_flag.at<int>(1079, 1919) << std::endl;
	
	// std::cout << filter_image_flag.rows << std::endl;
	// std::cout << filter_image_flag.cols << std::endl;
}

void Camera_reflect::get_xyz(xt::xarray<float> &_orgin_boxs)
{
    int box_num = _orgin_boxs.shape(0);
    xt::xarray<float> reflect_points_init = xt::zeros<float>({box_num, 3});
    auto all_object_h = xt::eval(xt::view(_orgin_boxs, xt::all(), 3) - xt::view(_orgin_boxs, xt::all(), 1));
    auto all_object_w = xt::eval(xt::view(_orgin_boxs, xt::all(), 2) - xt::view(_orgin_boxs, xt::all(), 0));
    auto all_center_x = xt::cast<int>((xt::view(_orgin_boxs, xt::all(), 0) + xt::view(_orgin_boxs, xt::all(), 2)) * 0.5);
    auto all_center_y = xt::cast<int>(xt::view(_orgin_boxs, xt::all(), 3));
    auto reflect_x_1 = all_center_x;
    auto reflect_y_1 = all_center_y;
    auto all_h_ratio = xt::cast<float>(all_center_y / 1080 * 0.25);
    auto all_w_ratio = xt::cast<float>(all_center_x / 1920 * 0.25);
    auto reflect_x_2 = xt::cast<int>(all_center_x + all_w_ratio * all_object_w);
    auto reflect_y_2 = xt::cast<int>(all_center_y - all_h_ratio * all_object_h);
    for (int i = 0; i < box_num; i++){
        int center_x = all_center_x(i);
        int center_y = all_center_y(i);

        if (m_filter_image_flag.at<int>(center_y, center_x) == 0){
            reflect_points_init(i, 0) = m_pixel_xyz(reflect_x_1(i), reflect_y_1(i), 0);
            reflect_points_init(i, 1) = m_pixel_xyz(reflect_x_1(i), reflect_y_1(i), 1);
            reflect_points_init(i, 2) = m_pixel_xyz(reflect_x_1(i), reflect_y_1(i), 2);
        }else{
            reflect_points_init(i, 0) = m_pixel_xyz(reflect_x_2(i), reflect_y_2(i), 0);
            reflect_points_init(i, 1) = m_pixel_xyz(reflect_x_2(i), reflect_y_2(i), 1);
            reflect_points_init(i, 2) = m_pixel_xyz(reflect_x_2(i), reflect_y_2(i), 2);
        }
    }
    reflect_points_init.reshape({-1, 3});
    xt::xarray<float> reflect_points = xt::transpose(xt::linalg::dot(m_lidar_rotate_matrix, xt::transpose(reflect_points_init)));
    xt::view(reflect_points, xt::all(), 0) += m_lidar_trans_vec(0, 0);
    xt::view(reflect_points, xt::all(), 1) += m_lidar_trans_vec(0, 1);
    xt::view(reflect_points, xt::all(), 2) += m_lidar_trans_vec(0, 2);

    m_reflect_points = reflect_points;
}
void Camera_reflect::load_npy(std::string _file_path)
{
    m_pixel_xyz = xt::load_npy<double>(_file_path);
}

void Camera_reflect::cal_rorate_matrix(float angle_x, float angle_y, float angle_z)
{
    xt::xarray<float> R_x = {{1, 0, 0,}, {0, cos(angle_x), -1 * sin(angle_x)}, {0, sin(angle_x), cos(angle_x)}};
    xt::xarray<float> R_y = {{cos(angle_y), 0, sin(angle_y)}, {0, 1, 0}, {-1 * sin(angle_y), 0, cos(angle_y)}};
    xt::xarray<float> R_z = {{cos(angle_z), -1 *sin(angle_z), 0}, {sin(angle_z), cos(angle_z), 0}, {0, 0, 1}};

    xt::xarray<float> rorate = xt::linalg::dot(R_z, R_y);
    m_lidar_rotate_matrix = xt::linalg::dot(rorate, R_x);
}

void Camera_reflect::set_lidar_rotate_matrix(xt::xarray<float> _rotate)
{
    cal_rorate_matrix(_rotate(0, 0), _rotate(0, 1), _rotate(0, 2));
}

void Camera_reflect::set_lidar_trans_vec(xt::xarray<float> _trans)
{
    m_lidar_trans_vec = _trans;
}

xt::xarray<float> Camera_reflect::get_reflect_points()
{
    return m_reflect_points;
}
