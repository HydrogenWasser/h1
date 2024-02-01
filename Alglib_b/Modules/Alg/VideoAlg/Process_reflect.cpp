#include "Process_reflect.h"

Process_reflect::Process_reflect(nlohmann::json *_parameter)
{
	m_parameter = _parameter;
}

Process_reflect::~Process_reflect()
{
}

void Process_reflect::load_csv(std::string file_path, xt::xarray<float> &matrix)
/*
	加载csv,把csv存储的数据转换成xt::xarray
*/
{
	std::ifstream file(file_path, std::ios::in);
	if (!file.good())
	{
		perror("can't open file");
	}
	std::string line;
	int row = 0;
	int column = 0;
	std::vector<float> data;
	while (std::getline(file, line))
	{
		/* code */
		column = 0;
		row++;

		// if(row == 1){
		// 	continue;
		// }

		std::stringstream ss(line);
		float value;

		// std::vector<float> row_data;

		while (ss >> value)
		{
			/* code */
			// std::cout << column << std::endl;
			data.push_back(value);
			if (ss.peek() == ',')
			{
				ss.ignore();
			}
			column++;
		}

		// data.push_back(row_data);
	}
	file.close();

	// for(int i = 0; i<data.size(); i++){
	// 	for (int j = 0; j<data[i].size(); j++){
	// 		std::cout << data[i][j] << " ";
	// 	}
	// 	std::cout << "\n";
	// }

	matrix = xt::adapt(data, {row, column});
}

void Process_reflect::load_rorate(std::string _file_path, xt::xarray<float> &_matrix)
{
	load_csv(_file_path, _matrix);
}

void Process_reflect::load_trans(std::string _file_path, xt::xarray<float> &_matrix)
{
	load_csv(_file_path, _matrix);
	_matrix = _matrix * PI / 180;
}

void Process_reflect::load_reflect(TCameraParam *_camera_param, int _camera_dev_num, std::string _filter_image_dir, std::string _pixel_xyz_dir)
{
	for (int i = 0; i < _camera_dev_num; i++)
	{
		std::string filter_image_path = _filter_image_dir + "back_filter" + std::to_string(i + 1) + ".png";
		std::string pixel_xyz_path = _pixel_xyz_dir + "pixel2lidar_channel" + std::to_string(i + 1) + ".npy";
		std::shared_ptr<Camera_reflect> reflect_handler(new Camera_reflect);
		// reflect_handler.load_camera_param(i, _camera_param);
		reflect_handler->load_filter_image(filter_image_path);
		reflect_handler->load_npy(pixel_xyz_path);

		reflect_handler->set_lidar_rotate_matrix(xt::view(m_rorate, i, xt::all()));
		reflect_handler->set_lidar_trans_vec(xt::view(m_trans, i, xt::all()));

		m_coordinate_estimators.push_back(reflect_handler);
	}
}

void Process_reflect::init_reflect(std::string _rorate_file_path, std::string _trans_file_path, TCameraParam *_camera_param, int _camera_dev_num, std::string _filter_image_dir, std::string _pixel_xyz_dir)
{
	load_rorate(_rorate_file_path, m_rorate);

	load_trans(_trans_file_path, m_trans);

	load_reflect(_camera_param, _camera_dev_num, _filter_image_dir, _pixel_xyz_dir);
}

void Process_reflect::video_class_transform(xt::xarray<int> &labels, int index)
{
	// auto &labels = *_labels;
	int video_box_nums = labels.shape(0);
	for (int i = 0; i < video_box_nums; i++)
	{
		std::string video_class_init = std::to_string(int(labels(i, index)));
		labels(i, index) = m_video_class_to_lidar_class[video_class_init];
	}
}

void Process_reflect::load_fusion_param()
{
	for (auto item : (*m_parameter)["fusion_param"]["camera_raw_size"].items())
	{
		m_camera_raw_size.push_back(int(item.value()));
	}

	for (auto item : (*m_parameter)["fusion_param"]["video_class_to_lidar_class"].items())
	{
		m_video_class_to_lidar_class[item.key()] = item.value();
	}

	std::vector<float> wlh;
	for (auto item : (*m_parameter)["fusion_param"]["class_to_size_dict"].items())
	{

		for (auto v : item.value().items())
		{
			wlh.push_back(float(v.value()));
		}

		m_class_to_size_dict[item.key()] = wlh;
		wlh.clear();
	}
}

void Process_reflect::get_tracker_videoBoxInfo(std::vector<xt::xarray<float>> &origin_box, std::vector<xt::xarray<float>> &new_box)
{

	int nVideoCount = origin_box.size(); // n 路相机的检测数据
	for (int i = 0; i < nVideoCount; i++)
	{
		if (origin_box[i].shape(0) == 0)
		{
			new_box.push_back(xt::empty<float>({0, 17}));
			continue;
		}
		else
		{
			xt::xarray<float> boxes = xt::view(origin_box[i], xt::all(), xt::range(0, 4));
			xt::xarray<float> pre_boxes = boxes;

			xt::view(boxes, xt::all(), 0) *= (float(m_camera_raw_size[0]) / float(IMG_size));
			xt::view(boxes, xt::all(), 1) *= (float(m_camera_raw_size[1]) / float(IMG_size));
			xt::view(boxes, xt::all(), 2) *= (float(m_camera_raw_size[0]) / float(IMG_size));
			xt::view(boxes, xt::all(), 3) *= (float(m_camera_raw_size[1]) / float(IMG_size));
			xt::view(boxes, xt::all(), 0) = xt::clip(xt::view(boxes, xt::all(), 0), 0, m_camera_raw_size[0] - 1);
			xt::view(boxes, xt::all(), 1) = xt::clip(xt::view(boxes, xt::all(), 1), 0, m_camera_raw_size[1] - 1);
			xt::view(boxes, xt::all(), 2) = xt::clip(xt::view(boxes, xt::all(), 2), 0, m_camera_raw_size[0] - 1);
			xt::view(boxes, xt::all(), 3) = xt::clip(xt::view(boxes, xt::all(), 3), 0, m_camera_raw_size[1] - 1);
			xt::xarray<float> scores = xt::view(origin_box[i], xt::all(), 4);
			scores = scores.reshape({-1, 1});

			xt::xarray<int> labels = xt::cast<int>(xt::view(origin_box[i], xt::all(), 5));
			labels = labels.reshape({-1, 1});
			xt::xarray<float> ids = xt::zeros<float>({int(origin_box[i].shape(0))}); // xt::view(origin_box[i], xt::all(), 6);
			ids = ids.reshape({-1, 1});

			video_class_transform(labels, 0);
			int box_num = labels.shape(0); // 目标数量
			xt::xarray<float> wlh = xt::zeros<float>({box_num, 3});
			for (int j = 0; j < box_num; j++)
			{
				std::string video_label = std::to_string(int(labels(j, 0)));
				xt::view(wlh, j, xt::all()) = xt::adapt(m_class_to_size_dict[video_label]);
			}
			m_coordinate_estimators[i]->get_xyz(boxes);
			xt::xarray<float> theta = xt::full_like(labels, 100 * PI / 180);
			xt::xarray<float> data_source = xt::full_like(labels, 1);
			xt::xarray<float> channel = xt::full_like(labels, i + 1);
			xt::xarray<float> speed = xt::full_like(labels, 1);

			xt::xarray<float> video_for_fusion = xt::zeros<float>({box_num, 17});
			video_for_fusion = xt::hstack(std::make_tuple(m_coordinate_estimators[i]->get_reflect_points(), wlh, theta, xt::cast<float>(labels), speed, ids, scores, pre_boxes, data_source, channel));
			new_box.push_back(video_for_fusion);
		}
	}
}
