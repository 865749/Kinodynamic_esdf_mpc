#include "plan_env/sdf_map.hpp"

void SDFMap::initMap(std::shared_ptr<rclcpp::Node> nh)
{
	this->node_ = nh;
	map_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map_topic", 10);
	mp_.frame_id_ = node_->declare_parameter<std::string>("sdf_map.frame_id", std::string("odom"));

	md_.use_global_map = node_->declare_parameter<bool>("sdf_map.use_global_map", true);
	md_.use_global_map = node_->get_parameter("sdf_map.use_global_map").as_bool();
	md_.global_map_num = node_->declare_parameter<int>("sdf_map.global_map_num", 2);
	md_.global_map_num = node_->get_parameter("sdf_map.global_map_num").as_int();
	node_->declare_parameter<std::vector<std::string>>("sdf_map.global_map_path", {});
	std::vector<std::string> global_map_path;
	node_->get_parameter("sdf_map.global_map_path", global_map_path);
	if (md_.use_global_map)
	{
		if (global_map_path.size() < md_.global_map_num)
		{
			RCLCPP_ERROR(node_->get_logger(), "Invalid number of elements in 'global_map_path': expected %ld, got %ld",
						 md_.global_map_num, global_map_path.size());
			throw std::runtime_error("Parameter 'global_map_path' has invalid size.");
		}
		int max_map_buffer_size = std::numeric_limits<int>::min();
		for (int i = 0; i < md_.global_map_num; i++)
		{
			int map_buffer_size = 0;
			md_.Global_Maps.push_back(load_map(global_map_path[i], mp_.frame_id_, map_buffer_size, mp_));

			if (max_map_buffer_size < map_buffer_size)
				max_map_buffer_size = map_buffer_size;
		}
		md_.occupancy_buffer_neg = vector<char>(max_map_buffer_size, 0);
		md_.occupancy_buffer_inflate_ = vector<char>(max_map_buffer_size, 0);
		md_.distance_buffer_ = vector<double>(max_map_buffer_size, 10000);
		md_.distance_buffer_neg_ = vector<double>(max_map_buffer_size, 10000);
		md_.distance_buffer_all_ = vector<double>(max_map_buffer_size, 10000);
		md_.tmp_buffer1_ = vector<double>(max_map_buffer_size, 0);
	}
	else
	{
		mp_.map_size_ = {node_->declare_parameter<double>("sdf_map.map_size_x", 30.0),
						 node_->declare_parameter<double>("sdf_map.map_size_y", 30.0)};

		mp_.resolution_ = node_->declare_parameter<double>("sdf_map.resolusion_", 0.01);
		mp_.map_origin_ = {node_->declare_parameter<double>("sdf_map.origin_x", -6.35),
						   node_->declare_parameter<double>("sdf_map.origin_y", -7.6)};
		mp_.map_origin_ = Eigen::Vector2d(-mp_.map_size_(0) / 2.0, -mp_.map_size_(0) / 2.0);
		for (int i = 0; i < 2; ++i)
			mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
		int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);

		md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
		md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
		md_.distance_buffer_ = vector<double>(buffer_size, 10000);
		md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
		md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);
		md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
	}

	esdf_timer_ = node_->create_wall_timer(0.05s, std::bind(&SDFMap::updateESDFCallback, this));
	mp_.obstacles_inflation_ = node_->declare_parameter<double>("sdf_map.obstacles_inflation", 0.0009);
	mp_.local_map_margin_ = node_->declare_parameter<int>("sdf_map.local_map_margin", 10);
	mp_.local_update_range_(0) = node_->declare_parameter<double>("sdf_map.local_update_range_x", 3.0);
	mp_.local_update_range_(1) = node_->declare_parameter<double>("sdf_map.local_update_range_y", 3.0);
	mp_.resolution_inv_ = 1 / mp_.resolution_;
	// mp_.map_origin_ = Eigen::Vector2d(-x_size / 2.0, -y_size / 2.0);
	// mp_.map_size_ = Eigen::Vector2d(x_size, y_size);
	mp_.show_esdf_time_ = node_->declare_parameter<bool>("sdf_map.show_esdf_time", false);

	// md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
	mp_.map_min_boundary_ = mp_.map_origin_;
	mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
	md_.max_esdf_time_ = 0.0;

	vis_timer_ = node_->create_wall_timer(0.05s, std::bind(&SDFMap::visCallback, this));
	map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy", 10);
	esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", 10);

	std::string odom_topic = node_->get_parameter("manager.odometry").as_string();
	RCLCPP_INFO(node_->get_logger(), "Odometry topic: %s", odom_topic.c_str());
	odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10,
																	std::bind(&SDFMap::odomCallback, this, std::placeholders::_1));

	std::string laser_topic = node_->declare_parameter<std::string>("sdf_map.laser_topic", "scan");
	laser_sub_.subscribe(node_, laser_topic, rmw_qos_profile_sensor_data);

	tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
	// Create the timer interface before call to waitForTransform,
	// to avoid a tf2_ros::CreateTimerInterfaceException exception
	auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
		node_->get_node_base_interface(), node_->get_node_timers_interface());
	tf2_buffer_->setCreateTimerInterface(timer_interface);
	tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
	tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
		laser_sub_, *tf2_buffer_, "odom", 10, node_->get_node_logging_interface(),
		node_->get_node_clock_interface(), std::chrono::duration<int>(1));
	// Register a callback with tf2_ros::MessageFilter to be called when transforms are available
	tf2_filter_->registerCallback(&SDFMap::laserCallback, this);
}
Global_Map SDFMap::load_map(std::string &path, const std::string &frame,int& map_buffer_size, MappingParameters &mp)
{
	// 打开并读取YAML元数据
	Global_Map gm;
	gm.path = path;
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		RCLCPP_ERROR(rclcpp::get_logger("load_map"), "Failed to open YAML file");
		throw std::runtime_error("Failed to open YAML file.");
	}

	// 获取元数据中的map分辨率和原点信息
	float resolution = 0.0;
	float occupied_thresh = 0.0;
	cv::Point3f origin;
	fs["resolution"] >> resolution;
	fs["origin"] >> origin;
	fs["occupied_thresh"] >> occupied_thresh;

	if (resolution <= 0.0)
	{
		RCLCPP_ERROR(rclcpp::get_logger("load_map"), "Invalid map resolution: %f", resolution);
		throw std::invalid_argument("Invalid map resolution.");
	}
	gm.resolution_ = resolution;
	gm.map_origin_ = {origin.x, origin.y};

	size_t dot_pos = path.find_last_of('.');

	if (dot_pos != std::string::npos)
	{
		path.replace(dot_pos, path.length() - dot_pos, ".pgm");
	}
	else
	{
		std::cerr << "No file extension found in the input path!" << std::endl;
	}
	cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);

	if (image.empty())
	{
		RCLCPP_ERROR(rclcpp::get_logger("load_map"), "Failed to load PGM image");
		throw std::runtime_error("Failed to load PGM image.");
	}

	gm.image_size = {image.rows, image.cols};
	gm.map_size_ = {image.cols * gm.resolution_, image.rows * gm.resolution_};

	for (int i = 0; i < 2; ++i)
		gm.map_voxel_num_(i) = ceil(gm.map_size_(i) / gm.resolution_);

	int buffer_size = gm.map_voxel_num_(0) * gm.map_voxel_num_(1);

	gm.occupancy_buffer_inflate_Global_Map = vector<char>(buffer_size, 0);

	cv::normalize(image, image, 0, 1, cv::NORM_MINMAX);

	auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
	msg->header.stamp = rclcpp::Clock().now();
	msg->header.frame_id = frame;
	msg->info.resolution = gm.resolution_;
	msg->info.width = image.cols;
	msg->info.height = image.rows;
	msg->info.origin.position.x = gm.map_origin_[0];
	msg->info.origin.position.y = gm.map_origin_[1];
	msg->info.origin.position.z = 0.0;
	msg->data.resize(gm.map_voxel_num_(0) * gm.map_voxel_num_(1));
	for (int y = 0; y < image.rows; ++y)
	{
		for (int x = 0; x < image.cols; ++x)
		{
			int pixelValue = image.at<uchar>(y, x);
			if (pixelValue < occupied_thresh)
			{
				msg->data[image.cols * (image.rows - y - 1) + x] = 100;
			}
			else
				msg->data[image.cols * (image.rows - y - 1) + x] = 0;
		}
	}
	map_msgs.push_back(msg);
	cv::flip(image, image, 0);
	std::cout << "image.rows:" << image.rows << std::endl
			  << "image.cols" << image.cols << std::endl;

	for (int y = 0; y < image.rows; ++y)
	{
		for (int x = 0; x < image.cols; ++x)
		{
			int pixelValue = image.at<uchar>(y, x);
			if (pixelValue < occupied_thresh)
			{
				gm.occupancy_buffer_inflate_Global_Map[x * gm.map_voxel_num_(1) + y] = 1;
			}
		}
	}
	mp.resolution_ = gm.resolution_;
	mp.map_origin_ = gm.map_origin_;
	mp_.map_size_(0) = std::max(image.cols * gm.resolution_, mp_.map_size_(0));
	mp_.map_size_(1) = std::max(image.rows * gm.resolution_, mp_.map_size_(1));
	mp_.map_voxel_num_(0) = std::max(gm.map_voxel_num_(0), mp_.map_voxel_num_(0));
	mp_.map_voxel_num_(1) = std::max(gm.map_voxel_num_(1), mp_.map_voxel_num_(1));
	map_buffer_size = std::max(map_buffer_size,buffer_size);
	return gm;
}

void SDFMap::resetBuffer(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos)
{

	Eigen::Vector2i min_id, max_id;
	posToIndex(min_pos, min_id);
	posToIndex(max_pos, max_id);

	boundIndex(min_id);
	boundIndex(max_id);

	/* reset occ and dist buffer */
	for (int x = min_id(0); x <= max_id(0); ++x)
		for (int y = min_id(1); y <= max_id(1); ++y)
		{
			md_.occupancy_buffer_inflate_[toAddress(x, y)] = 0;
			md_.distance_buffer_[toAddress(x, y)] = 10000;
		}
}
void SDFMap::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
{
	md_.laser_pos_(0) = odom->pose.pose.position.x;
	md_.laser_pos_(1) = odom->pose.pose.position.y;
	md_.laser_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
									  odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
	md_.has_odom_ = true;
	// std::cout << "sdf_odom" << std::endl;
}

void SDFMap::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser_msg)
{
	if (!md_.has_odom_)
	{
		return;
	}
	auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

	projectoir_.projectLaser(*laser_msg, *cloud_msg);
	try
	{
		*cloud_msg = tf2_buffer_->transform(*cloud_msg, "odom", tf2::durationFromSec(0.1));
	}
	catch (const tf2::ExtrapolationException &ex)
	{
		RCLCPP_ERROR(node_->get_logger(), "Error while transforming %s", ex.what());
		return;
	}
	pcl::PointCloud<pcl::PointXY>
		latest_laser;

	pcl::fromROSMsg(*cloud_msg, latest_laser);
	md_.has_cloud_ = true;
	if (latest_laser.points.size() == 0)
		return;

	if (isnan(md_.laser_pos_(0)) || isnan(md_.laser_pos_(1)))
		return;

	this->resetBuffer(md_.laser_pos_ - mp_.local_update_range_,
					  md_.laser_pos_ + mp_.local_update_range_);

	pcl::PointXY pt;
	Eigen::Vector2d p2d, p2d_inf;

	int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);

	double max_x, max_y, min_x, min_y;

	min_x = mp_.map_max_boundary_(0);
	min_y = mp_.map_max_boundary_(1);

	max_x = mp_.map_min_boundary_(0);
	max_y = mp_.map_min_boundary_(1);
	for (size_t i = 0; i < latest_laser.points.size(); ++i)
	{
		pt = latest_laser.points[i];
		p2d(0) = pt.x, p2d(1) = pt.y;
		// p2d = R2d * p2d;

		Eigen::Vector2d devi = p2d - md_.laser_pos_;
		Eigen::Vector2i inf_pt;
		if (devi.norm() < 0.2)
			continue;
		if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1))
		{
			for (int x = -inf_step; x <= inf_step; ++x)
				for (int y = -inf_step; y <= inf_step; ++y)
				{

					p2d_inf(0) = pt.x + x * mp_.resolution_;
					p2d_inf(1) = pt.y + y * mp_.resolution_;

					max_x = max(max_x, p2d_inf(0));
					max_y = max(max_y, p2d_inf(1));

					min_x = min(min_x, p2d_inf(0));
					min_y = min(min_y, p2d_inf(1));

					posToIndex(p2d_inf, inf_pt);

					if (!isInMap(inf_pt))
						continue;

					int idx_inf = toAddress(inf_pt);

					md_.occupancy_buffer_inflate_[idx_inf] = 1;
				}
		}
	}
	min_x = min(min_x, md_.laser_pos_(0));
	min_y = min(min_y, md_.laser_pos_(1));

	max_x = max(max_x, md_.laser_pos_(0));
	max_y = max(max_y, md_.laser_pos_(1));

	posToIndex(mp_.local_update_range_ + md_.laser_pos_, md_.local_bound_max_);
	posToIndex(-mp_.local_update_range_ + md_.laser_pos_, md_.local_bound_min_);

	boundIndex(md_.local_bound_min_);
	boundIndex(md_.local_bound_max_);

	md_.esdf_need_update_ = true;
	md_.update_num_ += 1;
}
void SDFMap::updateESDFCallback()
{
	if (!md_.esdf_need_update_)
		return;

	/* esdf */
	auto t1 = rclcpp::Clock().now();

	updateESDF2d();

	auto t2 = rclcpp::Clock().now();

	md_.esdf_time_ += (t2 - t1).seconds();
	md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).seconds());

	if (mp_.show_esdf_time_)
		RCLCPP_INFO(node_->get_logger(), "ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).seconds(),
					md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

	md_.esdf_need_update_ = false;
}
template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
	int v[mp_.map_voxel_num_(dim)];
	double z[mp_.map_voxel_num_(dim) + 1];

	int k = start;
	v[start] = start;
	z[start] = -std::numeric_limits<double>::max();
	z[start + 1] = std::numeric_limits<double>::max();

	for (int q = start + 1; q <= end; q++)
	{
		k++;
		double s;

		do
		{
			k--;
			s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
		} while (s <= z[k]);

		k++;

		v[k] = q;
		z[k] = s;
		z[k + 1] = std::numeric_limits<double>::max();
	}

	k = start;

	for (int q = start; q <= end; q++)
	{
		while (z[k + 1] < q)
			k++;
		double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
		f_set_val(q, val);
	}
}
void SDFMap::updateESDF2d()
{
	Eigen::Vector2i min_esdf = md_.local_bound_min_;
	Eigen::Vector2i max_esdf = md_.local_bound_max_;

	/* ========== compute positive DT ========== */

	for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
	{
		fillESDF(
			[&](int y)
			{
				return md_.is_occupancy(toAddress(x, y)) == 1 ? 0 : std::numeric_limits<double>::max();
			},
			[&](int y, double val)
			{ md_.tmp_buffer1_[toAddress(x, y)] = val; }, min_esdf[1],
			max_esdf[1], 1);
	}
	for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
	{
		fillESDF(
			[&](int x)
			{
				return md_.tmp_buffer1_[toAddress(x, y)];
			},
			[&](int x, double val)
			{ md_.distance_buffer_[toAddress(x, y)] = sqrt(val) * mp_.resolution_; }, min_esdf[0],
			max_esdf[0], 0);
	}

	// /* ========== compute negative distance ========== */
	for (int x = min_esdf(0); x <= max_esdf(0); ++x)
		for (int y = min_esdf(1); y <= max_esdf(1); ++y)
		{
			int idx = toAddress(x, y);
			if (md_.is_occupancy(idx) == 0)
			{
				md_.occupancy_buffer_neg[idx] = 1;
			}
			else if (md_.is_occupancy(idx) == 1)
			{
				md_.occupancy_buffer_neg[idx] = 0;
			}
			else
			{
				RCLCPP_ERROR(node_->get_logger(), "what");
			}
		}

	for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
	{
		fillESDF(
			[&](int y)
			{
				return md_.occupancy_buffer_neg[toAddress(x, y)] == 1 ? 0 : std::numeric_limits<double>::max();
			},
			[&](int y, double val)
			{ md_.tmp_buffer1_[toAddress(x, y)] = val; }, min_esdf[1],
			max_esdf[1], 1);
	}
	for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
	{
		fillESDF(
			[&](int x)
			{
				return md_.tmp_buffer1_[toAddress(x, y)];
			},
			[&](int x, double val)
			{ md_.distance_buffer_neg_[toAddress(x, y)] = sqrt(val) * mp_.resolution_; }, min_esdf[0],
			max_esdf[0], 0);
	}

	/* ========== combine pos and neg DT ========== */
	for (int x = min_esdf(0); x <= max_esdf(0); ++x)
		for (int y = min_esdf(1); y <= max_esdf(1); ++y)
		{

			int idx = toAddress(x, y);
			md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

			if (md_.distance_buffer_neg_[idx] > 0.0)
				md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
		}
}
void SDFMap::publishMap()
{
	pcl::PointXYZ pt;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	Eigen::Vector2i min_cut = md_.local_bound_min_;
	Eigen::Vector2i max_cut = md_.local_bound_max_;

	int lmm = mp_.local_map_margin_ / 2;
	min_cut -= Eigen::Vector2i(lmm, lmm);
	max_cut += Eigen::Vector2i(lmm, lmm);

	boundIndex(min_cut);
	boundIndex(max_cut);

	for (int x = min_cut(0); x <= max_cut(0); ++x)
		for (int y = min_cut(1); y <= max_cut(1); ++y)
		{
			if (md_.is_occupancy(toAddress(x, y)) == 0)
				continue;
			
			Eigen::Vector2d pos;
			indexToPos(Eigen::Vector2i(x, y), pos);

			pt.x = pos(0);
			pt.y = pos(1);
			pt.z = 0;
			cloud.push_back(pt);
		}
	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = mp_.frame_id_;
	sensor_msgs::msg::PointCloud2 cloud_msg;

	pcl::toROSMsg(cloud, cloud_msg);
	cloud_msg.header.stamp = node_->now();

	map_pub_->publish(cloud_msg);
}
void SDFMap::getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2],
							Eigen::Vector2d &diff)
{
	if (!isInMap(pos))
	{
		// cout << "pos invalid for interpolation." << endl;
	}

	/* interpolation position */
	Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();
	Eigen::Vector2i idx;
	Eigen::Vector2d idx_pos;

	posToIndex(pos_m, idx);
	indexToPos(idx, idx_pos);
	diff = (pos - idx_pos) * mp_.resolution_inv_;

	for (int x = 0; x < 2; ++x)
	{
		for (int y = 0; y < 2; ++y)
		{
			Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
			Eigen::Vector2d current_pos;
			indexToPos(current_idx, current_pos);
			pts[x][y] = current_pos;
		}
	}
}
void SDFMap::publishESDF()
{
	double dist;
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointXYZI pt;

	const double min_dist = 0.0;
	const double max_dist = 3.0;

	Eigen::Vector2i min_cut = md_.local_bound_min_ -
							  Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
	Eigen::Vector2i max_cut = md_.local_bound_max_ +
							  Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
	boundIndex(min_cut);
	boundIndex(max_cut);

	for (int x = min_cut(0); x <= max_cut(0); ++x)
		for (int y = min_cut(1); y <= max_cut(1); ++y)
		{

			Eigen::Vector2d pos;
			indexToPos(Eigen::Vector2i(x, y), pos);

			dist = getDistance(pos);
			dist = min(dist, max_dist);
			dist = max(dist, min_dist);

			pt.x = pos(0);
			pt.y = pos(1);
			pt.z = 0;
			pt.intensity = (dist - min_dist) / (max_dist - min_dist);
			cloud.push_back(pt);
		}

	cloud.width = cloud.points.size();
	cloud.height = 1;
	cloud.is_dense = true;
	cloud.header.frame_id = mp_.frame_id_;
	sensor_msgs::msg::PointCloud2 cloud_msg;
	pcl::toROSMsg(cloud, cloud_msg);
	cloud_msg.header.stamp = node_->now();

	esdf_pub_->publish(cloud_msg);

	// ROS_INFO("pub esdf");
}
void SDFMap::visCallback()
{
	publishMap();
	publish_map();

	// publishMapInflate(false);
	// publishUpdateRange();
	publishESDF();

	// publishUnknown();
	// publishDepth();
}
double SDFMap::getResolution() { return mp_.resolution_; }
void SDFMap::getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size)
{
	ori = mp_.map_origin_, size = mp_.map_size_;
}