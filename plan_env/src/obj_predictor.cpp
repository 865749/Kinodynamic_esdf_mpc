#include <plan_env/obj_predictor.hpp>

namespace fast_planner
{
    int ObjHistory::queue_size_;
    int ObjHistory::skip_num_;
    rclcpp::Time ObjHistory::global_start_time_;

    void ObjHistory::init(int id)
    {
        clear();
        skip_ = 0;
        obj_idx_ = id;
    }

    void ObjHistory::poseCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg)
    {
        ++skip_;
        if (skip_ < ObjHistory::skip_num_)
            return;

        Eigen::Vector3d pos_t;
        pos_t(0) = msg->pose.position.x, pos_t(1) = msg->pose.position.y;
        pos_t(2) = (rclcpp::Clock().now() - ObjHistory::global_start_time_).seconds();

        history_.push_back(pos_t);
        // cout << "idx: " << obj_idx_ << "pos_t: " << pos_t.transpose() << endl;

        if (history_.size() > queue_size_)
            history_.pop_front();

        skip_ = 0;
    }
}