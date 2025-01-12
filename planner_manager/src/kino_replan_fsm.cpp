#include "planner_manager/kino_replan_fsm.hpp"
using namespace fast_planner;

bool kino_replan_fsm::checkTrajCollision(double &distance)
{

    double t_now = (rclcpp::Clock().now() - start_time).seconds();

    double tm, tmp;
    position_traj_.getTimeSpan(tm, tmp);
    Eigen::Vector2d cur_pt = position_traj_.evaluateDeBoor(tm + t_now);

    double radius = 0.0;
    Eigen::Vector2d fut_pt;
    double fut_t = 0.02;

    while (radius < 3.0 && t_now + fut_t < duration_)
    {
        fut_pt = position_traj_.evaluateDeBoor(tm + t_now + fut_t);
        if (CarType)
        {
            for (const auto &point : local_points)
            {
                Eigen::Vector2d correct = {fut_pt.x() + point.x() * cos(odom_rpy(2)) - point.y() * sin(odom_rpy(2)),
                                           fut_pt.y() + point.x() * sin(odom_rpy(2)) + point.y() * cos(odom_rpy(2))};
                double dist = edt_environment_->evaluateCoarseEDT(correct, -1.0);
                if (dist <= 0 || edt_environment_->sdf_map_->isInMap(correct) == 0)
                {
                    distance = radius;
                    return false;
                }
            }
        }
        else
        {
            double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
            if (dist <= Car_radius || edt_environment_->sdf_map_->isInMap(fut_pt) == 0)
            {
                distance = radius;
                return false;
            }
        }

        radius = (fut_pt - cur_pt).norm();
        fut_t += 0.02;
    }

    return true;
}
bool kino_replan_fsm::plan(Eigen::Vector2d position,Eigen::Vector2d vel_,Eigen::Vector2d acc_,Eigen::Vector2d end_pt ,Eigen::Vector2d end_vel)
{
    kino_path_finder_->reset();
    start_time = rclcpp::Clock().now();

    int status = kino_path_finder_->search(position, vel_, acc_, end_pt, end_vel, true);
    auto an = rclcpp::Clock().now();

    if (status == KinodynamicAstar::NO_PATH)
    {
        cout << "[kino replan]: kinodynamic search fail!" << endl;

        // retry searching with discontinuous initial state
        kino_path_finder_->reset();
        status = kino_path_finder_->search(position, vel_, acc_, end_pt, end_vel, false);

        if (status == KinodynamicAstar::NO_PATH)
        {
            cout << "[kino replan]: Can't find path." << endl;
            have_traj = false;

            return false;
        }
        else
        {
            cout << "[kino replan]: retry search success." << endl;
        }
    }
    else
    {
        cout << "[kino replan]: kinodynamic search success." << endl;
    }
    auto n = rclcpp::Clock().now();
    std::cout << "搜索耗时::" << (n - an).seconds() * 1000 << "ms" << std::endl;
    vector<Eigen::Vector2d> kino_path_ = kino_path_finder_->getKinoTraj(0.01);
    double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
    vector<Eigen::Vector2d> point_set, start_end_derivatives;
    kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    int cost_function = BsplineOptimizer::NORMAL_PHASE;
    if (status != KinodynamicAstar::REACH_END)
    {
        cost_function |= BsplineOptimizer::ENDPOINT;
    }
    an = rclcpp::Clock().now();
    ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

    NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);

    bool feasible = pos.checkFeasibility(false);
    int iter_num = 0;
    while (!feasible && rclcpp::ok())
    {
        feasible = pos.reallocateTime();

        if (++iter_num >= 3)
            break;
    }
    n = rclcpp::Clock().now();
    std::cout << "优化耗时::" << (n - an).seconds() * 1000 << "ms" << std::endl;

    // have_traj = true;
    position_traj_ = pos;
    velocity_traj_ = position_traj_.getDerivative();
    acceleration_traj_ = velocity_traj_.getDerivative();
    duration_ = position_traj_.getTimeSum();
    planYaw(start_yaw_,target_yaw);

    bspline_process(10086);
    vector<Eigen::Vector2d>
        traj_pts;
    double tm, tmp;
    pos.getTimeSpan(tm, tmp);

    for (double t = tm; t <= tmp; t += 0.01)
    {
        Eigen::Vector2d pt = pos.evaluateDeBoor(t);
        traj_pts.push_back(pt);
    }

    geometry_msgs::msg::PoseStamped current_pose;
    path_.header.stamp = this->now();
    path_.header.frame_id = "/odom";
    path_.poses.clear();
    for (const auto &a : traj_pts)
    {
        current_pose.pose.position.x = a(0);
        current_pose.pose.position.y = a(1);
        current_pose.pose.position.z = 0.;
        path_.poses.push_back(current_pose);
    }
    // auto VN = kino_path_finder_->getVisitedNodes();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // cloud->header.frame_id = "odom"; // 点云的参考坐标系
    // cloud->width = VN.size();       // 点云点数
    // cloud->height = 1;              // 非结构化点云
    // cloud->is_dense = true;
    // for (auto &a : VN)
    // {
    //     pcl::PointXYZ point;
    //     point.x = static_cast<float>(a->state(0)); // x 坐标
    //     point.y = static_cast<float>(a->state(1));  // y 坐标
    //     point.z = static_cast<float>(0.);       // z 坐标
    //     cloud->points.push_back(point);
    // }
    // pcl::toROSMsg(*cloud, cloud_msg___); // PCL 点云转 ROS2 消息
    // cloud_msg___.header.stamp = this->get_clock()->now();
    path_pub_->publish(path_);
    have_traj = true;

    return true;
}
void kino_replan_fsm::bspline_process(double t)
{

    traj.clear();
    traj.push_back(position_traj_);
    traj.push_back(traj[0].getDerivative());
    traj.push_back(yaw_traj_);
    traj.push_back(yaw_traj_.getDerivative());
    if (t >= 100)
        traj_duration = traj[0].getTimeSum();
    else
        traj_duration = t;
}
vector<double> calculate_ref_states(const vector<double> &ref_x,
                                    const vector<double> &ref_y,
                                    const vector<double> &ref_q,
                                    const double &reference_v,
                                    const double &reference_w)
{
    vector<double> result;
    for (int i = 0; i < N; i++)
    {
        result.push_back(ref_x[i]);
        result.push_back(ref_y[i]);
        result.push_back(ref_q[i]);
        result.push_back(0);
        result.push_back(0);
    }
    return result;
}
vector<double> update_states(vector<double> state, double vx_cmd, double vy_cmd, double w_cmd)
{
    // based on kinematic model
    double x0 = state[0];
    double y0 = state[1];
    double q0 = state[2];
    double vx = vx_cmd;
    double vy = vy_cmd;
    double w0 = w_cmd;

    double x1 = x0 + (vx * cos(q0) - vy * sin(q0)) * Ts;
    double y1 = y0 + (vx * sin(q0) + vy * cos(q0)) * Ts;
    double q1 = q0 + w0 * Ts;
    return {x1, y1, q1};
}
vector<double> motion_prediction(const vector<double> &cur_states,
                                 const vector<vector<double>> &prev_u)
{
    vector<double> old_vx_cmd = prev_u[0];
    vector<double> old_vy_cmd = prev_u[1];
    vector<double> old_w_cmd = prev_u[2];

    vector<vector<double>> predicted_states;
    predicted_states.push_back(cur_states);

    for (int i = 0; i < N; i++)
    {
        vector<double> cur_state = predicted_states[i];
        // yaw angle compensation of overflow
        if (cur_state[3] > M_PI)
        {
            cur_state[3] -= 2 * M_PI;
        }
        if (cur_state[3] < -M_PI)
        {
            cur_state[3] += 2 * M_PI;
        }
        vector<double> next_state = update_states(cur_state, old_vx_cmd[i], old_vy_cmd[i], old_w_cmd[i]);
        predicted_states.push_back(next_state);
    }

    vector<double> result;
    for (int i = 0; i < (ACADO_N + 1); ++i)
    {
        for (int j = 0; j < NX; ++j)
        {
            result.push_back(predicted_states[i][j]);
        }
    }
    return result;
}
void kino_replan_fsm::publish_control_cmd()
{
    if (!have_traj)
        return;

    current_state << init_pose, start_yaw_(0);
    auto time_now = rclcpp::Clock().now();
    double t_cur = (time_now - start_time).seconds();
    Eigen::Vector2d pos, vel, pos_f;
    double yaw, yawdot;
    visited_pub->publish(cloud_msg___);
    // std::cout << "traj_duration" << traj_duration << "t_cur" << t_cur << std::endl;
    // Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N, 3);
    vector<double> ref_statess;
    if (t_cur + (N - 1) * Ts <= traj_duration && t_cur > 0)
    {
        for (int i = 0; i < N; ++i)
        {
            pos = traj[0].evaluateDeBoorT(t_cur + i * Ts);
            vel = traj[1].evaluateDeBoorT(t_cur + i * Ts);
            yaw = traj[2].evaluateDeBoorT(t_cur + i * Ts)[0];
            yawdot = traj[3].evaluateDeBoorT(t_cur + i * Ts)[0];

            ref_statess.push_back(pos[0]);
            ref_statess.push_back(pos[1]);
            ref_statess.push_back(yaw);
            ref_statess.push_back(vel[0]);
            ref_statess.push_back(vel[1]);
            ref_statess.push_back(yawdot);
        }
    }
    else if (t_cur + (N - 1) * Ts > traj_duration && t_cur < traj_duration)
    {
        int more_num = (t_cur + (N - 1) * Ts - traj_duration) / Ts;
        for (int i = 0; i < N - more_num; ++i)
        {
            pos = traj[0].evaluateDeBoorT(t_cur + i * Ts);
            vel = traj[1].evaluateDeBoorT(t_cur + i * Ts);
            yaw = traj[2].evaluateDeBoorT(t_cur + i * Ts)[0];
            yawdot = traj[3].evaluateDeBoorT(t_cur + i * Ts)[0];

            ref_statess.push_back(pos[0]);
            ref_statess.push_back(pos[1]);
            ref_statess.push_back(yaw);
            ref_statess.push_back(vel[0]);
            ref_statess.push_back(vel[1]);
            ref_statess.push_back(yawdot);
        }
        for (int i = N - more_num; i < N; ++i)
        {
            pos = traj[0].evaluateDeBoorT(traj_duration);
            vel.setZero();
            yaw = traj[2].evaluateDeBoorT(traj_duration)[0];
            yawdot = traj[3].evaluateDeBoorT(traj_duration)[0];

            ref_statess.push_back(pos[0]);
            ref_statess.push_back(pos[1]);
            ref_statess.push_back(yaw);
            ref_statess.push_back(vel[0]);
            ref_statess.push_back(vel[1]);
            ref_statess.push_back(yawdot);
        }
    }
    else if (t_cur >= traj_duration)
    {
        pos = traj[0].evaluateDeBoorT(traj_duration);
        vel.setZero();
        yaw = traj[2].evaluateDeBoorT(traj_duration)[0];
        yawdot = traj[3].evaluateDeBoorT(traj_duration)[0];
        for (int i = 0; i < N; ++i)
        {
            ref_statess.push_back(pos[0]);
            ref_statess.push_back(pos[1]);
            ref_statess.push_back(yaw);
            ref_statess.push_back(vel[0]);
            ref_statess.push_back(vel[1]);
            ref_statess.push_back(yawdot);
        }
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    rclcpp::Time t1, t2;
    t1 = rclcpp::Clock().now();
    std::vector<double> stdVec(current_state.data(), current_state.data() + current_state.size());
    vector<double> predicted_states = motion_prediction(stdVec, mpc_ptr->control_output);
    mpc_ptr->control_output = mpc_ptr->solve(predicted_states, ref_statess);
    t2 = rclcpp::Clock().now();
    // std::cout << "cost_time" << (t2-t1).seconds()* 1000 << "ms" << std::endl;
    // rmoss_interfaces::msg::ChassisCMDmind cmd_;
    auto cmd_ = geometry_msgs::msg::Twist();
    // rmoss_interfaces::msg::GimbalCmd cmd_gimbal;
    geometry_msgs::msg::Twist twist;
    // double cos_pitch = std::cos(gimbal_rpy(1));
    // double cos_roll = std::cos(gimbal_rpy(0));
    // if (std::abs(cos_pitch) < 1e-6)
    //     cos_pitch = 1e-6;
    // if (std::abs(cos_roll) < 1e-6)
    //     cos_roll = 1e-6;
    twist.linear.x = mpc_ptr->control_output[0][0];
    twist.linear.y = mpc_ptr->control_output[1][0];
    twist.angular.z = mpc_ptr->control_output[2][0];
    // cmd_gimbal.yaw_type = 1;
    // cmd_gimbal.position.yaw = yaw;
    // cmd_gimbal.position.pitch = 0.52;
    cmd_ = twist;
    // cmd_.mode = 1;
    if (control_output)
        cmd_vel_pub->publish(cmd_);
    // cmd_Gimbal_pub->publish(cmd_gimbal);
    // cout << "u:" << mpc_ptr->control_output[0][0] << " " << "r:" << mpc_ptr->control_output[1][0] << endl;
    predict_path.poses.clear();
    predict_path.header.frame_id = "/odom";
    predict_path.header.stamp = rclcpp::Clock().now();
    geometry_msgs::msg::PoseStamped pose_msg;
    geometry_msgs::msg::Point pt;
    for (int i = 0; i < ACADO_N; i++)
    {
        pose_msg.pose.position.x = acadoVariables.x[NX * i + 0];
        pose_msg.pose.position.y = acadoVariables.x[NX * i + 1];
        predict_path.poses.push_back(pose_msg);
    }
    pose_msg.pose.position.x = position_traj_.evaluateDeBoorT((rclcpp::Clock().now() - start_time).seconds())(0);
    pose_msg.pose.position.y = position_traj_.evaluateDeBoorT((rclcpp::Clock().now() - start_time).seconds())(1);
    predict_path.poses.push_back(pose_msg);
    predict_path_pub->publish(predict_path);

    // predict_path.poses.clear();
}
void calcNextYaw(const double &last_yaw, double &yaw)
{
    // round yaw to [-PI, PI]

    double round_last = last_yaw;

    while (round_last < -M_PI)
    {
        round_last += 2 * M_PI;
    }
    while (round_last > M_PI)
    {
        round_last -= 2 * M_PI;
    }

    double diff = yaw - round_last;

    if (fabs(diff) <= M_PI)
    {
        yaw = last_yaw + diff;
    }
    else if (diff > M_PI)
    {
        yaw = last_yaw + diff - 2 * M_PI;
    }
    else if (diff < -M_PI)
    {
        yaw = last_yaw + diff + 2 * M_PI;
    }
}
void kino_replan_fsm::planYaw(const Eigen::Vector2d &start_yaw,const double end_yaw_set)
{
    RCLCPP_INFO(this->get_logger(), "plan yaw");

    auto t1 = rclcpp::Clock().now();
    // calculate waypoints of heading

    auto &pos = position_traj_;
    double duration = pos.getTimeSum();

    double dt_yaw = 0.3;
    int seg_num = ceil(duration / dt_yaw);
    dt_yaw = duration / seg_num;

    const double forward_t = 2.0;
    double last_yaw = start_yaw(0);
    vector<Eigen::Vector2d> waypts;
    vector<int> waypt_idx;

    // seg_num -> seg_num - 1 points for constraint excluding the boundary states

    for (int i = 0; i < seg_num; ++i)
    {
        double tc = i * dt_yaw;
        Eigen::Vector2d pc = pos.evaluateDeBoorT(tc);

        double tf = min(duration, tc + forward_t);
        Eigen::Vector2d pf = pos.evaluateDeBoorT(tf);
        Eigen::Vector2d pd = pf - pc;

        Eigen::Vector2d waypt;
        if (pd.norm() > 1e-6)
        {
            waypt(0) = atan2(pd(1), pd(0));
            waypt(1) = 0.0;
            calcNextYaw(last_yaw, waypt(0));
        }
        else
        {
            waypt = waypts.back();
        }
        waypts.push_back(waypt);
        waypt_idx.push_back(i);
    }

    // calculate initial control points with boundary state constraints

    Eigen::MatrixXd yaw(seg_num + 3, 1);
    yaw.setZero();

    Eigen::Matrix2d states2pts;
    states2pts << 1.0, -2.0 * dt_yaw,
        1.0, dt_yaw * dt_yaw;
    yaw.block(0, 0, 2, 1) = states2pts * start_yaw;

    // Eigen::Vector2d end_v = pos.getDerivative().evaluateDeBoorT(duration - 0.1);
    Eigen::Vector2d end_yaw(end_yaw_set, 0);
    calcNextYaw(last_yaw, end_yaw(0));
    yaw.block(seg_num, 0, 2, 1) = states2pts * end_yaw;

    // solve
    bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
    int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
    yaw = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

    // update traj info
    yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
    yawdot_traj_ = yaw_traj_.getDerivative();
    // local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

    // vector<double> path_yaw;
    // for (int i = 0; i < waypts.size(); ++i)
    //     path_yaw.push_back(waypts[i][0]);
    // plan_data_.path_yaw_ = path_yaw;
    // plan_data_.dt_yaw_ = dt_yaw;
    // plan_data_.dt_yaw_path_ = dt_yaw;

    std::cout << "plan heading: " << (rclcpp::Clock().now() - t1).seconds() << std::endl;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fast_planner::kino_replan_fsm)
