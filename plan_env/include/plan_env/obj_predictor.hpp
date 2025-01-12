#ifndef _OBJ_PREDICTOR_H_
#define _OBJ_PREDICTOR_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <list>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner
{
    class PolynomialPrediction;
    typedef shared_ptr<vector<PolynomialPrediction>> ObjPrediction;
    typedef shared_ptr<vector<Eigen::Vector2d>> ObjScale;

    /* ========== prediction polynomial ========== */
    class PolynomialPrediction
    {
    private:
        vector<Eigen::Matrix<double, 6, 1>> polys;
        double t1, t2; // start / end

    public:
        PolynomialPrediction(/* args */)
        {
        }
        ~PolynomialPrediction()
        {
        }

        void setPolynomial(vector<Eigen::Matrix<double, 6, 1>> &pls)
        {
            polys = pls;
        }
        void setTime(double t1, double t2)
        {
            this->t1 = t1;
            this->t2 = t2;
        }

        bool valid()
        {
            return polys.size() == 2;
        }

        /* note that t should be in [t1, t2] */
        Eigen::Vector2d evaluate(double t)
        {
            Eigen::Matrix<double, 6, 1> tv;
            tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

            Eigen::Vector2d pt;
            pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]);

            return pt;
        }

        Eigen::Vector2d evaluateConstVel(double t)
        {
            Eigen::Matrix<double, 2, 1> tv;
            tv << 1.0, pow(t, 1);

            Eigen::Vector2d pt;
            pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2));

            return pt;
        }
    };

    /* ========== subscribe and record object history ========== */
    class ObjHistory
    {
    public:
        static int skip_num_;
        static int queue_size_;
        static rclcpp::Time global_start_time_;

        ObjHistory()
        {
        }
        ~ObjHistory()
        {
        }

        void init(int id);

        void poseCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg);

        void clear()
        {
            history_.clear();
        }

        void getHistory(list<Eigen::Vector3d> &his)
        {
            his = history_;
        }

    private:
        list<Eigen::Vector3d> history_; // x,y,t
        int skip_;
        int obj_idx_;
        Eigen::Vector2d scale_; // x,y
    };
} // namespace fast_planner

#endif