#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include "plan_env/edt_environment.hpp"
namespace fast_planner
{
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

    class PathNode
    {
    public:
        /* -------------------- */
        Eigen::Vector2i index;
        Eigen::Matrix<double, 4, 1> state;
        double g_score, f_score;
        Eigen::Vector2d input;
        double duration;
        double time; // dyn
        int time_idx;
        PathNode *parent;
        char node_state;

        /* -------------------- */
        PathNode()
        {
            parent = NULL;
            node_state = NOT_EXPAND;
        }
        ~PathNode() {};
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef PathNode *PathNodePtr;

    class NodeComparator
    {
    public:
        bool operator()(PathNodePtr node1, PathNodePtr node2)
        {
            return node1->f_score > node2->f_score;
        }
    };

    template <typename T>
    struct matrix_hash
    {
        std::size_t operator()(const T &matrix) const
        {
            std::size_t seed = 0;
            for (const auto &elem : matrix)
            {
                seed ^= std::hash<typename T::value_type>{}(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    class NodeHashTable
    {
    private:
        /* data */
        std::unordered_map<Eigen::Vector2i, PathNodePtr, matrix_hash<Eigen::Vector2i>>
            data_2d_;
        std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
            data_3d_;

    public:
        NodeHashTable(/* args */) {}
        ~NodeHashTable() {}
        void insert(Eigen::Vector2i idx, PathNodePtr node)
        {
            data_2d_.insert(std::make_pair(idx, node));
        }
        void insert(Eigen::Vector2i idx, int time_idx, PathNodePtr node)
        {
            data_3d_.insert(std::make_pair(
                Eigen::Vector3i(idx(0), idx(1), time_idx), node));
        }

        PathNodePtr find(Eigen::Vector2i idx)
        {
            auto iter = data_2d_.find(idx);
            return iter == data_2d_.end() ? NULL : iter->second;
        }
        PathNodePtr find(Eigen::Vector2i idx, int time_idx)
        {
            auto iter =
                data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
            return iter == data_3d_.end() ? NULL : iter->second;
        }

        void clear()
        {
            data_2d_.clear();
            data_3d_.clear();
        }
    };
    class KinodynamicAstar
    {
    public:
        KinodynamicAstar() {};
        ~KinodynamicAstar();
        void init();
        void setParam(std::shared_ptr<rclcpp::Node> nh);
        void setEnvironment(const EDTEnvironment::Ptr &env);
        void reset();
        std::vector<Eigen::Vector2d> getKinoTraj(double delta_t);
        std::vector<PathNodePtr> getVisitedNodes();

        int search(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel,
                   Eigen::Vector2d start_acc, Eigen::Vector2d end_pt,
                   Eigen::Vector2d end_vel, bool init, bool dynamic = false, double time_start = -1.0);
        void getSamples(double &ts, vector<Eigen::Vector2d> &point_set,
                        vector<Eigen::Vector2d> &start_end_derivatives);
        enum
        {
            REACH_HORIZON = 1,
            REACH_END = 2,
            NO_PATH = 3,
            NEAR_END = 4
        };

    private:
        std::shared_ptr<rclcpp::Node> node_;
        
        vector<PathNodePtr> path_node_pool_;
        std::vector<PathNodePtr> path_nodes_;
        int use_node_num_, iter_num_;

        NodeHashTable expanded_nodes_;
        EDTEnvironment::Ptr edt_environment_;
        Eigen::Matrix<double, 4, 4> phi_; // state transit matrix

        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
        int allocate_num_, check_num_;
        double w_time_, horizon_, lambda_heu_;
        double max_tau_, init_max_tau_;
        double max_vel_, max_acc_;
        bool has_path_ = false;

        Eigen::Vector2d start_vel_, end_vel_, start_acc_;
        double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
        double tie_breaker_;

        double time_origin_;
        Eigen::Vector2d origin_, map_size_3d_;
        Eigen::MatrixXd coef_shot_;
        double t_shot_;
        bool is_shot_succ_ = false;

        double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time);
        vector<double> quartic(double a, double b, double c, double d, double e);
        vector<double> cubic(double a, double b, double c, double d);

        bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
        Eigen::Vector2i posToIndex(Eigen::Vector2d pt);

        void retrievePath(PathNodePtr end_node);
        void stateTransit(Eigen::Matrix<double, 4, 1> &state0, Eigen::Matrix<double, 4, 1> &state1,
                          Eigen::Vector2d um, double tau);
        int timeToIndex(double time);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace fast_planner

#endif