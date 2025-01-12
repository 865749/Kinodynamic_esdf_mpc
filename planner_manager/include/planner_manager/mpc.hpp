#ifndef MPC_H
#define MPC_H

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "Eigen/Eigen"
#include "tf2/utils.h"
#include "../../../src/planner_manager/omni_model/acado_common.h"
#include "../../../src/planner_manager/omni_model/acado_auxiliary_functions.h"
/* Some convenient definitions. */
#define NX ACADO_NX   /* Number of differential state variables.  */
#define NXA ACADO_NXA /* Number of algebraic variables. */
#define NU ACADO_NU   /* Number of control inputs. */
#define NOD ACADO_NOD /* Number of online data values. */

#define NY ACADO_NY   /* Number of measurements/references on nodes 0..N - 1. */
#define NYN ACADO_NYN /* Number of measurements/references on node N. */

#define N ACADO_N /* Number of intervals in the horizon. */

#define NUM_STEPS 10 /* Number of real-time iterations. */
#define VERBOSE 1    /* Show iterations: 1, silent: 0.  */

#define Ts 0.1 // sampling time
#define Lf 1.0

using namespace std;

class Mpc
{
private:
    double weight_p, weight_yaw, weight_v, weight_w;

public:
    Mpc(){};

    std::shared_ptr<rclcpp::Node> node_;
    void init_weight();
    void init(std::shared_ptr<rclcpp::Node> nh)
    {
        node_ = nh;
        weight_p = node_->declare_parameter<double>("mpc.weight_p", 10.0);
        weight_yaw = node_->declare_parameter<double>("mpc.weight_yaw", 10.0);
        weight_v = node_->declare_parameter<double>("mpc.weight_v", 0.3);
        weight_w = node_->declare_parameter<double>("mpc.weight_w", 0.3);
        control_output = init_acado();
    };
    vector<vector<double>> init_acado();
    vector<vector<double>> control_output;
    vector<vector<double>> solve(vector<double> states, vector<double> desired_state);
};

#endif