#include "planner_manager/mpc.hpp"
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;
void Mpc::init_weight()
{
    for (int i = 0; i < N; i++)
    {
        // Setup diagonal entries
        acadoVariables.W[NY * NY * i + (NY + 1) * 0] = weight_p;
        acadoVariables.W[NY * NY * i + (NY + 1) * 1] = weight_p;
        acadoVariables.W[NY * NY * i + (NY + 1) * 2] = weight_yaw;
        acadoVariables.W[NY * NY * i + (NY + 1) * 3] = weight_v;
        acadoVariables.W[NY * NY * i + (NY + 1) * 4] = weight_v;
        acadoVariables.W[NY * NY * i + (NY + 1) * 5] = weight_w;
    }
    acadoVariables.WN[(NYN + 1) * 0] = weight_p;
    acadoVariables.WN[(NYN + 1) * 1] = weight_p;
    acadoVariables.WN[(NYN + 1) * 2] = weight_yaw;
}

vector<vector<double>> Mpc::init_acado()
{
    /* Initialize the solver. */
    acado_initializeSolver();

    /* Initialize the states and controls. */
    for (int i = 0; i < NX * (N + 1); ++i)
        acadoVariables.x[i] = 0.0;
    for (int i = 0; i < NU * N; ++i)
        acadoVariables.u[i] = 0.0;

    /* Initialize the measurements/reference. */
    for (int i = 0; i < NY * N; ++i)
        acadoVariables.y[i] = 0.0;
    for (int i = 0; i < NYN; ++i)
        acadoVariables.yN[i] = 0.0;

    acado_preparationStep();

    // 创建用于存储控制输出的向量
    vector<double> control_output_vx;
    vector<double> control_output_vy;
    vector<double> control_output_w;

    // 提取控制输出
    for (int i = 0; i < ACADO_N; ++i)
    {
        // 有三个控制输出：vx, vy, w
        control_output_vx.push_back(acadoVariables.u[i * ACADO_NU + 0]);
        control_output_vy.push_back(acadoVariables.u[i * ACADO_NU + 1]);
        control_output_w.push_back(acadoVariables.u[i * ACADO_NU + 2]);
    }

    // 初始化权重（如果需要）
    init_weight();

    // 返回包含三个控制输出向量的向量
    return {control_output_vx, control_output_vy, control_output_w};
}

vector<vector<double>> Mpc::solve(vector<double> states, vector<double> desired_state)
{
    /* Some temporary variables. */
    int i, iter;
    acado_timer t;

    /* Initialize the states and controls. */
    for (i = 0; i < NX * (N + 1); ++i)  
    {
        acadoVariables.x[i] = (real_t) states[i];
    }
    for (i = 0; i < NX; ++i)
    {
        acadoVariables.x0[i] = (real_t)states[i];
    }

    /* Initialize the measurements/reference. */
    for (i = 0; i < NY * N; ++i)
    {
        acadoVariables.y[i] = (real_t)desired_state[i];
    }
    for (i = 0; i < NYN; ++i)
    {
        acadoVariables.yN[i] = (real_t)desired_state[NY * (N - 1) + i];
    }

    // /* Prepare first step */
    acado_preparationStep();

    /* Get the time before start of the loop. */
    acado_tic(&t);

    /* The "real-time iterations" loop. */
    for (iter = 0; iter < NUM_STEPS; ++iter)
    {
        /* Perform the feedback step. */
        acado_feedbackStep();
        acado_preparationStep();
    }

    /* Read the elapsed time. */
    real_t te = acado_toc(&t);

    // 提取控制输出
    vector<double> control_output_vx;
    vector<double> control_output_vy;
    vector<double> control_output_w;
    real_t *u = acado_getVariablesU();
    for (int i = 0; i < ACADO_N; ++i)
    {
        control_output_vx.push_back((double)u[i * ACADO_NU + 0]);
        control_output_vy.push_back((double)u[i * ACADO_NU + 1]);
        control_output_w.push_back((double)u[i * ACADO_NU + 2]);
    }

    // 返回第一个时间步的控制输出
    return {control_output_vx, control_output_vy, control_output_w};
}