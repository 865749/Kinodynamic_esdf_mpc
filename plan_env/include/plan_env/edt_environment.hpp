#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_
#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <utility>
#include "plan_env/sdf_map.hpp"
#include "plan_env/obj_predictor.hpp"

namespace fast_planner
{
    class EDTEnvironment
    {
    private:
        ObjPrediction obj_prediction_;
        ObjScale obj_scale_;
        double resolution_inv_;
        double distToBox(int idx, const Eigen::Vector2d &pos, const double &time);
        double minDistToAllBox(const Eigen::Vector2d &pos, const double &time);

    public:
        EDTEnvironment(/* args */)
        {
        }
        ~EDTEnvironment()
        {
        }
        SDFMap::Ptr sdf_map_;
        void init();
        void setMap(SDFMap::Ptr map);
        void setObjPrediction(ObjPrediction prediction);
        void setObjScale(ObjScale scale);
        void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
        void interpolateBilinear(double values[2][2], const Eigen::Vector2d &diff,
                                 double &value, Eigen::Vector2d &grad);
        void evaluateEDTWithGrad(const Eigen::Vector2d &pos, double time,
                                 double &dist, Eigen::Vector2d &grad);
        double evaluateCoarseEDT(Eigen::Vector2d &pos, double time);
        void getMapRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size)
        {
            sdf_map_->getRegion(ori, size);
        }
        typedef shared_ptr<EDTEnvironment> Ptr;
    };
}
#endif