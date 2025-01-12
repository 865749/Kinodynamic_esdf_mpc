#include <plan_env/edt_environment.hpp>

namespace fast_planner
{
    void EDTEnvironment::init()
    {
    }
    void EDTEnvironment::setMap(shared_ptr<SDFMap> map)
    {
        this->sdf_map_ = map;
        resolution_inv_ = 1 / sdf_map_->getResolution();
    }

    void EDTEnvironment::setObjPrediction(ObjPrediction prediction)
    {
        this->obj_prediction_ = prediction;
    }

    void EDTEnvironment::setObjScale(ObjScale scale)
    {
        this->obj_scale_ = scale;
    }

    double EDTEnvironment::distToBox(int idx, const Eigen::Vector2d &pos, const double &time)
    {
        // Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluate(time);
        Eigen::Vector2d pos_box = obj_prediction_->at(idx).evaluateConstVel(time);

        Eigen::Vector2d box_max = pos_box + 0.5 * obj_scale_->at(idx);
        Eigen::Vector2d box_min = pos_box - 0.5 * obj_scale_->at(idx);

        Eigen::Vector2d dist;

        for (int i = 0; i < 2; i++)
        {
            dist(i) = pos(i) >= box_min(i) && pos(i) <= box_max(i) ? 0.0 : min(fabs(pos(i) - box_min(i)), fabs(pos(i) - box_max(i)));
        }

        return dist.norm();
    }

    double EDTEnvironment::minDistToAllBox(const Eigen::Vector2d &pos, const double &time)
    {
        std::cout << "asd" << std::endl;

        double dist = std::numeric_limits<double>::max();
        for (int i = 0; i < obj_prediction_->size(); i++)
        {
        std::cout << "asd" << std::endl;

            double di = distToBox(i, pos, time);
            if (di < dist)
                dist = di;
        }

        return dist;
    }

    void EDTEnvironment::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
    {
        for (int x = 0; x < 2; x++)
        {
            for (int y = 0; y < 2; y++)
            {

                dists[x][y] = sdf_map_->getDistance(pts[x][y]);
            }
        }
    }

    void EDTEnvironment::interpolateBilinear(double values[2][2],
                                             const Eigen::Vector2d &diff,
                                             double &value,
                                             Eigen::Vector2d &grad)
    {
        // bilinear interpolation
        double v0 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0];
        double v1 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1];

        value = (1 - diff(1)) * v0 + diff(1) * v1;

        // Gradient calculation
        grad[1] = (v1 - v0) * resolution_inv_;
        grad[0] = (1 - diff(1)) * (values[1][0] - values[0][0]) + diff(1) * (values[1][1] - values[0][1]);
        grad[0] *= resolution_inv_;
    }

    void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector2d &pos,
                                             double time, double &dist,
                                             Eigen::Vector2d &grad)
    {
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];
        sdf_map_->getSurroundPts(pos, sur_pts, diff);

        double dists[2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateBilinear(dists, diff, dist, grad);
    }

    double EDTEnvironment::evaluateCoarseEDT(Eigen::Vector2d &pos, double time)
    {
        double d1 = sdf_map_->getDistance(pos);
        if (time < 0.0)
        {
            return d1;
        }
        else
        {
            double d2 = minDistToAllBox(pos, time);
            return min(d1, d2);
        }
    }

    // EDTEnvironment::
}