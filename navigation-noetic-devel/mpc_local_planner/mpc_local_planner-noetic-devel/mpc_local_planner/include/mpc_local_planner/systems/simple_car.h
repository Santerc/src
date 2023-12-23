/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *********************************************************************/
/*********************************************************************
 *  Modified by Liu Zeyuan
 *  1. about line 82 change the dynamic model of simple car into Sentry model
 *  2. about line 87 change the input Matrix values
 *********************************************************************/
#ifndef SYSTEMS_SIMPLE_CAR_H_
#define SYSTEMS_SIMPLE_CAR_H_

#include <mpc_local_planner/systems/base_robot_se2.h>

#include <cmath>

namespace mpc_local_planner {

/**
 * @brief Simple car model
 *
 * This class implements the dynamics for a simple car model
 * in which the rear wheels are actuated.
 * The front wheels are the steering wheels (for wheelbase > 0).
 * The state vector [x, y, theta] is defined at the center of the rear axle.
 * See [1,2] for a mathematical description and a figure.
 *
 * [1] S. M. LaValle, Planning Algorithms, Cambridge University Press, 2006.
 *     (Chapter 13, http://planning.cs.uiuc.edu/)
 * [2] A. De Luca et al., Feedback Control of a Nonholonomic Car-like Robot,
 *     in Robot Motion Planning and Control (Ed. J.-P. Laumond), Springer, 1998.
 *     (https://homepages.laas.fr/jpl/promotion/chap4.pdf)
 *
 * @see SimpleCarFrontWheelDrivingModel KinematicBicycleModelVelocityInput
 *      BaseRobotSE2 RobotDynamicsInterface corbo::SystemDynamicsInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class SimpleCarModel : public BaseRobotSE2
{
 public:
    //! Default constructor
    SimpleCarModel() = default;

    //! Constructs model with given wheelbase
    SimpleCarModel(double wheelbase) : _wheelbase(wheelbase) {}

    // implements interface method
    SystemDynamicsInterface::Ptr getInstance() const override { return std::make_shared<SimpleCarModel>(); }

    // implements interface method
    int getInputDimension() const override { return 2; }

    // implements interface method
    void dynamics(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u, Eigen::Ref<StateVector> f) const override
    {
        assert(x.size() == getStateDimension());
        assert(u.size() == getInputDimension());
        assert(x.size() == f.size() && "SimpleCarModel::dynamics(): x and f are not of the same size, do not forget to pre-allocate f.");

        f[0] = u[0];
        f[1] = u[1];
        f[2] = 0;
    }

    // implements interface method
    bool getTwistFromControl(const Eigen::Ref<const Eigen::VectorXd>& u, geometry_msgs::Twist& twist) const override
    {
        assert(u.size() == getInputDimension());
        twist.linear.x = u[0];
        twist.linear.y = u[1];
        twist.linear.z = 0;

        twist.angular.x = twist.angular.y = twist.angular.z = 0;

        return true;
    }

    //! Set wheelbase
    void setWheelbase(double wheelbase) { _wheelbase = wheelbase; }
    //! Get wheelbase
    double getWheelbase() const { return _wheelbase; }

 protected:
    double _wheelbase = 1.0;                //Liu Zeyuan: 目前先按这个来吧，具体再说
};

/**
 * @brief Simple car model with front wheel actuation
 *
 * This class implements the dynamics for a simple car model
 * in which the front wheels are actuated and steered (for wheelbase > 0).
 * The state vector [x, y, theta] is defined at the center of the rear axle.
 * See [1] for a mathematical description and a figure.
 *
 * [1] A. De Luca et al., Feedback Control of a Nonholonomic Car-like Robot,
 *     in Robot Motion Planning and Control (Ed. J.-P. Laumond), Springer, 1998.
 *     (https://homepages.laas.fr/jpl/promotion/chap4.pdf)
 *
 * @see SimpleCarFrontWheelDrivingModel BaseRobotSE2 RobotDynamicsInterface
 *      corbo::SystemDynamicsInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class SimpleCarFrontWheelDrivingModel : public SimpleCarModel        //Liu Zeyuan: 前轮驱动的，我们不需要
{
 public:
    //! Default constructor
    SimpleCarFrontWheelDrivingModel() = default;

    //! Constructs model with given wheelbase
    SimpleCarFrontWheelDrivingModel(double wheelbase) : SimpleCarModel(wheelbase) {}

    // implements interface method
    SystemDynamicsInterface::Ptr getInstance() const override { return std::make_shared<SimpleCarFrontWheelDrivingModel>(); }

    // implements interface method
    void dynamics(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u, Eigen::Ref<StateVector> f) const override
    {
        assert(x.size() == getStateDimension());
        assert(u.size() == getInputDimension());
        assert(x.size() == f.size() &&
               "SimpleCarFrontWheelDrivingModel::dynamics(): x and f are not of the same size, do not forget to pre-allocate f.");

        f[0] = u[0];
        f[1] = u[1];
        f[2] = 0;
    }
};

}  // namespace mpc_local_planner

#endif  // SYSTEMS_SIMPLE_CAR_H_
