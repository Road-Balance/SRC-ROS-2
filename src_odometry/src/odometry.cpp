/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 * Author: Masaru Morita
 */

#include "src_odometry/odometry.hpp"

#include <boost/bind.hpp>

namespace ackermann_steering_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
      : timestamp_(0.0), x_(0.0), y_(0.0), heading_(0.0), linear_(0.0), angular_(0.0), wheel_separation_h_(0.0), wheel_radius_(0.0), rear_wheel_old_pos_(0.0), velocity_rolling_window_size_(velocity_rolling_window_size), linear_acc_(RollingWindow::window_size = velocity_rolling_window_size), angular_acc_(RollingWindow::window_size = velocity_rolling_window_size), integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2))
  {
  }

  void Odometry::init(const rclcpp::Time &time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::updateWithHeading(double rear_wheel_pos, double heading_angle, const rclcpp::Time &time)
  {
    /// Get current wheel joint positions:
    const double rear_wheel_cur_pos = rear_wheel_pos * wheel_radius_;
    /// Estimate velocity of wheels using old and current position:
    const double rear_wheel_est_vel = rear_wheel_cur_pos - rear_wheel_old_pos_;

    double heading_difference = heading_angle - heading_angle_old_;
    // Exception handler required
    if (fabs(heading_difference) > 3.0)
    {
      if (heading_difference < 0.0)
        heading_difference = -1 * (heading_difference + 6.2831);
      else
        heading_difference = -1 * (heading_difference - 6.2831);
    }
    /// Update old position with current:
    rear_wheel_old_pos_ = rear_wheel_cur_pos;
    heading_angle_old_ = heading_angle;

    /// Compute linear and angular diff:
    const double linear = rear_wheel_est_vel;
    const double angular = heading_difference;

    std::cout << "angular : " << angular << std::endl;
    // std::cout << "simple angular : " << wheel_separation_h_ / tan(front_steer_pos) << std::endl;

    /// Integrate odometry:
    // integrate_fun_(linear, angular);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).seconds();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear / dt);
    angular_acc_(angular / dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    x_ += rear_wheel_est_vel * cos(heading_angle);
    y_ += rear_wheel_est_vel * sin(heading_angle);

    std::cout << "heading_angle : " << heading_angle << std::endl;
    std::cout << "cos(heading_angle) : " << cos(heading_angle) << std::endl;

    return true;
  }

  bool Odometry::update(double rear_wheel_pos, double front_steer_pos, const rclcpp::Time &time)
  {
    /// Get current wheel joint positions:
    const double rear_wheel_cur_pos = rear_wheel_pos * wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double rear_wheel_est_vel = rear_wheel_cur_pos - rear_wheel_old_pos_;

    // std::cout << "rear_wheel_pos : " << rear_wheel_pos << " / " <<
    //   "front_steer_pos" << front_steer_pos << " / " <<
    //   "rear_wheel_est_vel : " << rear_wheel_est_vel << std::endl;

    /// Update old position with current:
    rear_wheel_old_pos_ = rear_wheel_cur_pos;

    /// Compute linear and angular diff:
    const double linear = rear_wheel_est_vel;
    const double angular = tan(front_steer_pos) * linear / wheel_separation_h_;

    // std::cout << "angular : " << angular << std::endl;
    // std::cout << "simple angular : " << wheel_separation_h_ / tan(front_steer_pos) << std::endl;

    /// Integrate odometry:
    integrate_fun_(linear, angular);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).seconds();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear / dt);
    angular_acc_(angular / dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }

  void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Time &time)
  {
    /// Save last linear and angular velocity:
    linear_ = linear;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = (time - timestamp_).seconds();
    timestamp_ = time;
    integrate_fun_(linear * dt, angular * dt);
  }

  void Odometry::setWheelParams(double wheel_separation_h, double wheel_radius)
  {
    wheel_separation_h_ = wheel_separation_h;
    wheel_radius_ = wheel_radius;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_ += linear * cos(direction);
    y_ += linear * sin(direction);
    heading_ += angular;
  }

  /**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
  void Odometry::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear / angular;

      std::cout << "r : " << r << std::endl;

      heading_ += angular;
      x_ += r * (sin(heading_) - sin(heading_old));
      y_ += -r * (cos(heading_) - cos(heading_old));
    }
  }

  void Odometry::resetAccumulators()
  {
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace diff_drive_controller
