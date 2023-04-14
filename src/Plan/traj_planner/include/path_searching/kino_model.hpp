#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <iostream>


#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "common/basics/basics.h"
#include "common/state/state.h"
#include "common/math/calculations.h"
#include "odeint-v2/boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

namespace  path_searching{

  class KinoModel {
  public:

    KinoModel(){}
    ~KinoModel(){}

    void setParam(common::VehicleParam vp){
  
      vp_ = vp;

    }

    void stateTransit(Eigen::Vector4d &state0,  Eigen::Vector4d &state1,
              Eigen::Vector2d &ctrl_input, double dt) {

      state_   = state0;
      control_ = ctrl_input;
      UpdateInternalState();

      odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

      state_(0) = internal_state_[0];
      state_(1) = internal_state_[1];
      decimal_t theta_tmp = internal_state_[2];
      //hzchzc
      // theta_tmp -= (internal_state_[2]>= kPi) * 2 * kPi;
      // theta_tmp += (internal_state_[2] < -kPi) * 2 * kPi;
      state_(2) = theta_tmp;
      state_(3) = internal_state_[3];

      state1 = state_;
    } 

    // For internal use, but needs to be public for odeint
    typedef boost::array<double, 4> InternalState;
    void operator()(const InternalState &x, InternalState &dxdt,
                                  const double /* t */) {

      dxdt[0] = cos(x[2]) * x[3];
      dxdt[1] = sin(x[2]) * x[3];
      dxdt[2] = tan(control_(0)) * x[3] / vp_.wheel_base();
      dxdt[3] = control_(1);
    }


  private:
  
    void UpdateInternalState(void) {
      internal_state_[0] = state_(0);
      internal_state_[1] = state_(1);
      internal_state_[2] = state_(2);
      internal_state_[3] = state_(3);
    }

    Eigen::Vector4d state_;
    Eigen::Vector2d control_;
    common::VehicleParam vp_;
    InternalState internal_state_;
  };


}
