#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <cstdint>
#include <memory>

namespace Xycar {
/**
 * @brief PID Controller Class
 * @tparam PREC Precision of data
 */
template <typename PREC>
class PIDController {
 public:
  PIDController(PREC kp, PREC ki, PREC kd);
  /**
   * @brief Get the Control Output object
   * @param error
   * @return u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
   */
  PREC getControlOutput(PREC error);

 private:
  PREC mKp;  // Proportional gain
  PREC mKi;  // Integral gain
  PREC mKd;  // Derivative gain

  PREC mPrevError;  // Previous error
  PREC mIntegral;   // Integral of error
};
}  // namespace Xycar
#endif  // PID_CONTROLLER_HPP_
