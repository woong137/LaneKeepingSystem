#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {

template <typename PREC>
PIDController<PREC>::PIDController(PREC kp, PREC ki, PREC kd)
    : mKp(kp), mKi(ki), mKd(kd), mPrevError(0), mIntegral(0) {}

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(PREC error) {
    PREC derivative = error - mPrevError;
    mIntegral += error;
    mPrevError = error;
    return mKp * error + mKi * mIntegral + mKd * derivative;
}
template class PIDController<float>;
template class PIDController<double>;
} // namespace Xycar