#include "LaneKeepingSystem/PIDController.hpp"
namespace Xycar {

template <typename PREC>
PREC PIDController<PREC>::getControlOutput(PREC error) {
    PREC derivative = error - mPrevError;
    mIntegral += error;
    mPrevError = error;
    return mKp * error + mKi * mIntegral + mKd * derivative;
}
} // namespace Xycar