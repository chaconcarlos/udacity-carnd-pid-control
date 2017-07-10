/* INCLUDES ******************************************************************/

#include "PidController.h"

#include <cmath>
#include <limits>

#include <iostream>

/* DEFINITIONS ***************************************************************/

//static const double INITIAL_P_VALUE    = 0.165; /* Initial Parameter for P */
//static const double INITIAL_I_VALUE    = 0.000; /* Initial Parameter for I */
//static const double INITIAL_D_VALUE    = 3;     /* Initial Parameter for D */
static const double INITIAL_P_VALUE    = 0.23595;
static const double INITIAL_I_VALUE    = 0.0030;
static const double INITIAL_D_VALUE    = 2.757;
static const int    SETTLE_STEPS_COUNT = 100;
static const int    TWIDDLE_LOOP_STEPS = 1150;
static const int    TUNE_STEPS_TOTAL   = SETTLE_STEPS_COUNT + TWIDDLE_LOOP_STEPS;

/* CLASS IMPLEMENTATION ******************************************************/

PidController::PidController()
: m_isEnabledParameterTuning(false)
, m_firstAdjustmentDone(false)
, m_secondAdjustmentDone(false)
, m_isTuningIterationFinished(false)
, m_currentIteration(0)
, m_currentStep(0)
, m_currentParameterIndex(0)
, m_proportionalError(0)
, m_integralError(0)
, m_derivativeError(0)
, m_bestError(std::numeric_limits<double>::max())
, m_accumulatedError(0)
{
  initialize();
}

PidController::PidController(double Kp, double Ki, double Kd)
: m_isEnabledParameterTuning(false)
, m_firstAdjustmentDone(false)
, m_secondAdjustmentDone(false)
, m_isTuningIterationFinished(false)
, m_currentIteration(0)
, m_currentStep(0)
, m_currentParameterIndex(0)
, m_proportionalError(0)
, m_integralError(0)
, m_derivativeError(0)
, m_bestError(std::numeric_limits<double>::max())
, m_accumulatedError(0)
{
  initialize();
}

PidController::~PidController()
{
}

void
PidController::enableParameterTuning(bool enable)
{
  m_isEnabledParameterTuning = enable;
}

void
PidController::updateError(double cte)
{
  m_derivativeError   = cte - m_proportionalError;
  m_proportionalError = cte;
  m_integralError    += cte;

  if (m_isEnabledParameterTuning && m_currentStep == 0)
  {
    m_isTuningIterationFinished = false;
    std::cout << "Current Iteration:   " << m_currentIteration << std::endl;
    std::cout << "Current best values: " << "[" << m_bestKs[0] << "," << m_bestKs[1] << "," << m_bestKs[2] << "]" << std::endl;
    std::cout << "Current best error:  " << m_bestError        << std::endl;
    ++m_currentIteration;
  }

  ++m_currentStep;

  tune();
}

double
PidController::getTotalError()
{
  double totalError = - (m_ks[0] * m_proportionalError + m_ks[1] * m_integralError + m_ks[2] * m_derivativeError);

  if (m_currentStep > 0 && m_currentStep % 100 == 0)
    std::cout << "Current Step: " << m_currentStep << " CTE: " << m_proportionalError << " TotalError: " << totalError << std::endl;

  return totalError;
}

bool
PidController::isTuningIterationFinished()
{
  return m_isTuningIterationFinished;
}

void
PidController::initialize()
{
  m_ks     = std::vector<double>(3, 0);
  m_ks[0]  = INITIAL_P_VALUE;
  m_ks[1]  = INITIAL_I_VALUE;
  m_ks[2]  = INITIAL_D_VALUE;
  m_bestKs = m_ks;
  m_dp     = { 0.1 * m_ks[0], 0.1 * m_ks[1], 0.1 * m_ks[2] };
}

void
PidController::tune()
{
  bool shouldSaveError = m_currentStep % (TUNE_STEPS_TOTAL) > SETTLE_STEPS_COUNT;
  bool shouldTune      = m_currentStep % (TUNE_STEPS_TOTAL) == 0;

  if (m_isEnabledParameterTuning == false)
    return;

  if (shouldSaveError)
    m_accumulatedError += pow(m_proportionalError, 2);

  if (shouldTune)
  {
    tuneCurrentParameter();
    std::cout << "Tuned Parameters values: " << "[" << m_ks[0] << "," << m_ks[1] << "," << m_ks[2] << "]" << std::endl;
  }
}

void
PidController::tuneCurrentParameter()
{
  if (m_accumulatedError < m_bestError)
  {
    std::cout << "I improved the error! Was " << m_bestError << " and now is " << m_accumulatedError << std::endl;

    m_bestError = m_accumulatedError;

    if (m_currentStep > TUNE_STEPS_TOTAL)
      m_dp[m_currentParameterIndex] *= 1.1;

    m_bestKs = m_ks;

    setNextParameter();
  }
  else if (m_firstAdjustmentDone == false)
  {
    std::cout << "Doing first adjustment to parameter " << m_currentParameterIndex << "..." << std::endl;

    m_ks[m_currentParameterIndex] += m_dp[m_currentParameterIndex];
    m_firstAdjustmentDone          = true;
  }
  else if (m_secondAdjustmentDone == false)
  {
    std::cout << "Doing second adjustment to parameter " << m_currentParameterIndex << "..." << std::endl;

    m_ks[m_currentParameterIndex] -= 2 * m_dp[m_currentParameterIndex];
    m_secondAdjustmentDone         = true;
  }
  else
  {
    std::cout << "Done with parameter " << m_currentParameterIndex << "..." << std::endl;
    m_ks[m_currentParameterIndex] += m_dp[m_currentParameterIndex];
    m_dp[m_currentParameterIndex] *= 0.9;
    setNextParameter();
  }

  m_currentStep               = 0;
  m_accumulatedError          = 0;
  m_isTuningIterationFinished = true;
}

void
PidController::setNextParameter()
{
  m_currentParameterIndex = (m_currentParameterIndex + 1) % m_ks.size();

  m_firstAdjustmentDone       = false;
  m_secondAdjustmentDone      = false;
}
