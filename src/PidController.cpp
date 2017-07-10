/* INCLUDES ******************************************************************/

#include "PidController.h"

#include <iostream>

/* DEFINITIONS ***************************************************************/

static const double INITIAL_P_VALUE = 0.25;
static const double INITIAL_I_VALUE = 0.0025;
static const double INITIAL_D_VALUE = 10;
static const double TOLERANCE       = 0.2;
static const int    CACHE_LIMIT     = 50;

/* CLASS IMPLEMENTATION ******************************************************/

PidController::PidController()
: m_bestError(0)
, m_cumulativeError(0)
, m_proportionalError(0)
, m_integralError(0)
, m_derivativeError(0)
, m_isFirstRun(true)
, m_firstAdjusmentDone(false)
, m_secondAdjusmentDone(false)
, m_currentVariable(0)
{
  m_factors = std::vector<double>(3, 1);
  m_ks      = std::vector<double>(3, 0);
  m_ks[0]   = INITIAL_P_VALUE;
  m_ks[1]   = INITIAL_I_VALUE;
  m_ks[2]   = INITIAL_D_VALUE;
}

PidController::PidController(double Kp, double Ki, double Kd)
: m_bestError(0)
, m_cumulativeError(0)
, m_proportionalError(0)
, m_integralError(0)
, m_derivativeError(0)
, m_isFirstRun(true)
, m_firstAdjusmentDone(false)
, m_secondAdjusmentDone(false)
, m_currentVariable(0)
{
  m_factors = std::vector<double>(3, 1);
  m_ks      = std::vector<double>(3, 0);
  m_ks[0]   = INITIAL_P_VALUE;
  m_ks[1]   = INITIAL_I_VALUE;
  m_ks[2]   = INITIAL_D_VALUE;
}

PidController::~PidController()
{
}

void
PidController::updateError(double cte)
{
  m_derivativeError   = cte - m_proportionalError;
  m_proportionalError = cte;
  m_cumulativeError  += std::pow(cte, 2);

  //insertError(cte);

  m_integralError += cte;
  //m_integralError += getErrorCacheSum();
}

double
PidController::getTotalError()
{
  return - (m_ks[0] * m_proportionalError + m_ks[1] * m_integralError + m_ks[2] * m_derivativeError);
}

bool
PidController::twiddle()
{
  double factorsTotal = getFactorsTotal();

  if (factorsTotal < TOLERANCE)
  {
    std::cout << "FINISHED TWIDDLE - [" << m_ks[0] << "," << m_ks[1] << "," << m_ks[2] << "] ";
    std::cout << " CErr = " << m_cumulativeError;
    std::cout << " BErr = " << m_bestError;
    std::cout << " FA = " << m_firstAdjusmentDone;
    std::cout << " SA = " << m_secondAdjusmentDone;
    std::cout << " CV = " << m_currentVariable;
    std::cout << " FT = " << factorsTotal << std::endl;
    return false;
  }

  m_cumulativeError = m_cumulativeError / 1000.0;

  if (m_isFirstRun)
  {
    m_bestError  = m_cumulativeError;
    m_isFirstRun = false;
  }

  std::cout << "Twiddling - [" << m_ks[0] << "," << m_ks[1] << "," << m_ks[2] << "] ";
  std::cout << " CErr = " << m_cumulativeError;
  std::cout << " BErr = " << m_bestError;
  std::cout << " FA = " << m_firstAdjusmentDone;
  std::cout << " SA = " << m_secondAdjusmentDone;
  std::cout << " CV = " << m_currentVariable;
  std::cout << " FT = " << factorsTotal << std::endl;


  if (m_firstAdjusmentDone == false)
  {
    m_ks[m_currentVariable] += m_factors[m_currentVariable];
    m_firstAdjusmentDone     = true;
  }
  else if (m_cumulativeError < m_bestError)
  {
    m_factors[m_currentVariable] *= 1.1;
    m_bestError                   = m_cumulativeError;
    m_firstAdjusmentDone          = false;
    m_secondAdjusmentDone         = false;
    incrementCurrentVariableIndex();
  }
  else if (m_secondAdjusmentDone == false)
  {
    m_ks[m_currentVariable] -= 2 * m_factors[m_currentVariable];
    m_secondAdjusmentDone    = true;
  }
  else if (m_firstAdjusmentDone && m_secondAdjusmentDone)
  {
    m_ks[m_currentVariable]      += m_factors[m_currentVariable];
    m_factors[m_currentVariable] *= 0.9;
    m_firstAdjusmentDone          = false;
    m_secondAdjusmentDone         = false;
    incrementCurrentVariableIndex();
  }

  m_derivativeError   = 0;
  m_integralError     = 0;
  m_proportionalError = 0;
  m_cumulativeError   = 0;

  return true;
}

double
PidController::getFactorsTotal()
{
  double total = 0;

  for (size_t i = 0; i < m_factors.size(); ++i)
    total += m_factors[i];

  return total;
}

double
PidController::getErrorCacheSum()
{
  double sum = 0;

  for (size_t i = 0; i < m_errorCache.size(); ++i)
    sum += m_errorCache[i];

  return sum;
}

void
PidController::incrementCurrentVariableIndex()
{
  ++m_currentVariable;

  if (m_currentVariable > 2)
    m_currentVariable = 0;
}

void
PidController::insertError(double cte)
{
  if (m_errorCache.size() == CACHE_LIMIT)
    m_errorCache.pop_front();

  m_errorCache.push_back(cte);
}
