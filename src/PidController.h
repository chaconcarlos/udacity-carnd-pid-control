#ifndef PID_H
#define PID_H

/* INCLUDES ******************************************************************/

#include <cstdlib>
#include <cmath>
#include <deque>
#include <vector>

/* CLASS DECLARATION *********************************************************/

class PidController
{
  public:

    /**
     * @brief Initializes an instance of the PidController class.
     */
    PidController();

    /**
     * @brief Initializes an instance of the PidController class.
     *
     * @param Kp The initial coefficient for P.
     * @param Ki The initial coefficient for I.
     * @param Kd The initial coefficient for D
     */
    PidController(double Kp, double Ki, double Kd);

    /**
     * @brief Finalizes an instance of the PidController class.
     */
    virtual ~PidController();

  public:

    /**
     * @brief Updates the cross-track error.
     *
     * @param cte The new cross-track error.
     */
    void updateError(double cte);

    /**
     * @brief Gets the total error.
     *
     * @return The total error.
     */
    double getTotalError();

    /**
     * @brief Does the twiddle process to correct the parameters.
     *
     * @return true if the parameters need more adjusting.
     */
    bool twiddle();

  private:

    /**
     * @brief Gets the total of the factors.
     *
     * @return The total of the factors.
     */
    double getFactorsTotal();

    /**
     * @brief Gets the sum of the errors cache.
     *
     * @return The sum of the errors cache.
     */
    double getErrorCacheSum();

    /**
     * @brief Increments the current variable index.
     */
    void incrementCurrentVariableIndex();

    /**
     * @brief Inserts an error in the cache.
     *
     * @param cte The CTE to insert.
     */
    void insertError(double cte);

  private:

    double              m_bestError;
    double              m_cumulativeError;
    double              m_proportionalError;
    double              m_integralError;
    double              m_derivativeError;
    bool                m_isFirstRun;
    bool                m_firstAdjusmentDone;
    bool                m_secondAdjusmentDone;
    std::deque<double>  m_errorCache;
    size_t              m_currentVariable;
    std::vector<double> m_ks;
    std::vector<double> m_factors;
};

#endif /* PID_H */
