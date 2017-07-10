#ifndef PID_H
#define PID_H

/* INCLUDES ******************************************************************/

#include <cstdlib>
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
     * @brief Enables parameter tuning.
     *
     * @param enable Inidicates if the parameter tuning should be enabled.
     */
    void enableParameterTuning(bool enable = true);

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
     * @brief Indicates if a tuning iteration has been finished.
     *
     * @return true if the tuning iteration is finished.
     */
    bool isTuningIterationFinished();

  private:

    /**
     * @brier Initializes the PID controller.
     */
    void initialize();

    /**
     * @brief Does an iteration of the process that tunes the parameters.
     */
    void tune();

    /**
     * @brief Does fine tuning for the current parameter.
     */
    void tuneCurrentParameter();

    /**
     * @brief Loads the index next parameter to be tuned.
     */
    void setNextParameter();

  private:

    bool                m_isEnabledParameterTuning;
    bool                m_firstAdjustmentDone;
    bool                m_secondAdjustmentDone;
    bool                m_isTuningIterationFinished;
    size_t              m_currentIteration;
    size_t              m_currentStep;
    size_t              m_currentParameterIndex;
    double              m_proportionalError;
    double              m_integralError;
    double              m_derivativeError;
    double              m_bestError;
    double              m_accumulatedError;
    std::vector<double> m_bestKs;
    std::vector<double> m_ks;
    std::vector<double> m_dp;
};

#endif /* PID_H */
