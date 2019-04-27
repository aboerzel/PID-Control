#ifndef PID_H
#define PID_H
#include <vector>

class PID
{
public:
    /**
     * Constructor
     */
    PID();

    /**
     * Destructor.
     */
    virtual ~PID();

    /**
     * \brief Initialize PID controller
     * \param kp Proportional steering coefficient
     * \param ki Integral steering coefficient
     * \param kd Derivative steering coefficient
     * \param k_cte Cross-tracking coefficient for speed control
     * \param k_steer Steering coefficient for speed control
     * \param k_speed Speed coefficient for speed control
     * \param twiddle_steer Enable/disable twiddle for steering controller
     * \param twiddle_speed Enable/disable twiddle for speed controller
     */
    void init(double kp, double ki, double kd,
              double k_cte, double k_steer, double k_speed,
              bool twiddle_steer, bool twiddle_speed);

    /**
     * \brief Update the error variables based on cross-tracking error, speed and steering angle (CTE).
     * \param cte Current cross-tracking error
     * \param speed Current speed
     * \param angle Current steering angle
     */
    void update_error(double cte, double speed, double angle);

    /**
     * \brief Get steering control value
     * \return Steering control value between -1.0 and 1.0
     */
    double steer_control();

    /**
     * \brief Get speed control value
     * \return Speed control value
     */
    double speed_control();

    /**
     * \brief Optimizes the control coefficients using the Twiddle algorithm
     * \return Flag indicating whether the simulator should be reset
     */
    bool twiddle();

private:

    /**
     * \brief Calculate new control coefficients
     * \param step Last twiddle step
     * \param param Last twiddle parameter
     * \param err Current error
     * \param best_err Best error
     * \param p Current control coefficients
     * \param dp Current twiddle coefficients
     * \param best_p Best control coefficients
     */
    static void twiddle_update(unsigned& step, unsigned& param, double err, double& best_err,
                               std::vector<double>& p, std::vector<double>& dp, std::vector<double>& best_p);

    bool is_initialized_ = false; // Initialized flag

    /*
     * Errors
    */
    double p_error_{}; // Proportional error
    double i_error_{}; // Integral error
    double d_error_{}; // Derivative error

    double speed_error_{}; // Speed error
    double steer_error_{}; // Steer error

    /*
    * Coefficients
    */
    std::vector<double> steer_p_ = {0, 0, 0}; // Steering control coefficients
    std::vector<double> speed_p_ = {0, 0, 0}; // Speed control coefficients

    /*
    * Twiddle
    */
    std::vector<double> steer_dp_ = {1, 1, 1}; // twiddle coefficients for steering
    std::vector<double> speed_dp_ = {1, 1, 1}; // twiddle coefficients for speed

    bool twiddle_steer_ = false; // Flag to enable / disable twiddle for steering controller
    bool twiddle_speed_ = false; // Flag to enable / disable twiddle for speed controller
    bool twiddle_steer_finished_ = false; // Twiddle goal achieved for steering controller
    bool twiddle_speed_finished_ = false; // Twiddle goal achieved for steering controller

    double speed_err_sum_ = 0; // Total speed error
    double cte_sum_ = 0; // Total steer error     

    int twiddle_update_count_ = 0; // Number of error updates between twiddle updates

    unsigned twiddle_step_steer_ = 0; // Current twiddle step for steering
    unsigned twiddle_step_speed_ = 0; // Current twiddle step for speed

    unsigned twiddle_steer_param_ = 0; // Current twiddle parameter for steering
    unsigned twiddle_speed_param_ = 0; // Current twiddle parameter for speed

    std::vector<double> best_steer_p_ = {0, 0, 0}; // Best steering control coefficients so far
    std::vector<double> best_speed_p_ = {0, 0, 0}; // Best speed control coefficients so far

    double best_steer_err_ = std::numeric_limits<double>::infinity(); // Best steering error so far
    double best_speed_err_ = std::numeric_limits<double>::infinity(); // Best speed error so far

    double steer_tolerance_ = 0.0000001; // twiddle goal for steering
    double speed_tolerance_ = 0.0000001; // twiddle goal for speed
};

#endif  // PID_H
