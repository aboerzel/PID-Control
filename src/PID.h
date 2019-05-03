#ifndef PID_H
#define PID_H
#include <vector>
#include <limits>
#include <uWS/uWS.h>


class PID
{
public:

    /**
     * Constructor
     * 
     * \brief Initializes the PID controller
     * \param kp Proportional coefficient
     * \param ki Integral coefficient
     * \param kd Derivative coefficient
     */
    PID(double kp, double ki, double kd);

    /**
     * Destructor.
     */
    virtual ~PID();

    /**
     * \brief Enable twiddle for only one parameter
     * \param param Index of the parameter to be twiddled
     * \param twiddle_coefficient Start twiddle delta 
     * \param tolerance Twiddle goal
     * \param stabilization_steps Number of error updates before total error calculation is started
     * \param twiddle_steps Number of error updates between two twiddle steps
     */
    void twiddle_one_param(unsigned param, double twiddle_coefficient, double tolerance, int stabilization_steps, int twiddle_steps);

    /**
     * \brief Enable twiddle for all parameters
     * \param dp Start twiddle delta for kp
     * \param di Start twiddle delta for ki
     * \param dd Start twiddle delta for kd
     * \param tolerance Twiddle goal
     * \param stabilization_steps Number of error updates before total error calculation is started
     * \param twiddle_steps Number of error updates between two twiddle steps
     */
    void twiddle_all_params(double dp, double di, double dd, double tolerance, int stabilization_steps, int twiddle_steps);
    
    /**
    * \brief Update the error variables based on error value.
    * \param error Error value
    */ 
    void update_error(double error);

    /**
     * \brief Get control value
     * \return Control value
     */
    double control();

    /*
     * Set the server for restarting simulator.
     * @param ws The Websocket
     */
    void set_server(uWS::WebSocket<uWS::SERVER> ws);

private:

    /*
    * Restarts the simulator
    */
    void restart_simulator();

    /**
     * \Performs twiddle step
     */
    void twiddle();

    double p_error_ = 0;    // Proportional error
    double i_error_ = 0;    // Integral error
    double d_error_ = 0;    // Derivation error

    double total_error_ = 0; // Total error    

    std::vector<double> p_ = { 0, 0, 0 }; // Control coefficients

    int update_count_ = 0; // Number of error updates between twiddle updates


    bool twiddle_enabled_ = false; // Enable/disable twiddle

    bool twiddle_all_params_ = false; // Switch to train only on parameter or all parameters

    bool twiddle_finished_ = false; // Twiddle goal achieved

    unsigned current_param_ = 0; // Index of the parameter to be twiddled

    unsigned twiddle_step_ = 0; // Current twiddle step

    double tolerance_ = 0; // Twiddle goal

    int stabilization_steps_ = 1; // Number of error updates before total error calculation is started

    int twiddle_update_steps_ = 0; // Number of error updates between two twiddle steps

    double best_error_ = std::numeric_limits<double>::infinity(); // Best twiddled error so far

    std::vector<double> dp_ = { 0, 0, 0 }; // Twiddle coefficients

    std::vector<double> best_p_ = { 0, 0, 0 }; // Best twiddled control coefficients so far

    uWS::WebSocket<uWS::SERVER> server_{}; // Websocket
};

#endif  // PID_H
