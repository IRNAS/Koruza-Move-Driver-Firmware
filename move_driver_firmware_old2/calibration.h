#ifndef calibration_h
#define calibration_h

class Calibration
{
  private:
    Switch& m_limit_switch;
    TLV493D& m_sensor;
    AccelStepper& m_stepper;
    
  public:
    // calibration parameters
    long m_start_step;
    static const long m_steps_per_revolution;
    double m_start_point;
    double m_end_point;
    
  public:
    Calibration(Switch& limit_switch, TLV493D& sensor, AccelStepper& stepper);
    uint8_t calibrate(const long N_points);
    uint8_t calculate_step(const double sensor_reading, long& motor_step);

  private:
    double phase_unwrap(const double angle_0_2pi, const double prev_value, long& interval);
    uint8_t wrap(long& value, const long min_value, const long max_value);
};

#endif
