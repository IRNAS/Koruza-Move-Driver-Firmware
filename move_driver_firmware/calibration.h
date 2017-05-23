#ifndef calibration_h
#define calibration_h

#include "Arduino.h"
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "Wire.h"


enum class CalibrationState
{
  STANDBY,
  NEXT_POINT,
  RUN_MOTOR,
  READ_SENSOR,
  ERROR
};

extern String convertToString(const CalibrationState& state);

enum class CalibrationStatus
{
  OK,
  SENSOR_COMM_ERROR,
  SENSOR_BUSSY,
  LIMIT_SWITCH_PRESSED
};

extern String convertToString(const CalibrationStatus& status);

class Calibration
{
  private:
    Switch& m_limit_switch;
    TLV493D& m_sensor;
    AccelStepper& m_stepper;

    CalibrationState m_state;
    CalibrationStatus m_status;

    long m_N_cal_points;
    long m_current_point;
    double m_angle_meas_unwrap;
    long m_interval;
    double m_prev_value;

  public:
    // calibration parameters
    long m_start_step;
    static const long m_steps_per_revolution;
    double m_start_point;
    double m_end_point;

  public:
    Calibration(Switch& limit_switch, TLV493D& sensor, AccelStepper& stepper);
    bool start(const long N_points);
    void reset();
    uint8_t calculate_step(long& motor_step);
    void process();
    CalibrationState currentState();
    CalibrationStatus currentStatus();

  private:
    double phase_unwrap(const double angle_0_2pi, const double prev_value, long& interval);
    uint8_t wrap(long& value, const long min_value, const long max_value);
};

#endif
