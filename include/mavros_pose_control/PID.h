#ifndef PID_H_
#define PID_H_

#include <mutex>

class PIDController {
 public:
  PIDController(double kp, double ki, double kd, double start_time, double max, double min) {
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
    max_ = max;
    min_ = min;
    prev_time_ = start_time;
  }

  void setGains( double kp, double ki, double kd ) {
    gains_mtx_.lock();
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
    gains_mtx_.unlock();
  }

  void setSetPoint( double sp ) {
    set_point_mtx_.lock();
    set_point_ = sp;
    set_point_mtx_.unlock();
  }

  void resetIntegral() {
      gains_mtx_.lock();
      sum_error_ = 0.0;
      gains_mtx_.unlock();
  }

  double process( double cur_time, double present_value ) {
    gains_mtx_.lock();
    set_point_mtx_.lock();
    // deal with time pieces
    cur_time_ = cur_time;
    double dt = cur_time_ - prev_time_; // only dt is needed for rest of calculations
    prev_time_ = cur_time_;

    // get the current error
    cur_error_ = set_point_ - present_value; // p

    sum_error_ += cur_error_ * dt; // i

    if ( sum_error_ > max_ )
       sum_error_ = max_;
    else if ( sum_error_ < min_ )
       sum_error_ = min_;


    deriv_error_ = (cur_error_ - prev_error_)/dt; // d

    prev_error_ = cur_error_; // store 
    output_ = (kp_*cur_error_) + (kd_*deriv_error_) + (ki_*sum_error_);

    if ( output_ > max_ )
       output_ = max_;
    else if ( output_ < min_ )
       output_ = min_;

    gains_mtx_.unlock();
    set_point_mtx_.unlock();

    return output_;
  }

  double getKp(){ return kp_; }
  double getKi(){ return ki_; }
  double getKd(){ return kd_; }
  double getCE(){ return cur_error_; }
  double getDE(){ return deriv_error_; }
  double getSE(){ return sum_error_; }


  double getOutput() {
    return output_;
  }

 private:
  // gains
  std::mutex gains_mtx_;
  double kp_, kd_, ki_;

  // set point
  std::mutex set_point_mtx_;
  double set_point_;

  // times
  double prev_time_, cur_time_;

  // errors
  double prev_error_, cur_error_, sum_error_, deriv_error_;

  // clamping
  double max_, min_;

  // results
  double output_;

};

#endif
