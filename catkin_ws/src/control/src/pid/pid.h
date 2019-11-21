#include <algorithm>
#include <atomic>
#include <cmath>
#include <utility>

namespace pid {

template <typename T>
class Controller {
  public:
  const T max_windup;
  Controller(const T& Kp,
      const T& Ki,
      const T& Kd,
      const T& max_windup,
      const T& min_effort,
      const T& max_effort,
      const bool angular = false)
      : max_windup(max_windup)
      , _min_effort(min_effort)
      , _max_effort(max_effort)
      , _started(false)
      , _curr({ 0, 0 })
      , _prev({ 0, 0 })
      , _setpoint(0)
      , _Kp(Kp)
      , _Ki(Ki)
      , _Kd(Kd)
      , _angular(angular)
      , _bias((min_effort + max_effort) / 2.)
  {
  }

  ~Controller() {}

  inline T Kp() const
  {
    return _Kp;
  }

  inline T Ki() const
  {
    return _Ki;
  }

  inline T Kd() const
  {
    return _Kd;
  }

  inline T min_effort() const
  {
    return _min_effort;
  }

  inline T max_effort() const
  {
    return _max_effort;
  }

  inline T target() const
  {
    return _setpoint;
  }

  inline void set_Kp(const T& Kp)
  {
    _Kp = Kp;
  }

  inline void set_Kd(const T& Kd)
  {
    _Kd = Kd;
  }

  inline void set_Ki(const T& Ki)
  {
    _Ki = Ki;
  }

  inline void set_min_effort(const T& min_effort)
  {
    _min_effort = min_effort;
  }

  inline void set_max_effort(const T& max_effort)
  {
    _max_effort = max_effort;
  }

  inline void set_target(const T& setpoint)
  {
    _setpoint = setpoint;
  }

  void measure(const T& data, const double& stamp, const bool reset_integral = false)
  {
    float error = _setpoint.load() - data;
    if (_angular) {
      error = std::fmod(error + M_PI, 2 * M_PI) - (float)M_PI; // wrap to [-pi, pi]
    }

    _curr = { error, stamp };
    if (reset_integral) {
      _sum = 0;
    }
  }

  T control()
  {
    T u = 0;

    float& error = _curr.first;

    u += Kp() * error;

    if (!_started && _curr.second > 0) {
      _started = true;
    } else if (_started) {
      float de = _curr.first - _prev.first;
      float dt = std::max(0., _curr.second - _prev.second);

      _sum = std::clamp(_sum + dt * error, -max_windup, max_windup);
      u += _sum * Ki();

      if (dt > 0.)
        u += (de / dt) * Kd();
    }

    u = std::clamp(u + _bias, min_effort() , max_effort());
    _prev = _curr;

    return u;
  }

  private:
  Controller(Controller& controller);

  std::atomic<T> _Kp;
  std::atomic<T> _Ki;
  std::atomic<T> _Kd;
  std::atomic<T> _bias;

  std::atomic<T> _min_effort;
  std::atomic<T> _max_effort;

  std::pair<T, double> _curr;
  std::pair<T, double> _prev;
  std::atomic<T> _setpoint;

  T _sum;

  bool _started;
  bool _angular;
};

template <typename T>
Controller<T> make_controller(const T& Kp,
    const T& Ki,
    const T& Kd,
    const T& max_windup,
    const T& min_effort,
    const T& max_effort,
    const bool angular = false)
{
  return Controller<T>(Kp, Ki, Kd, max_windup, min_effort, max_effort, angular);
}
} // namespace pid
