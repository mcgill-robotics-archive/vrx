#include <algorithm>
#include <atomic>
#include <cmath>
#include <utility>

namespace pid {

template <typename T>
class Controller {
  public:
  const T max_windup;
  const T min_effort;
  const T max_effort;

  Controller(const T& Kp,
      const T& Ki,
      const T& Kd,
      const T& max_windup,
      const T& min_effort,
      const T& max_effort,
      const bool angular = false)
      : max_windup(max_windup)
      , min_effort(min_effort)
      , max_effort(max_effort)
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

  inline T& Kp() const
  {
    return _Kp;
  }

  inline T& Ki() const
  {
    return _Ki;
  }

  inline T& Kd() const
  {
    return _Kd;
  }

  inline T& target() const
  {
    return _setpoint;
  }

  inline void reconfigure(const T& Kp, const T& Ki, const T& Kd)
  {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
  }

  inline void set_target(const T& setpoint)
  {
    _setpoint = setpoint;
  }

  void measure(const T& data, const double& stamp, const bool reset_integral = false)
  {
    auto error = data - _setpoint.load();
    if (_angular) {
      error = (error + M_PI) % (2 * M_PI) - M_PI; // wrap to [-pi, pi]
    }

    _curr = { error, stamp };
    if (reset_integral) {
      _sum = 0;
    }
  }

  T control()
  {
    T u = 0;

    auto& error = _curr.first;

    u += _Kp * error;

    if (!_started && _curr.second > 0) {
      _started = true;
    } else if (_started) {
      auto de = _curr.first - _prev.first;
      auto dt = _curr.second - _prev.second;

      _sum = std::clamp(_sum + dt * error, -max_windup, max_windup);
      u += _sum * _Ki;
      u += (de / dt) * _Kd;
    }

    u = std::clamp(u + _bias, min_effort, max_effort);
    _prev = _curr;

    return u;
  }

  private:
  Controller(Controller& controller);

  T _Kp;
  T _Ki;
  T _Kd;
  T _bias;

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
