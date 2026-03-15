#ifndef DOG_CONTROLLER_FILTER_SMOOTHER_HPP_
#define DOG_CONTROLLER_FILTER_SMOOTHER_HPP_

namespace dog_controller
{
  namespace filter
  {

    /**
     * @class Smoother
     * @brief A simple second-order low-pass filter to generate smooth trajectories.
     *
     * This class simulates a critically damped spring-mass-damper system to
     * smoothly transition a value towards a target. It generates a trajectory
     * that is continuous in position, velocity, and acceleration.
     */
    class Smoother
    {
    public:
      Smoother() = default;

      /**
       * @brief Set the parameters of the smoother.
       * @param w Natural frequency (rad/s), determines the response speed.
       * @param d Damping ratio, determines how the system settles.
       */
      void set_params(float w, float d)
      {
        w_ = w;
        d_ = d;
      }

      /**
       * @brief Update the state of the smoother.
       * @param target The desired position.
       * @param dt The time step since the last update.
       * @return The new smoothed position.
       */
      float update(float target, float dt)
      {
        if (dt <= 0.0f)
        {
          return position_;
        }
        float error = target - position_;
        acceleration_ = w_ * w_ * error - 2 * d_ * w_ * velocity_;
        velocity_ += acceleration_ * dt;
        position_ += velocity_ * dt;
        return position_;
      }

      /**
       * @brief Update the state of the smoother with velocity feedforward.
       * @param target_pos The desired position.
       * @param target_vel The desired velocity.
       * @param dt The time step since the last update.
       * @return The new smoothed position.
       */
      float update_with_feedforward(float target_pos, float target_vel, float dt)
      {
        if (dt <= 0.0f)
        {
          return position_;
        }
        float pos_error = target_pos - position_;
        float vel_error = target_vel - velocity_;
        acceleration_ = w_ * w_ * pos_error + 2 * d_ * w_ * vel_error;
        velocity_ += acceleration_ * dt;
        position_ += velocity_ * dt;
        return position_;
      }

      /**
       * @brief Reset the state of the smoother.
       * @param new_pos The position to reset to.
       * @param new_vel The velocity to reset to (defaults to 0).
       */
      void reset(float new_pos, float new_vel = 0.0f)
      {
        position_ = new_pos;
        velocity_ = new_vel;
        acceleration_ = 0.0f;
      }

      /**
       * @brief Get the current smoothed position.
       */
      float get_position() const
      {
        return position_;
      }

      /**
       * @brief Get the current smoothed velocity.
       */
      float get_velocity() const
      {
        return velocity_;
      }

      /**
       * @brief Get the current smoothed acceleration.
       */
      float get_acceleration() const
      {
        return acceleration_;
      }

    private:
      float position_ = 0.0f;
      float velocity_ = 0.0f;
      float acceleration_ = 0.0f;

      // Default parameters, can be updated via set_params
      float w_ = 30.0f; // Natural frequency (rad/s)
      float d_ = 0.8f;  // Damping ratio
    };

  } // namespace filter
} // namespace dog_controller

#endif // DOG_CONTROLLER_FILTER_SMOOTHER_HPP_
