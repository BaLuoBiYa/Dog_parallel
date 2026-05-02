// #include "dog_trajectory/jumpTrajectory.hpp"

// namespace Dog8Dof {
//     JumpTrajectory::JumpTrajectory(const std::string &configPath, rclcpp::Duration
//     period)
//         : BaseTrajectory(configPath, period) {
//         if (!reloadParameters()) {
//             throw std::invalid_argument("Could not load jump config YAML");
//         }
//     }

//     bool JumpTrajectory::reloadParameters() {
//         return false;
//     }

//     bool JumpTrajectory::setTarget(float jumpLength, float jumpHeight) {
//         jumpLength_ = jumpLength;
//         jumpHeight_ = jumpHeight;
//         return true;
//     }

//     bool JumpTrajectory::generate(const Body &currentTip) {
//         Trajectory traj;

//         // Trajectory crouchTraj = generateCrouching(currentTip);
//         // Trajectory takeoffTraj = generateTakeoff(jumpLength_ /
//         // parameters_.pushDuration,
//         //                                          jumpHeight_ /
//         //                                          parameters_.pushDuration);
//         // Trajectory flightTraj =
//         //     generateFlight(parameters_.pushDuration + parameters_.landDuration);
//         // Trajectory landTraj = generateLanding();

//         trajectory_ = std::move(traj);
//         return !trajectory_.points.empty();
//     }

//     bool JumpTrajectory::start(const rclcpp::Time &startTime) {
//         startTimeNanoSec_ = static_cast<size_t>(startTime.nanoseconds());
//         trajectory_.progress = 0;

//         if (trajectory_.points.empty()) {
//             return false;
//         }

//         for (size_t i = 0; i < trajectory_.time.size(); ++i) {
//             trajectory_.time[i] = startTimeNanoSec_ + i * controlDurationNanoSec_;
//         }

//         target_ = trajectory_.points.front();
//         return true;
//     }

//     TrajectoryPoint JumpTrajectory::get(const rclcpp::Time &currentTime) {
//         if (finished_) {
//             return target_;
//         }

//         if (trajectory_.points.empty()) {
//             finished_ = true;
//             return target_;
//         }

//         const size_t currentNanoSec = static_cast<size_t>(currentTime.nanoseconds());

//         if (currentNanoSec <= trajectory_.time.front()) {
//             target_ = trajectory_.points.front();
//             trajectory_.progress = 0;
//             return target_;
//         }

//         if (currentNanoSec >= trajectory_.time.back()) {
//             target_ = trajectory_.points.back();
//             trajectory_.progress = trajectory_.points.size();
//             finished_ = true;
//             return target_;
//         }

//         interpolate(currentNanoSec);

//         const size_t idx = std::min(trajectory_.progress, trajectory_.points.size() -
//         1); target_.kp = trajectory_.points[idx].kp; target_.kd =
//         trajectory_.points[idx].kd; return target_;
//     }
// } // namespace Dog8Dof

// namespace YAML {
//     template <> struct convert<Dog8Dof::JumpTrajectory::parameters_t> {
//         static Node encode(const Dog8Dof::JumpTrajectory::parameters_t &rhs) {
//             Node node;
//             node["crouchKp"] = rhs.crouchKp;
//             node["crouchKd"] = rhs.crouchKd;
//             node["crouchDuration"] = rhs.crouchDuration;
//             node["crouchDepth"] = rhs.crouchDepth;

//             node["pushDuration"] = rhs.pushDuration;

//             node["landDuration"] = rhs.landDuration;
//             return node;
//         }

//         static bool decode(const Node &node, Dog8Dof::JumpTrajectory::parameters_t
//         &rhs) {

//             rhs.crouchKp = node["crouchKp"].as<float>(1.0f);
//             rhs.crouchKd = node["crouchKd"].as<float>(0.05f);
//             rhs.crouchDuration = node["crouchDuration"].as<float>(0.3f);
//             rhs.crouchDepth = node["crouchDepth"].as<float>(50.0f);

//             rhs.pushDuration = node["pushDuration"].as<float>(0.1f);

//             rhs.landDuration = node["landDuration"].as<float>(0.25f);
//             return true;
//         }
//     };
// } // namespace YAML