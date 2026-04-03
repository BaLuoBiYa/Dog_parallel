#include "dog_controller/trajectoryTracker.hpp"

namespace Dog8Dof {
    controller_interface::CallbackReturn TrajectoryTracker::on_init() {
        // Initialize parameters
        robotName_ = auto_declare<std::string>("robotName", "legged_robot");
        standProfile_ = auto_declare<std::string>("standProfile", "");
        walkProfile_ = auto_declare<std::string>("walkProfile", "");
        jumpProfile_ = auto_declare<std::string>("jumpProfile", "");

        return controller_interface::CallbackReturn::SUCCESS;
    }

} // namespace Dog8Dof