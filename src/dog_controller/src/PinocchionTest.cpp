#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <iostream>
#include <iterator>
#include <pinocchio/algorithm/constraints/fwd.hpp>
#include <pinocchio/algorithm/contact-info.hpp>
#include <pinocchio/algorithm/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/fwd.hpp>

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    using namespace pinocchio;

    // You should change here to set up your own URDF file or just pass it as an argument of this
    // example.
    const std::string urdf_filename = "/home/luo/Project/Horse/Dog_parallel/src/dog_bringup/resource/model/model.urdf";

    // Load the urdf model
    Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    std::cout << "model name: " << model.name << std::endl;

    // Create data required by the algorithms
    Data data(model);

    // Sample a random configuration
    Eigen::VectorXd q = randomConfiguration(model);
    std::cout << "q: " << q.transpose() << std::endl;

    // RigidConstraintModel ci_RF(CONTACT_6D, model, RF_joint_id, LOCAL);
    // ci_RF.joint1_placement.setRandom(); // Offset in joint frame

    // Perform the forward kinematics over the kinematic tree
    forwardKinematics(model, data, q);

    // Print out the placement of each joint of the kinematic tree
    for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left << model.names[joint_id] << ": " << std::fixed << std::setprecision(2)
                  << data.oMi[joint_id].rotation().eulerAngles(2, 1, 0) << std::endl;
}