#include "dog_trajectory/jumpTrajectoryLegacy.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using Dog8Dof::Body;
using Dog8Dof::JumpTrajectoryLegacy;
using Dog8Dof::TrajectoryPoint;

static void write_header(std::ofstream &csv) {
    csv << "t_s," << "pos_x,pos_y,pos_z," << "vel_x,vel_y,vel_z," << "tau_x,tau_y,tau_z,"
        << "kp_front,kp_rear," << "kd_front,kd_rear," << "finished\n";
}

static void
write_row(std::ofstream &csv, double t, const TrajectoryPoint &p, bool finished) {
    csv << std::fixed << std::setprecision(6) << t << "," << p.pos[0] << "," << p.pos[1]
        << "," << p.pos[2] << "," << p.vel[0] << "," << p.vel[1] << "," << p.vel[2] << ","
        << p.tau[0] << "," << p.tau[1] << "," << p.tau[2] << "," << p.kp[0] << ","
        << p.kp[1] << "," << p.kd[0] << "," << p.kd[1] << "," << (finished ? 1 : 0)
        << "\n";
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("dog_controller");
    const std::string yaml_path =
        (std::filesystem::path(share_dir) / "parameters" / "jumpLegacy.yaml").string();

    const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);
    JumpTrajectoryLegacy traj(yaml_path, period);

    Body current_tip{};
    current_tip.pos.setZero();
    current_tip.linerVel.setZero();
    current_tip.pos[0] = 0.0f;
    current_tip.pos[2] = 180.0f;

    std::string jumpConfig = "high_wall";

    if (!traj.setTarget(jumpConfig)) {
        std::cerr << "setTarget failed\n";
        return 1;
    }
    if (!traj.generate(current_tip)) {
        std::cerr << "generate failed\n";
        return 2;
    }
    if (!traj.start(rclcpp::Time(0, 0, RCL_SYSTEM_TIME))) {
        std::cerr << "start failed\n";
        return 3;
    }

    std::string csv_path;
    std::cout << "Type csv Path: ";
    std::cin >> csv_path;

    std::ofstream csv(csv_path, std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
        std::cerr << "cannot open csv file: " << csv_path << "\n";
        return 4;
    }

    write_header(csv);

    const int max_steps = 3000;
    const int64_t dt_ns = period.nanoseconds();

    for (int i = 0; i < max_steps; ++i) {
        const int64_t t_ns = static_cast<int64_t>(i) * dt_ns;
        const rclcpp::Time now(t_ns, RCL_SYSTEM_TIME);

        const TrajectoryPoint p = traj.get(now);
        const bool finished = traj.finished();
        write_row(csv, static_cast<double>(t_ns) * 1e-9, p, finished);

        if (finished) {
            break;
        }
    }

    csv.close();
    std::cout << "csv written to: " << csv_path << std::endl;

    rclcpp::shutdown();
    return 0;
}