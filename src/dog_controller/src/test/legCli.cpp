#include "dog_controller/leg.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

using namespace Dog8Dof;

namespace {

    enum class SolveMode {
        IK,
        FK,
    };

    void printUsage(const char *prog) {
        std::cout
            << "Usage:\n"
            << "  " << prog << " --mode <ik|fk> [common options] [mode options]\\n\\n"
            << "Common options:\n"
            << "  [--upper <mm>] [--lower <mm>] [--tip <mm>]\\n\\n"
            << "IK mode options (tip -> motor):\n"
            << "  --x <mm> --z <mm> [--vx <mm/s>] [--vz <mm/s>] [--fx <N>] [--fz <N>]\\n"
            << "  [--front-kp <val>] [--front-kd <val>] [--rear-kp <val>] [--rear-kd "
               "<val>]\n\n"
            << "FK mode options (motor -> tip):\n"
            << "  --front-pos <rad> --rear-pos <rad> [--front-vel <rad/s>] [--rear-vel "
               "<rad/s>]\\n"
            << "  [--front-tau <Nm>] [--rear-tau <Nm>]\n\n"
            << "Defaults:\n"
            << "  --mode ik, --upper 100, --lower 100, --tip 10\n"
            << "  IK: --vx 0 --vz 0 --fx 0 --fz 0 --front-kp 0 --front-kd 0 --rear-kp 0 "
               "--rear-kd 0\n"
            << "  FK: --front-vel 0 --rear-vel 0 --front-tau 0 --rear-tau 0\n\n"
            << "Examples:\n"
            << "  " << prog
            << " --mode ik --x 20 --z 180 --vx 5 --vz -2 --fx 10 --fz 80 --front-kp 20 "
               "--rear-kp 20\n"
            << "  " << prog
            << " --mode fk --front-pos 1.02 --rear-pos 1.24 --front-vel -0.04 --rear-vel "
               "0.01 --front-tau 3.3 --rear-tau 3.5\n";
    }

    bool parseModeArg(const std::map<std::string, std::string> &args, SolveMode &mode) {
        const auto it = args.find("mode");
        if (it == args.end()) {
            mode = SolveMode::IK;
            return true;
        }

        if (it->second == "ik" || it->second == "IK") {
            mode = SolveMode::IK;
            return true;
        }
        if (it->second == "fk" || it->second == "FK") {
            mode = SolveMode::FK;
            return true;
        }

        std::cerr << "Invalid --mode value: " << it->second << ", expected ik or fk\n";
        return false;
    }

    bool parseFloatArg(const std::map<std::string, std::string> &args,
                       const std::string &name,
                       float &value,
                       bool required,
                       float defaultValue = 0.0f) {
        const auto it = args.find(name);
        if (it == args.end()) {
            if (required) {
                std::cerr << "Missing required argument: --" << name << "\n";
                return false;
            }
            value = defaultValue;
            return true;
        }

        char *end = nullptr;
        const float v = std::strtof(it->second.c_str(), &end);
        if (end == it->second.c_str() || *end != '\0' || !std::isfinite(v)) {
            std::cerr << "Invalid float value for --" << name << ": " << it->second
                      << "\n";
            return false;
        }

        value = v;
        return true;
    }

    bool promptFloat(const std::string &label, float &value, float defaultValue) {
        std::cout << label << " [default " << defaultValue << "]: ";
        std::string line;
        if (!std::getline(std::cin, line)) {
            return false;
        }
        if (line.empty()) {
            value = defaultValue;
            return true;
        }

        char *end = nullptr;
        const float v = std::strtof(line.c_str(), &end);
        if (end == line.c_str() || *end != '\0' || !std::isfinite(v)) {
            std::cerr << "Invalid number: " << line << "\n";
            return false;
        }
        value = v;
        return true;
    }

    bool promptMode(SolveMode &mode) {
        std::cout << "mode [ik/fk, default ik]: ";
        std::string line;
        if (!std::getline(std::cin, line)) {
            return false;
        }
        if (line.empty() || line == "ik" || line == "IK") {
            mode = SolveMode::IK;
            return true;
        }
        if (line == "fk" || line == "FK") {
            mode = SolveMode::FK;
            return true;
        }
        std::cerr << "Invalid mode: " << line << "\n";
        return false;
    }

} // namespace

int main(int argc, char **argv) {
    std::map<std::string, std::string> args;

    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--ros-args") {
            // Ignore ROS-specific trailing arguments when launched by ros2 run.
            break;
        }
        if (key == "--help" || key == "-h") {
            printUsage(argv[0]);
            return 0;
        }
        if (key.rfind("__", 0) == 0) {
            continue;
        }
        if (key.rfind("--", 0) != 0) {
            std::cerr << "Unknown argument format: " << key << "\n";
            printUsage(argv[0]);
            return 2;
        }
        if (i + 1 >= argc) {
            std::cerr << "Missing value for " << key << "\n";
            printUsage(argv[0]);
            return 2;
        }
        args[key.substr(2)] = argv[++i];
    }

    float x = 0.0f;
    float z = 0.0f;
    float vx = 0.0f;
    float vz = 0.0f;
    float fx = 0.0f;
    float fz = 0.0f;
    float upper = 100.0f;
    float lower = 100.0f;
    float tip = 10.0f;
    float frontKp = 0.0f;
    float frontKd = 0.0f;
    float rearKp = 0.0f;
    float rearKd = 0.0f;
    float frontPos = 0.0f;
    float rearPos = 0.0f;
    float frontVel = 0.0f;
    float rearVel = 0.0f;
    float frontTau = 0.0f;
    float rearTau = 0.0f;
    SolveMode mode = SolveMode::IK;

    if (argc == 1) {
        std::cout << "Interactive mode: press Enter to use defaults.\n";
        if (!promptMode(mode) || !promptFloat("upper length (mm)", upper, 100.0f) ||
            !promptFloat("lower length (mm)", lower, 100.0f) ||
            !promptFloat("tip length (mm)", tip, 10.0f)) {
            return 2;
        }

        if (mode == SolveMode::IK) {
            if (!promptFloat("x (mm)", x, 0.0f) || !promptFloat("z (mm)", z, 180.0f) ||
                !promptFloat("vx (mm/s)", vx, 0.0f) ||
                !promptFloat("vz (mm/s)", vz, 0.0f) || !promptFloat("fx (N)", fx, 0.0f) ||
                !promptFloat("fz (N)", fz, 0.0f) ||
                !promptFloat("front kp", frontKp, 0.0f) ||
                !promptFloat("front kd", frontKd, 0.0f) ||
                !promptFloat("rear kp", rearKp, 0.0f) ||
                !promptFloat("rear kd", rearKd, 0.0f)) {
                return 2;
            }
        } else {
            if (!promptFloat("front pos (rad)", frontPos, 0.0f) ||
                !promptFloat("rear pos (rad)", rearPos, 0.0f) ||
                !promptFloat("front vel (rad/s)", frontVel, 0.0f) ||
                !promptFloat("rear vel (rad/s)", rearVel, 0.0f) ||
                !promptFloat("front tau (Nm)", frontTau, 0.0f) ||
                !promptFloat("rear tau (Nm)", rearTau, 0.0f)) {
                return 2;
            }
        }
    } else {
        if (!parseModeArg(args, mode) ||
            !parseFloatArg(args, "upper", upper, false, 100.0f) ||
            !parseFloatArg(args, "lower", lower, false, 100.0f) ||
            !parseFloatArg(args, "tip", tip, false, 10.0f) ||
            !parseFloatArg(args, "front-kp", frontKp, false, 0.0f) ||
            !parseFloatArg(args, "front-kd", frontKd, false, 0.0f) ||
            !parseFloatArg(args, "rear-kp", rearKp, false, 0.0f) ||
            !parseFloatArg(args, "rear-kd", rearKd, false, 0.0f)) {
            return 2;
        }

        if (mode == SolveMode::IK) {
            if (!parseFloatArg(args, "x", x, true) ||
                !parseFloatArg(args, "z", z, true) ||
                !parseFloatArg(args, "vx", vx, false, 0.0f) ||
                !parseFloatArg(args, "vz", vz, false, 0.0f) ||
                !parseFloatArg(args, "fx", fx, false, 0.0f) ||
                !parseFloatArg(args, "fz", fz, false, 0.0f)) {
                return 2;
            }
        } else {
            if (!parseFloatArg(args, "front-pos", frontPos, true) ||
                !parseFloatArg(args, "rear-pos", rearPos, true) ||
                !parseFloatArg(args, "front-vel", frontVel, false, 0.0f) ||
                !parseFloatArg(args, "rear-vel", rearVel, false, 0.0f) ||
                !parseFloatArg(args, "front-tau", frontTau, false, 0.0f) ||
                !parseFloatArg(args, "rear-tau", rearTau, false, 0.0f)) {
                return 2;
            }
        }
    }

    if (upper <= 0.0f || lower <= 0.0f || tip < 0.0f) {
        std::cerr << "Link lengths must satisfy: upper > 0, lower > 0, tip >= 0\n";
        return 2;
    }

    Leg2Dof leg(upper, lower, tip, 0.0f, 0.0f);

    if (!leg.updateKp(Leg2Dof::FRONT, frontKp) ||
        !leg.updateKd(Leg2Dof::FRONT, frontKd) || !leg.updateKp(Leg2Dof::REAR, rearKp) ||
        !leg.updateKd(Leg2Dof::REAR, rearKd)) {
        std::cerr << "kp/kd out of valid range [0, 100]\n";
        return 2;
    }

    std::cout << std::fixed << std::setprecision(6);

    if (mode == SolveMode::IK) {
        const Eigen::Vector3f pos(x, 0.0f, z);
        const Eigen::Vector3f vel(vx, 0.0f, vz);
        const Eigen::Vector3f force(fx, 0.0f, fz);

        if (!leg.setTip(pos, vel, force)) {
            std::cerr << "IK failed: input tip state may be unreachable or singular\n";
            return 1;
        }

        const Motor front = leg.getMotorTarget(Leg2Dof::FRONT);
        const Motor rear = leg.getMotorTarget(Leg2Dof::REAR);

        std::cout << "Mode: IK (tip -> motor)\n";
        std::cout << "Input Tip:\n";
        std::cout << "  pos(mm): x=" << x << " z=" << z << "\n";
        std::cout << "  vel(mm/s): vx=" << vx << " vz=" << vz << "\n";
        std::cout << "  force(N): fx=" << fx << " fz=" << fz << "\n";

        std::cout << "\nSolved Motor Targets:\n";
        std::cout << "  FRONT: pos(rad)=" << front.pos << " vel(rad/s)=" << front.vel
                  << " tau(Nm)=" << front.tau << " kp=" << front.kp << " kd=" << front.kd
                  << "\n";
        std::cout << "  REAR : pos(rad)=" << rear.pos << " vel(rad/s)=" << rear.vel
                  << " tau(Nm)=" << rear.tau << " kp=" << rear.kp << " kd=" << rear.kd
                  << "\n";

        // Forward check: project solved motor targets back to tip state.
        const Eigen::Vector2f motorPos(front.pos, rear.pos);
        const Eigen::Vector2f motorVel(front.vel, rear.vel);
        const Eigen::Vector2f motorTau(front.tau, rear.tau);

        if (leg.updateMotor(motorPos, motorVel, motorTau)) {
            const Body tipNow = leg.getTipCurrent();
            std::cout << "\nForward Check (motor -> tip):\n";
            std::cout << "  pos(mm): x=" << tipNow.pos[0] << " z=" << tipNow.pos[2]
                      << "\n";
            std::cout << "  vel(mm/s): vx=" << tipNow.linerVel[0]
                      << " vz=" << tipNow.linerVel[2] << "\n";
            std::cout << "  force(N): fx=" << tipNow.tau[0] << " fz=" << tipNow.tau[2]
                      << "\n";
        } else {
            std::cout << "\nForward Check skipped: solver singular during updateMotor\n";
        }
    } else {
        const Eigen::Vector2f motorPos(frontPos, rearPos);
        const Eigen::Vector2f motorVel(frontVel, rearVel);
        const Eigen::Vector2f motorTau(frontTau, rearTau);

        if (!leg.updateMotor(motorPos, motorVel, motorTau)) {
            std::cerr << "FK failed: motor state may be singular or unreachable\n";
            return 1;
        }

        const Body tipNow = leg.getTipCurrent();
        std::cout << "Mode: FK (motor -> tip)\n";
        std::cout << "Input Motors:\n";
        std::cout << "  FRONT: pos(rad)=" << frontPos << " vel(rad/s)=" << frontVel
                  << " tau(Nm)=" << frontTau << "\n";
        std::cout << "  REAR : pos(rad)=" << rearPos << " vel(rad/s)=" << rearVel
                  << " tau(Nm)=" << rearTau << "\n";

        std::cout << "\nSolved Tip State:\n";
        std::cout << "  pos(mm): x=" << tipNow.pos[0] << " z=" << tipNow.pos[2] << "\n";
        std::cout << "  vel(mm/s): vx=" << tipNow.linerVel[0]
                  << " vz=" << tipNow.linerVel[2] << "\n";
        std::cout << "  force(N): fx=" << tipNow.tau[0] << " fz=" << tipNow.tau[2]
                  << "\n";
    }

    return 0;
}
