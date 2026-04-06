#include "dog_controller/leg.hpp"
#include "dog_tools/type.hpp"
#include <array>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace Dog8Dof;

namespace {
    std::array<float, 3> toArray3(const Eigen::Vector3f &v) {
        return {v[0], v[1], v[2]};
    }

    std::array<std::array<float, 3>, 3> toArray33(const Eigen::Matrix3f &m) {
        return {{{m(0, 0), m(0, 1), m(0, 2)},
                 {m(1, 0), m(1, 1), m(1, 2)},
                 {m(2, 0), m(2, 1), m(2, 2)}}};
    }

    Eigen::Vector3f toVec3(const std::array<float, 3> &a) {
        return Eigen::Vector3f(a[0], a[1], a[2]);
    }

    Eigen::Vector2f toVec2(const std::array<float, 2> &a) {
        return Eigen::Vector2f(a[0], a[1]);
    }

    Eigen::Matrix3f toMat33(const std::array<std::array<float, 3>, 3> &a) {
        Eigen::Matrix3f m;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                m(r, c) = a[r][c];
            }
        }
        return m;
    }
} // namespace

PYBIND11_MODULE(legPy, m) {
    py::enum_<Leg2Dof::MotorPlacement>(m, "MotorPlacement")
        .value("FRONT", Leg2Dof::MotorPlacement::FRONT)
        .value("REAR", Leg2Dof::MotorPlacement::REAR)
        .export_values();

    py::class_<Motor>(m, "Motor")
        .def(py::init<>())
        .def_readwrite("pos", &Motor::pos)
        .def_readwrite("vel", &Motor::vel)
        .def_readwrite("tau", &Motor::tau)
        .def_readwrite("kp", &Motor::kp)
        .def_readwrite("kd", &Motor::kd);

    py::class_<Body>(m, "Body")
        .def(py::init<const std::string &>(), py::arg("parent") = "base")
        .def_readwrite("parent", &Body::parent)
        .def_property(
            "pos",
            [](const Body &b) { return toArray3(b.pos); },
            [](Body &b, const std::array<float, 3> &v) { b.pos = toVec3(v); })
        .def_property(
            "rot",
            [](const Body &b) { return toArray33(b.rot); },
            [](Body &b, const std::array<std::array<float, 3>, 3> &m) {
                b.rot = toMat33(m);
            })
        .def_property(
            "linerVel",
            [](const Body &b) { return toArray3(b.linerVel); },
            [](Body &b, const std::array<float, 3> &v) { b.linerVel = toVec3(v); })
        .def_property(
            "angularVel",
            [](const Body &b) { return toArray3(b.angularVel); },
            [](Body &b, const std::array<float, 3> &v) { b.angularVel = toVec3(v); })
        .def_property(
            "tau",
            [](const Body &b) { return toArray3(b.tau); },
            [](Body &b, const std::array<float, 3> &v) { b.tau = toVec3(v); });

    py::class_<Leg2Dof>(m, "Leg2Dof")
        .def(py::init<float, float, float, float, float>(),
             py::arg("upperLength"),
             py::arg("lowerLength"),
             py::arg("tipLength"),
             py::arg("frontInitPos") = 0.0f,
             py::arg("rearInitPos") = 0.0f)
        .def("setTip",
             [](Leg2Dof &leg,
                const std::array<float, 3> &pos,
                const std::array<float, 3> &vel,
                const std::array<float, 3> &tau) {
                 return leg.setTip(toVec3(pos), toVec3(vel), toVec3(tau));
             })
        .def("updateMotor",
             [](Leg2Dof &leg,
                const std::array<float, 2> &pos,
                const std::array<float, 2> &vel,
                const std::array<float, 2> &tau) {
                 return leg.updateMotor(toVec2(pos), toVec2(vel), toVec2(tau));
             })
        .def("updateKp", &Leg2Dof::updateKp)
        .def("updateKd", &Leg2Dof::updateKd)
        .def("getMotorTarget", &Leg2Dof::getMotorTarget)
        .def("getTipCurrent", &Leg2Dof::getTipCurrent)
        .def("getTipTarget", &Leg2Dof::getTipTarget);
}