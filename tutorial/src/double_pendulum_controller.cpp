#include "mujoco_tutorial/double_pendulum_controller.hpp"

#include "mujoco_tutorial/common.hpp"

#include <algorithm>
#include <array>

namespace mujoco_tutorial {

bool UprightPdController::configure(const mjModel* model) {
    return model->nu >= 2;
}

void UprightPdController::activate() {
    target_ = {0.0, 0.0};
}

void UprightPdController::update(double, double, const JointState& state,
                                 JointCommand& command) {
    constexpr std::array<mjtNum, 2> kp{35.0, 18.0};
    constexpr std::array<mjtNum, 2> kd{4.5, 2.5};
    constexpr std::array<mjtNum, 2> limit{25.0, 15.0};

    for (int i = 0; i < 2; ++i) {
        const mjtNum raw = kp[i] * (target_[i] - state.position[i]) -
                           kd[i] * state.velocity[i];
        command.effort[i] = std::clamp(raw, -limit[i], limit[i]);
    }
}

MujocoJointSystem::MujocoJointSystem(const mjModel* model, mjData* data)
    : model_(model), data_(data) {
    joint_ids_[0] = require_id(model_, mjOBJ_JOINT, "joint1");
    joint_ids_[1] = require_id(model_, mjOBJ_JOINT, "joint2");
}

JointState MujocoJointSystem::read() const {
    JointState state;
    for (int i = 0; i < 2; ++i) {
        // qpos uses the joint position address; qvel uses the joint dof
        // address. For these hinge joints they map to angle and angular rate.
        state.position[i] = data_->qpos[model_->jnt_qposadr[joint_ids_[i]]];
        state.velocity[i] = data_->qvel[model_->jnt_dofadr[joint_ids_[i]]];
    }
    return state;
}

void MujocoJointSystem::write(const JointCommand& command) {
    for (int i = 0; i < 2; ++i) {
        // ctrl is indexed by actuator order. The XML defines one motor for
        // each joint, so command.effort[i] maps directly to ctrl[i].
        data_->ctrl[i] = command.effort[i];
    }
}

void reset_double_pendulum(const mjModel* model, mjData* data) {
    if (model->nkey > 0) {
        mj_resetDataKeyframe(model, data, 0);
    } else {
        data->qpos[0] = 0.12;
        data->qpos[1] = -0.08;
    }
    mj_forward(model, data);
}

JointState step_double_pendulum_controller(const mjModel* model, mjData* data,
                                           MujocoJointSystem& system,
                                           ControllerInterface& controller) {
    mj_step1(model, data);
    JointState state = system.read();
    JointCommand command;
    controller.update(data->time, model->opt.timestep, state, command);
    system.write(command);
    mj_step2(model, data);
    return system.read();
}

}  // namespace mujoco_tutorial
