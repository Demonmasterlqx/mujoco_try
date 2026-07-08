#include "mujoco_tutorial/common.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <string>

namespace {

struct JointState {
    std::array<mjtNum, 2> position{};
    std::array<mjtNum, 2> velocity{};
};

struct JointCommand {
    std::array<mjtNum, 2> effort{};
};

class ControllerInterface {
public:
    virtual ~ControllerInterface() = default;
    virtual bool configure(const mjModel* model) = 0;
    virtual void activate() = 0;
    virtual void update(double time, double period,
                        const JointState& state, JointCommand& command) = 0;
};

class UprightPdController final : public ControllerInterface {
public:
    bool configure(const mjModel* model) override {
        return model->nu >= 2;
    }

    void activate() override {
        target_ = {0.0, 0.0};
    }

    void update(double, double, const JointState& state, JointCommand& command) override {
        constexpr std::array<mjtNum, 2> kp{35.0, 18.0};
        constexpr std::array<mjtNum, 2> kd{4.5, 2.5};
        constexpr std::array<mjtNum, 2> limit{25.0, 15.0};

        for (int i = 0; i < 2; ++i) {
            const mjtNum raw = kp[i] * (target_[i] - state.position[i]) -
                               kd[i] * state.velocity[i];
            command.effort[i] = std::clamp(raw, -limit[i], limit[i]);
        }
    }

private:
    std::array<mjtNum, 2> target_{};
};

class MujocoJointSystem {
public:
    MujocoJointSystem(const mjModel* model, mjData* data)
        : model_(model), data_(data) {
        joint_ids_[0] = mujoco_tutorial::require_id(model_, mjOBJ_JOINT, "joint1");
        joint_ids_[1] = mujoco_tutorial::require_id(model_, mjOBJ_JOINT, "joint2");
    }

    JointState read() const {
        JointState state;
        for (int i = 0; i < 2; ++i) {
            // qpos uses the joint position address; qvel uses the joint dof
            // address. For these hinge joints they map to angle and angular rate.
            state.position[i] = data_->qpos[model_->jnt_qposadr[joint_ids_[i]]];
            state.velocity[i] = data_->qvel[model_->jnt_dofadr[joint_ids_[i]]];
        }
        return state;
    }

    void write(const JointCommand& command) {
        for (int i = 0; i < 2; ++i) {
            // ctrl is indexed by actuator order. The XML defines one motor for
            // each joint, so command.effort[i] maps directly to ctrl[i].
            data_->ctrl[i] = command.effort[i];
        }
    }

private:
    const mjModel* model_;
    mjData* data_;
    std::array<int, 2> joint_ids_{};
};

}  // namespace

int main() {
    try {
        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("double_inverted_pendulum.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        if (model->nkey > 0) {
            mj_resetDataKeyframe(model, data, 0);
        } else {
            // Fallback seed: qpos[0] and qpos[1] are the two hinge angles.
            data->qpos[0] = 0.12;
            data->qpos[1] = -0.08;
        }
        mj_forward(model, data);

        MujocoJointSystem system(model, data);
        UprightPdController controller;
        if (!controller.configure(model)) {
            throw std::runtime_error("controller configure failed");
        }
        controller.activate();

        const double fixed_period = model->opt.timestep;
        const int steps = mujoco_tutorial::step_count_for_duration(model, 3.0);
        for (int step = 0; step < steps; ++step) {
            mj_step1(model, data);
            JointState state = system.read();
            JointCommand command;
            controller.update(data->time, fixed_period, state, command);
            system.write(command);
            mj_step2(model, data);
        }

        mj_forward(model, data);
        const JointState final_state = system.read();

        std::cout << "steps=" << steps << '\n';
        std::cout << "fixed_period=" << fixed_period << '\n';
        std::cout << "final_time=" << data->time << '\n';
        std::cout << "joint1_position=" << final_state.position[0] << '\n';
        std::cout << "joint2_position=" << final_state.position[1] << '\n';
        std::cout << "joint1_velocity=" << final_state.velocity[0] << '\n';
        std::cout << "joint2_velocity=" << final_state.velocity[1] << '\n';
        // Print the final actuator command vector.
        mujoco_tutorial::print_vector("ctrl", data->ctrl, model->nu);
        mujoco_tutorial::print_sensor_table(model, data);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
