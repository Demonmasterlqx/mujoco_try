#pragma once

#include <mujoco/mujoco.h>

#include <array>

namespace mujoco_tutorial {

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
    bool configure(const mjModel* model) override;
    void activate() override;
    void update(double time, double period,
                const JointState& state, JointCommand& command) override;

private:
    std::array<mjtNum, 2> target_{};
};

class MujocoJointSystem {
public:
    MujocoJointSystem(const mjModel* model, mjData* data);

    JointState read() const;
    void write(const JointCommand& command);

private:
    const mjModel* model_;
    mjData* data_;
    std::array<int, 2> joint_ids_{};
};

void reset_double_pendulum(const mjModel* model, mjData* data);
JointState step_double_pendulum_controller(const mjModel* model, mjData* data,
                                           MujocoJointSystem& system,
                                           ControllerInterface& controller);

}  // namespace mujoco_tutorial
