#include "mujoco_tutorial/common.hpp"

#include <algorithm>
#include <iostream>

namespace {

mjtNum clamp(mjtNum value, mjtNum lower, mjtNum upper) {
    return std::min(std::max(value, lower), upper);
}

void control_callback(const mjModel* model, mjData* data) {
    const int hinge_id = mj_name2id(model, mjOBJ_JOINT, "hinge");
    if (hinge_id < 0 || model->nu < 1) {
        return;
    }

    const int qpos_address = model->jnt_qposadr[hinge_id];
    const int qvel_address = model->jnt_dofadr[hinge_id];
    const mjtNum target = 0.25;
    // qpos/qvel read the hinge angle and angular velocity. ctrl[0] is the motor
    // command applied by MuJoCo during the control callback.
    data->ctrl[0] = clamp(16.0 * (target - data->qpos[qpos_address]) -
                              1.5 * data->qvel[qvel_address],
                          -3.0, 3.0);
}

class ScopedControlCallback {
public:
    explicit ScopedControlCallback(mjfGeneric callback)
        : previous_(mjcb_control) {
        mjcb_control = callback;
    }

    ~ScopedControlCallback() {
        mjcb_control = previous_;
    }

private:
    mjfGeneric previous_;
};

}  // namespace

int main() {
    try {
        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("hinge_arm.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        // This model has one hinge dof, so qpos[0] seeds its initial angle.
        data->qpos[0] = -0.9;
        mj_forward(model, data);

        const mjfGeneric previous_callback = mjcb_control;
        {
            const ScopedControlCallback callback_guard(control_callback);
            mujoco_tutorial::step_for(model, data, 2.0);
        }

        std::cout << "callback_restored="
                  << (mjcb_control == previous_callback ? "true" : "false") << '\n';
        std::cout << "final_time=" << data->time << '\n';
        // Print generalized position, generalized velocity, and actuator command
        // after the callback-driven rollout.
        mujoco_tutorial::print_vector("qpos", data->qpos, model->nq);
        mujoco_tutorial::print_vector("qvel", data->qvel, model->nv);
        mujoco_tutorial::print_vector("ctrl", data->ctrl, model->nu);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
