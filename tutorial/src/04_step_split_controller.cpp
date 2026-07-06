#include "mujoco_tutorial/common.hpp"

#include <algorithm>
#include <iostream>

namespace {

mjtNum clamp(mjtNum value, mjtNum lower, mjtNum upper) {
    return std::min(std::max(value, lower), upper);
}

}  // namespace

int main() {
    try {
        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("hinge_arm.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        const int hinge_id = mujoco_tutorial::require_id(model, mjOBJ_JOINT, "hinge");
        const int tip_site = mujoco_tutorial::require_id(model, mjOBJ_SITE, "tip");
        const int qpos_address = model->jnt_qposadr[hinge_id];
        const int qvel_address = model->jnt_dofadr[hinge_id];

        data->qpos[qpos_address] = -0.7;
        mj_forward(model, data);

        const mjtNum target = 0.45;
        const mjtNum kp = 20.0;
        const mjtNum kd = 2.5;
        const int step_count = mujoco_tutorial::step_count_for_duration(model, 2.0);

        for (int step = 0; step < step_count; ++step) {
            mj_step1(model, data);

            const mjtNum position = data->qpos[qpos_address];
            const mjtNum velocity = data->qvel[qvel_address];
            data->ctrl[0] = clamp(kp * (target - position) - kd * velocity, -3.0, 3.0);

            mj_step2(model, data);
        }

        mj_forward(model, data);
        std::cout << "target=" << target << '\n';
        std::cout << "final_time=" << data->time << '\n';
        std::cout << "final_position=" << data->qpos[qpos_address] << '\n';
        mujoco_tutorial::print_vector("tip_xpos", data->site_xpos + 3 * tip_site, 3);
        mujoco_tutorial::print_sensor_table(model, data);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
