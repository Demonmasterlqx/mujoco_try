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
        const int qpos_address = model->jnt_qposadr[hinge_id];
        const int qvel_address = model->jnt_dofadr[hinge_id];

        std::cout<<"hinge_id = " << hinge_id << '\n';
        std::cout<<"qpos_address = " << qpos_address << '\n';
        std::cout<<"qvel_address = " << qvel_address << '\n';


        // qpos uses each joint's qpos address. For a hinge this value is the angle
        // in radians, so this seeds the arm away from the target.
        data->qpos[qpos_address] = 0.8;
        mj_forward(model, data);

        const mjtNum target = -0.4;
        const mjtNum kp = 18.0;
        const mjtNum kd = 2.0;
        const int step_count = mujoco_tutorial::step_count_for_duration(model, 2.0);

        for (int step = 0; step < step_count; ++step) {
            // qvel uses each joint's dof address. ctrl[0] is the first actuator's
            // command, here the torque command for the hinge motor.
            const mjtNum position = data->qpos[qpos_address];
            const mjtNum velocity = data->qvel[qvel_address];
            data->ctrl[hinge_id] = clamp(kp * (target - position) - kd * velocity, -3.0, 3.0);

            std::cout<< "step=" << step
                     << " time=" << data->time
                     << " position=" << position
                     << " velocity=" << velocity
                     << " torque=" << data->ctrl[hinge_id] << '\n';

            mj_step(model, data);
        }

        std::cout << "target=" << target << '\n';
        std::cout << "final_time=" << data->time << '\n';
        std::cout << "final_position=" << data->qpos[qpos_address] << '\n';
        std::cout << "final_velocity=" << data->qvel[qvel_address] << '\n';
        mujoco_tutorial::print_sensor_table(model, data);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
