#include "mujoco_tutorial/common.hpp"
#include "mujoco_tutorial/double_pendulum_controller.hpp"

#include <iostream>

int main() {
    try {
        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("double_inverted_pendulum.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        mujoco_tutorial::reset_double_pendulum(model, data);

        mujoco_tutorial::MujocoJointSystem system(model, data);
        mujoco_tutorial::UprightPdController controller;
        if (!controller.configure(model)) {
            throw std::runtime_error("controller configure failed");
        }
        controller.activate();

        const double fixed_period = model->opt.timestep;
        const int steps = mujoco_tutorial::step_count_for_duration(model, 3.0);
        for (int step = 0; step < steps; ++step) {
            mujoco_tutorial::step_double_pendulum_controller(
                model, data, system, controller);
        }

        mj_forward(model, data);
        const mujoco_tutorial::JointState final_state = system.read();

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
