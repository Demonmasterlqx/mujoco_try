#include "mujoco_tutorial/common.hpp"

#include <iostream>

int main() {
    try {
        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("free_box.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        mujoco_tutorial::print_model_summary(model);
        mujoco_tutorial::print_vector("initial qpos", data->qpos, model->nq);

        mujoco_tutorial::step_for(model, data, 1.0);

        std::cout << "time=" << data->time << '\n';
        mujoco_tutorial::print_vector("final qpos", data->qpos, model->nq);
        mujoco_tutorial::print_vector("final qvel", data->qvel, model->nv);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
