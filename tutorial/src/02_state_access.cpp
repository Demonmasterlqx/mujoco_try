#include "mujoco_tutorial/common.hpp"

#include <iostream>
#include <vector>

int main() {
    try {
        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("hinge_arm.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        data->qpos[0] = 0.35;
        data->qvel[0] = -0.15;
        data->ctrl[0] = 0.25;
        mj_forward(model, data);

        const int signature = mjSTATE_TIME | mjSTATE_QPOS | mjSTATE_QVEL | mjSTATE_CTRL;
        const int state_size = mj_stateSize(model, signature);
        std::vector<mjtNum> state(state_size);
        mj_getState(model, data, state.data(), signature);

        std::cout << "state_size=" << state_size << '\n';
        mujoco_tutorial::print_vector("saved state", state.data(), state_size);

        data->time = 12.0;
        data->qpos[0] = -0.75;
        data->qvel[0] = 1.0;
        data->ctrl[0] = -1.0;
        mj_forward(model, data);
        mujoco_tutorial::print_vector("mutated qpos", data->qpos, model->nq);

        mj_setState(model, data, state.data(), signature);
        mj_forward(model, data);

        std::cout << "restored time=" << data->time << '\n';
        mujoco_tutorial::print_vector("restored qpos", data->qpos, model->nq);
        mujoco_tutorial::print_vector("restored qvel", data->qvel, model->nv);
        mujoco_tutorial::print_vector("restored ctrl", data->ctrl, model->nu);
        mujoco_tutorial::print_sensor_table(model, data);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
