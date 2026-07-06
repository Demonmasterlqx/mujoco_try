#include "mujoco_tutorial/common.hpp"

#include <iostream>
#include <vector>

int main() {
    try {
        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("hinge_arm.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        const int tip_site = mujoco_tutorial::require_id(model, mjOBJ_SITE, "tip");
        data->qpos[0] = 0.4;
        data->qvel[0] = 0.2;
        mj_forward(model, data);

        std::vector<mjtNum> mass_matrix(model->nv * model->nv);
        mj_fullM(model, data, mass_matrix.data());
        mujoco_tutorial::print_vector("dense mass matrix", mass_matrix.data(),
                                      static_cast<int>(mass_matrix.size()));

        std::vector<mjtNum> jacobian_position(3 * model->nv);
        std::vector<mjtNum> jacobian_rotation(3 * model->nv);
        mj_jacSite(model, data, jacobian_position.data(), jacobian_rotation.data(), tip_site);
        mujoco_tutorial::print_vector("tip position Jacobian", jacobian_position.data(),
                                      static_cast<int>(jacobian_position.size()));
        mujoco_tutorial::print_vector("tip rotation Jacobian", jacobian_rotation.data(),
                                      static_cast<int>(jacobian_rotation.size()));

        data->qacc[0] = -0.1;
        mj_inverse(model, data);
        mujoco_tutorial::print_vector("inverse dynamics qfrc", data->qfrc_inverse, model->nv);

        mj_energyPos(model, data);
        mj_energyVel(model, data);
        std::cout << "potential_energy=" << data->energy[0] << '\n';
        std::cout << "kinetic_energy=" << data->energy[1] << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
