#include "mujoco_tutorial/common.hpp"

#include <iostream>

int main() {
    try {
        const std::string model_file =
            mujoco_tutorial::rm26_path("urdf/rm26_version3_engineering_model_mujoco_split.urdf");
        auto simulation = mujoco_tutorial::load_simulation(model_file);
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        mj_forward(model, data);
        mujoco_tutorial::print_model_summary(model);
        std::cout << "nbody=" << model->nbody
                  << " njnt=" << model->njnt
                  << " ngeom=" << model->ngeom
                  << " nmesh=" << model->nmesh << '\n';
        std::cout << "first_body=" << mujoco_tutorial::object_name(model, mjOBJ_BODY, 1) << '\n';
        std::cout << "time=" << data->time << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
