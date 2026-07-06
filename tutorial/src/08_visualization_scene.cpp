#include "mujoco_tutorial/common.hpp"

#include <iostream>

int main() {
    try {
        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("hinge_arm.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        mjvCamera camera;
        mjvOption option;
        mjvScene scene;
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultScene(&scene);
        mjv_makeScene(model, &scene, 1000);

        data->qpos[0] = 0.25;
        mj_forward(model, data);
        mjv_updateScene(model, data, &option, nullptr, &camera, mjCAT_ALL, &scene);

        std::cout << "scene_ngeom=" << scene.ngeom << '\n';
        std::cout << "scene_nlight=" << scene.nlight << '\n';
        if (scene.ngeom > 0) {
            std::cout << "first_geom_type=" << scene.geoms[0].type << '\n';
        }

        mjv_freeScene(&scene);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
