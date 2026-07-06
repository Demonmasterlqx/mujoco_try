#include "mujoco_tutorial/common.hpp"

#include <mujoco/mjplugin.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

std::string join_path(const std::string& root, const std::string& child) {
    if (root.empty()) {
        return child;
    }
    if (root.back() == '/') {
        return root + child;
    }
    return root + "/" + child;
}

void plugin_load_callback(const char* filename, int first, int count) {
    std::cout << "plugin_library=" << filename
              << " first=" << first
              << " count=" << count << '\n';
    for (int slot = first; slot < first + count; ++slot) {
        const mjpPlugin* plugin = mjp_getPluginAtSlot(slot);
        if (plugin != nullptr) {
            std::cout << "  plugin=" << plugin->name << '\n';
        }
    }
}

}  // namespace

int main() {
    try {
        const std::string mujoco_root = TUTORIAL_MUJOCO_ROOT;
        const std::string plugin_dir = join_path(mujoco_root, "bin/mujoco_plugin");
        const std::string model_file =
            join_path(mujoco_root, "model/plugin/sensor/touch_grid.xml");

        mj_loadAllPluginLibraries(plugin_dir.c_str(), plugin_load_callback);

        auto simulation = mujoco_tutorial::load_simulation(model_file);
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        mj_step(model, data);

        mujoco_tutorial::print_model_summary(model);
        std::cout << "nplugin=" << model->nplugin << '\n';
        for (int sensor_id = 0; sensor_id < model->nsensor; ++sensor_id) {
            const int address = model->sensor_adr[sensor_id];
            const int dimension = model->sensor_dim[sensor_id];
            const int shown = std::min(dimension, 12);
            std::cout << "sensor="
                      << mujoco_tutorial::object_name(model, mjOBJ_SENSOR, sensor_id)
                      << " dim=" << dimension
                      << " first_values=[";
            for (int i = 0; i < shown; ++i) {
                if (i > 0) {
                    std::cout << ", ";
                }
                std::cout << std::fixed << std::setprecision(6)
                          << data->sensordata[address + i];
            }
            if (shown < dimension) {
                std::cout << ", ...";
            }
            std::cout << "]\n";
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
