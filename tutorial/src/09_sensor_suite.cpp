#include "mujoco_tutorial/common.hpp"

#include <iostream>

namespace {

void user_sensor_callback(const mjModel* model, mjData* data, int stage) {
    if (stage != mjSTAGE_VEL) {
        return;
    }

    for (int sensor_id = 0; sensor_id < model->nsensor; ++sensor_id) {
        if (model->sensor_type[sensor_id] != mjSENS_USER) {
            continue;
        }

        const int address = model->sensor_adr[sensor_id];
        const int body_id = model->sensor_objid[sensor_id];
        data->sensordata[address] = data->xpos[3 * body_id + 2];
        // The user sensor exposes qvel[0], the hinge generalized velocity, as its
        // second custom output value.
        data->sensordata[address + 1] = data->qvel[0];
    }
}

class ScopedSensorCallback {
public:
    explicit ScopedSensorCallback(mjfSensor callback)
        : previous_(mjcb_sensor) {
        mjcb_sensor = callback;
    }

    ~ScopedSensorCallback() {
        mjcb_sensor = previous_;
    }

private:
    mjfSensor previous_;
};

}  // namespace

int main() {
    try {
        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("sensor_suite.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        // Seed generalized position/velocity and actuator command so the sensor
        // table has nonzero joint, IMU, actuator, and user-sensor readings.
        data->qpos[0] = 0.3;
        data->qvel[0] = -0.2;
        data->ctrl[0] = 1.0;

        const mjfSensor previous_callback = mjcb_sensor;
        {
            const ScopedSensorCallback sensor_guard(user_sensor_callback);
            mj_forward(model, data);
            mj_step(model, data);
            mj_forward(model, data);
            std::cout << "nsensor=" << model->nsensor
                      << " nsensordata=" << model->nsensordata << '\n';
            mujoco_tutorial::print_sensor_table(model, data);
        }

        std::cout << "sensor_callback_restored="
                  << (mjcb_sensor == previous_callback ? "true" : "false") << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
