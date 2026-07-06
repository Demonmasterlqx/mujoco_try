#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace mujoco_tutorial {

struct ModelDeleter {
    void operator()(mjModel* model) const;
};

struct DataDeleter {
    void operator()(mjData* data) const;
};

using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

struct Simulation {
    ModelPtr model;
    DataPtr data;
};

std::string model_path(std::string_view filename);
std::string rm26_path(std::string_view filename);
ModelPtr load_model(const std::string& path);
Simulation load_simulation(const std::string& path);
std::string object_name(const mjModel* model, int object_type, int id);
int require_id(const mjModel* model, int object_type, const char* name);
void print_model_summary(const mjModel* model);
void print_vector(std::string_view label, const mjtNum* values, int count);
void print_sensor_table(const mjModel* model, const mjData* data);
int step_count_for_duration(const mjModel* model, double duration);
void step_for(const mjModel* model, mjData* data, double duration);

}  // namespace mujoco_tutorial
