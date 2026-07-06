#include "mujoco_tutorial/common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace mujoco_tutorial {
namespace {

std::string join_path(const char* root, std::string_view filename) {
    if (filename.empty()) {
        return std::string(root);
    }

    std::string path(root);
    if (!path.empty() && path.back() != '/') {
        path.push_back('/');
    }
    path.append(filename.data(), filename.size());
    return path;
}

}  // namespace

void ModelDeleter::operator()(mjModel* model) const {
    if (model != nullptr) {
        mj_deleteModel(model);
    }
}

void DataDeleter::operator()(mjData* data) const {
    if (data != nullptr) {
        mj_deleteData(data);
    }
}

std::string model_path(std::string_view filename) {
    return join_path(TUTORIAL_MODEL_DIR, filename);
}

std::string rm26_path(std::string_view filename) {
    return join_path(TUTORIAL_RM26_DIR, filename);
}

ModelPtr load_model(const std::string& path) {
    char error[1024] = {0};
    mjModel* raw_model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (raw_model == nullptr) {
        std::ostringstream message;
        message << "mj_loadXML failed for " << path << ": " << error;
        throw std::runtime_error(message.str());
    }
    return ModelPtr(raw_model);
}

Simulation load_simulation(const std::string& path) {
    ModelPtr model = load_model(path);
    mjData* raw_data = mj_makeData(model.get());
    if (raw_data == nullptr) {
        throw std::runtime_error("mj_makeData failed");
    }
    return Simulation{std::move(model), DataPtr(raw_data)};
}

std::string object_name(const mjModel* model, int object_type, int id) {
    const char* name = mj_id2name(model, object_type, id);
    if (name == nullptr || name[0] == '\0') {
        return "<unnamed>";
    }
    return name;
}

int require_id(const mjModel* model, int object_type, const char* name) {
    const int id = mj_name2id(model, object_type, name);
    if (id < 0) {
        std::ostringstream message;
        message << "Object not found: " << name;
        throw std::runtime_error(message.str());
    }
    return id;
}

void print_model_summary(const mjModel* model) {
    std::cout << "model=" << (model->names ? model->names : "<unnamed>")
              << " nq=" << model->nq
              << " nv=" << model->nv
              << " nu=" << model->nu
              << " nsensor=" << model->nsensor
              << " timestep=" << model->opt.timestep << '\n';
}

void print_vector(std::string_view label, const mjtNum* values, int count) {
    std::cout << label << "=[";
    for (int i = 0; i < count; ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << std::fixed << std::setprecision(6) << values[i];
    }
    std::cout << "]\n";
}

void print_sensor_table(const mjModel* model, const mjData* data) {
    for (int sensor_id = 0; sensor_id < model->nsensor; ++sensor_id) {
        const int address = model->sensor_adr[sensor_id];
        const int dimension = model->sensor_dim[sensor_id];
        const int type = model->sensor_type[sensor_id];
        const std::string name = object_name(model, mjOBJ_SENSOR, sensor_id);

        std::cout << std::setw(24) << std::left << name
                  << " type=" << std::setw(2) << type
                  << " adr=" << std::setw(3) << address
                  << " dim=" << dimension
                  << " value=[";
        for (int i = 0; i < dimension; ++i) {
            if (i > 0) {
                std::cout << ", ";
            }
            std::cout << std::fixed << std::setprecision(6) << data->sensordata[address + i];
        }
        std::cout << "]\n";
    }
}

int step_count_for_duration(const mjModel* model, double duration) {
    if (duration <= 0.0) {
        return 0;
    }

    const double timestep = model->opt.timestep;
    if (!std::isfinite(timestep) || timestep <= 0.0) {
        throw std::runtime_error("model timestep must be positive and finite");
    }

    const double exact_steps = duration / timestep;
    if (!std::isfinite(exact_steps) ||
        exact_steps > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("duration requires too many simulation steps");
    }

    const double nearest = std::round(exact_steps);
    const double tolerance = 1.0e-9 * std::max(1.0, std::abs(exact_steps));
    if (std::abs(exact_steps - nearest) <= tolerance) {
        return static_cast<int>(nearest);
    }
    return static_cast<int>(std::ceil(exact_steps));
}

void step_for(const mjModel* model, mjData* data, double duration) {
    const int steps = step_count_for_duration(model, duration);
    for (int step = 0; step < steps; ++step) {
        mj_step(model, data);
    }
}

}  // namespace mujoco_tutorial
