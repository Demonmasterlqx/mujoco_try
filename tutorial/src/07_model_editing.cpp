#include "mujoco_tutorial/common.hpp"

#include <mujoco/mjspec.h>

#include <array>
#include <iostream>
#include <memory>

namespace {

struct SpecDeleter {
    void operator()(mjSpec* spec) const {
        if (spec != nullptr) {
            mj_deleteSpec(spec);
        }
    }
};

using SpecPtr = std::unique_ptr<mjSpec, SpecDeleter>;

}  // namespace

int main() {
    try {
        SpecPtr spec(mj_makeSpec());
        if (spec == nullptr) {
            throw std::runtime_error("mj_makeSpec failed");
        }

        mjs_setString(spec->modelname, "programmatic_box");
        spec->option.timestep = 0.005;
        spec->option.gravity[2] = -9.81;

        mjsBody* world = mjs_findBody(spec.get(), "world");
        if (world == nullptr) {
            throw std::runtime_error("mjs_findBody(world) failed");
        }

        mjsGeom* floor = mjs_addGeom(world, nullptr);
        mjs_setName(floor->element, "floor");
        floor->type = mjGEOM_PLANE;
        floor->size[0] = 1.0;
        floor->size[1] = 1.0;
        floor->size[2] = 0.05;
        floor->rgba[0] = 0.8F;
        floor->rgba[1] = 0.8F;
        floor->rgba[2] = 0.8F;
        floor->rgba[3] = 1.0F;

        mjsBody* body = mjs_addBody(world, nullptr);
        mjs_setName(body->element, "falling_box");
        body->pos[2] = 0.8;

        mjsJoint* joint = mjs_addFreeJoint(body);
        mjs_setName(joint->element, "free_joint");

        mjsGeom* box = mjs_addGeom(body, nullptr);
        mjs_setName(box->element, "box");
        box->type = mjGEOM_BOX;
        box->size[0] = 0.08;
        box->size[1] = 0.10;
        box->size[2] = 0.12;
        box->mass = 0.5;
        box->rgba[0] = 0.1F;
        box->rgba[1] = 0.5F;
        box->rgba[2] = 0.9F;
        box->rgba[3] = 1.0F;

        mujoco_tutorial::ModelPtr model(mj_compile(spec.get(), nullptr));
        if (model == nullptr) {
            const char* error = mjs_getError(spec.get());
            const std::string detail = (error == nullptr || error[0] == '\0')
                                           ? "no compiler detail"
                                           : error;
            throw std::runtime_error("mj_compile failed: " + detail);
        }

        mujoco_tutorial::DataPtr data(mj_makeData(model.get()));
        if (data == nullptr) {
            throw std::runtime_error("mj_makeData failed");
        }
        mujoco_tutorial::step_for(model.get(), data.get(), 0.2);

        std::array<char, 65536> xml{};
        std::array<char, 1024> error{};
        const int save_result = mj_saveXMLString(spec.get(), xml.data(),
                                                 static_cast<int>(xml.size()),
                                                 error.data(),
                                                 static_cast<int>(error.size()));
        if (save_result != 0) {
            const std::string detail = error[0] == '\0' ? "no save detail" : error.data();
            throw std::runtime_error("mj_saveXMLString failed: " + detail +
                                     ", return=" + std::to_string(save_result));
        }

        mujoco_tutorial::print_model_summary(model.get());
        std::cout << "saved_xml_prefix=" << std::string(xml.data()).substr(0, 160) << '\n';
        std::cout << "time=" << data->time << '\n';
        mujoco_tutorial::print_vector("qpos", data->qpos, model->nq);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}
