#include "mujoco_tutorial/common.hpp"
#include "mujoco_tutorial/double_pendulum_controller.hpp"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

constexpr double kRenderPeriod = 1.0 / 60.0;

struct Options {
    double duration = 8.0;
    bool interactive = false;
    bool headless_check = false;
    bool require_window = false;
};

struct ViewerState {
    const mjModel* model = nullptr;
    mjData* data = nullptr;
    mujoco_tutorial::MujocoJointSystem* system = nullptr;
    mujoco_tutorial::ControllerInterface* controller = nullptr;
    mjvCamera* camera = nullptr;
    mjvScene* scene = nullptr;
    bool paused = false;
    bool reset_requested = false;
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double last_x = 0.0;
    double last_y = 0.0;
};

ViewerState* g_viewer = nullptr;
std::string g_glfw_error;

class GlfwSession {
public:
    GlfwSession() = default;
    GlfwSession(const GlfwSession&) = delete;
    GlfwSession& operator=(const GlfwSession&) = delete;

    void activate() {
        active_ = true;
    }

    ~GlfwSession() {
        if (active_) {
            glfwTerminate();
        }
    }

private:
    bool active_ = false;
};

class GlfwWindow {
public:
    explicit GlfwWindow(GLFWwindow* window) : window_(window) {}
    GlfwWindow(const GlfwWindow&) = delete;
    GlfwWindow& operator=(const GlfwWindow&) = delete;

    ~GlfwWindow() {
        if (window_ != nullptr) {
            glfwDestroyWindow(window_);
        }
    }

    GLFWwindow* get() const {
        return window_;
    }

private:
    GLFWwindow* window_ = nullptr;
};

void print_usage(const char* program) {
    std::cout << "Usage: " << program
              << " [--duration seconds] [--interactive] [--headless-check] [--require-window]\n";
}

double parse_duration_value(const std::string& text) {
    std::size_t parsed = 0;
    double value = 0.0;
    try {
        value = std::stod(text, &parsed);
    } catch (const std::exception&) {
        std::ostringstream message;
        message << "duration must be a finite non-negative number: " << text;
        throw std::runtime_error(message.str());
    }
    if (parsed != text.size() || !std::isfinite(value) || value < 0.0) {
        std::ostringstream message;
        message << "duration must be a finite non-negative number: " << text;
        throw std::runtime_error(message.str());
    }
    return value;
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--duration") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--duration requires a numeric argument");
            }
            options.duration = parse_duration_value(argv[++i]);
        } else if (arg == "--interactive") {
            options.interactive = true;
        } else if (arg == "--headless-check") {
            options.headless_check = true;
        } else if (arg == "--require-window") {
            options.require_window = true;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            std::ostringstream message;
            message << "unknown argument: " << arg;
            throw std::runtime_error(message.str());
        }
    }
    return options;
}

void glfw_error_callback(int, const char* description) {
    g_glfw_error = description == nullptr ? "unknown GLFW error" : description;
}

void keyboard_callback(GLFWwindow* window, int key, int, int action, int) {
    if (action != GLFW_PRESS || g_viewer == nullptr) {
        return;
    }

    if (key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    } else if (key == GLFW_KEY_SPACE) {
        g_viewer->paused = !g_viewer->paused;
    } else if (key == GLFW_KEY_BACKSPACE) {
        g_viewer->reset_requested = true;
    }
}

void mouse_button_callback(GLFWwindow* window, int, int, int) {
    if (g_viewer == nullptr) {
        return;
    }

    g_viewer->button_left =
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    g_viewer->button_middle =
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    g_viewer->button_right =
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &g_viewer->last_x, &g_viewer->last_y);
}

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
    if (g_viewer == nullptr || g_viewer->camera == nullptr || g_viewer->scene == nullptr) {
        return;
    }
    if (!g_viewer->button_left && !g_viewer->button_middle && !g_viewer->button_right) {
        return;
    }

    const double dx = xpos - g_viewer->last_x;
    const double dy = ypos - g_viewer->last_y;
    g_viewer->last_x = xpos;
    g_viewer->last_y = ypos;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);
    if (width <= 0 || height <= 0) {
        return;
    }

    const bool shift_pressed =
        glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    mjtMouse action = mjMOUSE_ZOOM;
    if (g_viewer->button_right) {
        action = shift_pressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (g_viewer->button_left) {
        action = shift_pressed ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }

    mjv_moveCamera(g_viewer->model, action,
                   static_cast<mjtNum>(dx / height),
                   static_cast<mjtNum>(dy / height),
                   g_viewer->scene, g_viewer->camera);
}

void scroll_callback(GLFWwindow*, double, double yoffset) {
    if (g_viewer == nullptr || g_viewer->camera == nullptr || g_viewer->scene == nullptr) {
        return;
    }
    mjv_moveCamera(g_viewer->model, mjMOUSE_ZOOM, 0.0,
                   static_cast<mjtNum>(-0.05 * yoffset),
                   g_viewer->scene, g_viewer->camera);
}

void configure_camera(mjvCamera* camera) {
    mjv_defaultCamera(camera);
    camera->type = mjCAMERA_FREE;
    camera->lookat[0] = 0.0;
    camera->lookat[1] = 0.0;
    camera->lookat[2] = 0.55;
    camera->distance = 1.8;
    camera->azimuth = 90.0;
    camera->elevation = -12.0;
}

void step_controller_steps(const mjModel* model, mjData* data,
                           mujoco_tutorial::MujocoJointSystem& system,
                           mujoco_tutorial::ControllerInterface& controller,
                           int steps) {
    for (int step = 0; step < steps; ++step) {
        mujoco_tutorial::step_double_pendulum_controller(
            model, data, system, controller);
    }
}

int run_headless_check(const mjModel* model, mjData* data,
                       mujoco_tutorial::MujocoJointSystem& system,
                       mujoco_tutorial::ControllerInterface& controller,
                       const std::string& reason) {
    const int steps = mujoco_tutorial::step_count_for_duration(model, 0.25);
    step_controller_steps(model, data, system, controller, steps);

    const mujoco_tutorial::JointState state = system.read();
    if (!reason.empty()) {
        std::cout << "visualization_unavailable=" << reason << '\n';
    }
    std::cout << "headless_check_steps=" << steps << '\n';
    std::cout << "headless_check_time=" << data->time << '\n';
    std::cout << "joint1_position=" << state.position[0] << '\n';
    std::cout << "joint2_position=" << state.position[1] << '\n';
    return 0;
}

std::string overlay_left(const mjData* data, const mujoco_tutorial::JointState& state) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "time " << data->time << '\n'
           << "joint1 " << state.position[0] << " rad\n"
           << "joint2 " << state.position[1] << " rad";
    return stream.str();
}

std::string overlay_right(const mjData* data, bool paused) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << (paused ? "paused" : "running") << '\n'
           << "ctrl1 " << data->ctrl[0] << '\n'
           << "ctrl2 " << data->ctrl[1] << '\n'
           << "space pause\nbackspace reset\nesc close";
    return stream.str();
}

int run_viewer(const mjModel* model, mjData* data,
               mujoco_tutorial::MujocoJointSystem& system,
               mujoco_tutorial::ControllerInterface& controller,
               const Options& options) {
    GlfwSession glfw_session;
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        const std::string reason =
            g_glfw_error.empty() ? "glfwInit failed" : g_glfw_error;
        if (options.require_window) {
            std::cerr << reason << '\n';
            return 2;
        }
        return run_headless_check(model, data, system, controller, reason);
    }
    glfw_session.activate();

    GlfwWindow window(glfwCreateWindow(
        1200, 900, "MuJoCo visualized double inverted pendulum", nullptr, nullptr));
    if (window.get() == nullptr) {
        const std::string reason =
            g_glfw_error.empty() ? "glfwCreateWindow failed" : g_glfw_error;
        if (options.require_window) {
            std::cerr << reason << '\n';
            return 2;
        }
        return run_headless_check(model, data, system, controller, reason);
    }

    glfwMakeContextCurrent(window.get());
    glfwSwapInterval(1);

    mjvCamera camera;
    mjvOption visualization_options;
    mjvScene scene;
    mjrContext context;
    configure_camera(&camera);
    mjv_defaultOption(&visualization_options);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    mjv_makeScene(model, &scene, 2000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    ViewerState viewer;
    viewer.model = model;
    viewer.data = data;
    viewer.system = &system;
    viewer.controller = &controller;
    viewer.camera = &camera;
    viewer.scene = &scene;
    g_viewer = &viewer;

    glfwSetKeyCallback(window.get(), keyboard_callback);
    glfwSetMouseButtonCallback(window.get(), mouse_button_callback);
    glfwSetCursorPosCallback(window.get(), mouse_move_callback);
    glfwSetScrollCallback(window.get(), scroll_callback);

    std::cout << "window_opened=true\n";
    std::cout << "duration=" << (options.interactive ? -1.0 : options.duration) << '\n';

    const int steps_per_frame = mujoco_tutorial::step_count_for_duration(
        model, kRenderPeriod);
    int remaining_steps = options.interactive
        ? -1
        : mujoco_tutorial::step_count_for_duration(model, options.duration);
    int simulation_steps = 0;
    int render_frames = 0;

    while (!glfwWindowShouldClose(window.get())) {
        if (viewer.reset_requested) {
            mujoco_tutorial::reset_double_pendulum(model, data);
            controller.activate();
            viewer.reset_requested = false;
            if (!options.interactive) {
                remaining_steps = mujoco_tutorial::step_count_for_duration(
                    model, options.duration);
                simulation_steps = 0;
            }
        }

        if (!viewer.paused) {
            const int steps_this_frame = options.interactive
                ? steps_per_frame
                : std::min(steps_per_frame, remaining_steps);
            step_controller_steps(model, data, system, controller, steps_this_frame);
            if (!options.interactive) {
                remaining_steps -= steps_this_frame;
            }
            simulation_steps += steps_this_frame;
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window.get(), &viewport.width, &viewport.height);

        mjv_updateScene(model, data, &visualization_options, nullptr,
                        &camera, mjCAT_ALL, &scene);
        mjr_render(viewport, &scene, &context);

        const mujoco_tutorial::JointState state = system.read();
        const std::string left = overlay_left(data, state);
        const std::string right = overlay_right(data, viewer.paused);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                    left.c_str(), right.c_str(), &context);

        glfwSwapBuffers(window.get());
        glfwPollEvents();
        ++render_frames;

        if (!options.interactive && remaining_steps <= 0) {
            glfwSetWindowShouldClose(window.get(), GLFW_TRUE);
        }
    }

    const mujoco_tutorial::JointState final_state = system.read();
    std::cout << "window_closed=true\n";
    std::cout << "final_time=" << data->time << '\n';
    std::cout << "simulation_steps=" << simulation_steps << '\n';
    std::cout << "render_frames=" << render_frames << '\n';
    std::cout << "joint1_position=" << final_state.position[0] << '\n';
    std::cout << "joint2_position=" << final_state.position[1] << '\n';

    g_viewer = nullptr;
    mjv_freeScene(&scene);
    mjr_freeContext(&context);

    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);

        auto simulation = mujoco_tutorial::load_simulation(
            mujoco_tutorial::model_path("double_inverted_pendulum.xml"));
        const mjModel* model = simulation.model.get();
        mjData* data = simulation.data.get();

        mujoco_tutorial::reset_double_pendulum(model, data);

        mujoco_tutorial::MujocoJointSystem system(model, data);
        mujoco_tutorial::UprightPdController controller;
        if (!controller.configure(model)) {
            throw std::runtime_error("controller configure failed");
        }
        controller.activate();

        if (options.headless_check) {
            return run_headless_check(model, data, system, controller, "");
        }
        return run_viewer(model, data, system, controller, options);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
