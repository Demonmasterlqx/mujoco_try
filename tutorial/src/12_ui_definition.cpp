#include <mujoco/mujoco.h>

#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace {

mjuiDef make_definition(int type, const char* name, int state, void* data, const char* other) {
    mjuiDef definition{};
    definition.type = type;
    if (name != nullptr) {
        std::snprintf(definition.name, sizeof(definition.name), "%s", name);
    }
    definition.state = state;
    definition.pdata = data;
    if (other != nullptr) {
        std::snprintf(definition.other, sizeof(definition.other), "%s", other);
    }
    return definition;
}

}  // namespace

int main() {
    mjUI ui;
    std::memset(&ui, 0, sizeof(ui));
    ui.spacing = mjui_themeSpacing(0);
    ui.color = mjui_themeColor(0);

    int pause = 0;
    mjtNum gain = 1.0;
    char label[mjMAXUITEXT] = "headless ui demo";

    const std::array<mjuiDef, 5> definitions = {
        make_definition(mjITEM_SECTION, "Simulation", mjSECT_OPEN, nullptr, ""),
        make_definition(mjITEM_CHECKINT, "Pause", 1, &pause, ""),
        make_definition(mjITEM_SLIDERNUM, "Gain", 1, &gain, "0 10"),
        make_definition(mjITEM_EDITTXT, "Label", 1, label, ""),
        make_definition(mjITEM_END, nullptr, 0, nullptr, nullptr)
    };

    mjui_add(&ui, definitions.data());

    std::cout << "nsection=" << ui.nsect << '\n';
    if (ui.nsect > 0) {
        std::cout << "section0=" << ui.sect[0].name
                  << " nitem=" << ui.sect[0].nitem << '\n';
        for (int i = 0; i < ui.sect[0].nitem; ++i) {
            const mjuiItem& item = ui.sect[0].item[i];
            std::cout << "item" << i
                      << " name=" << item.name
                      << " type=" << item.type
                      << " state=" << item.state << '\n';
        }
    }

    std::cout << "pause=" << pause << " gain=" << gain << " label=" << label << '\n';
    return 0;
}
