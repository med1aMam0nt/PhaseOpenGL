#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>

#include "phase_math.h"
#include "systems.h"
#include "custom_system.h"
#include "camera2d.h"
#include "gl_utils.h"
#include "renderer2d.h"
#include "shaders.h"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

static void framebuffer_size_callback(GLFWwindow*, int width, int height) {
    glViewport(0, 0, width, height);
}

static float clamp01(float x) { return std::max(0.0f, std::min(1.0f, x)); }
static glm::vec3 heatColor(float t) {
    t = clamp01(t);
    float r = clamp01(1.5f * t);
    float g = clamp01(1.5f * (1.0f - std::abs(t - 0.5f) * 2.0f));
    float b = clamp01(1.5f * (1.0f - t));
    return { r, g, b };
}

enum class SystemKind { Linear = 0, VanDerPol = 1, Lotka = 2, Custom = 3 };

struct AppState {
    Camera2D cam;
    phase::AABB worldBox{ {-5,-5},{5,5} };

    phase::LinearSystem linear;
    phase::VanDerPol vdp;
    phase::LotkaVolterra lotka;
    phase::CustomSystem custom; // NEW

    SystemKind current = SystemKind::VanDerPol;

    const phase::ISystem2D& sys() const {
        switch (current) {
        case SystemKind::Linear:    return linear;
        case SystemKind::VanDerPol: return vdp;
        case SystemKind::Lotka:     return lotka;
        case SystemKind::Custom:    return custom;
        }
        return vdp;
    }

    phase::TrajectorySettings trajS;

    PolylineBatch trajectories;
    LinesBatch vectorField;
    LinesBatch gridAxes;
    LinesBatch equilibriaCrosses;

    GLuint program = 0;
    GLint uVP = -1;

    std::vector<phase::EquilibriumInfo> eqInfos;

    // GUI state
    int demoNx = 10;
    int demoNy = 10;
    bool bothDirections = true;

    char fxBuf[256] = "y";
    char fyBuf[256] = "-x";
    std::string customError;
    bool autoApplyCustom = false;
};

// ---------- grid/axes ----------
static void rebuildGridAxes(AppState& st) {
    st.gridAxes.clear();

    double h = st.cam.worldHeight;
    double step = 1.0;
    if (h > 40) step = 5.0;
    if (h > 120) step = 10.0;
    if (h < 10) step = 0.5;
    if (h < 4) step = 0.2;

    auto he = st.cam.halfExtents();
    double l = st.cam.center.x - he.x;
    double r = st.cam.center.x + he.x;
    double b = st.cam.center.y - he.y;
    double t = st.cam.center.y + he.y;

    l -= step; r += step; b -= step; t += step;

    auto gridCol = glm::vec3(0.15f, 0.16f, 0.18f);
    auto axisCol = glm::vec3(0.35f, 0.36f, 0.40f);

    auto snapDown = [&](double x) { return std::floor(x / step) * step; };
    auto snapUp = [&](double x) { return std::ceil(x / step) * step; };

    for (double x = snapDown(l); x <= snapUp(r); x += step) {
        glm::vec3 col = (std::abs(x) < 1e-12) ? axisCol : gridCol;
        st.gridAxes.addSegment(glm::vec2((float)x, (float)b), glm::vec2((float)x, (float)t), col);
    }
    for (double y = snapDown(b); y <= snapUp(t); y += step) {
        glm::vec3 col = (std::abs(y) < 1e-12) ? axisCol : gridCol;
        st.gridAxes.addSegment(glm::vec2((float)l, (float)y), glm::vec2((float)r, (float)y), col);
    }

    st.gridAxes.upload(GL_DYNAMIC_DRAW);
}

// ---------- vector field ----------
static void rebuildVectorField(AppState& st, int nx = 30, int ny = 30) {
    st.vectorField.clear();
    auto samples = phase::sampleVectorField(st.sys(), st.worldBox, nx, ny);

    double len = 0.18 * (st.worldBox.max.y - st.worldBox.min.y) / double(ny);

    for (auto& s : samples) {
        double speed = glm::length(s.v);
        if (!std::isfinite(speed) || speed < 1e-12) continue;

        glm::dvec2 dir = s.v / speed;
        glm::dvec2 a = s.p - 0.5 * len * dir;
        glm::dvec2 b = s.p + 0.5 * len * dir;

        float t = (float)(speed / (1.0 + speed));
        glm::vec3 col = heatColor(t) * 0.8f;

        st.vectorField.addSegment(glm::vec2(a), glm::vec2(b), col);
    }
    st.vectorField.upload(GL_DYNAMIC_DRAW);
}

static glm::vec3 eqColor(phase::EquilibriumType tp) {
    using T = phase::EquilibriumType;
    switch (tp) {
    case T::Saddle:        return { 1.0f, 0.2f, 0.2f };
    case T::StableNode:
    case T::StableFocus:   return { 0.2f, 1.0f, 0.2f };
    case T::UnstableNode:
    case T::UnstableFocus: return { 1.0f, 0.6f, 0.2f };
    case T::Center:        return { 0.4f, 0.6f, 1.0f };
    default:               return { 0.9f, 0.9f, 0.9f };
    }
}

static void rebuildEquilibria(AppState& st) {
    st.eqInfos.clear();
    st.equilibriaCrosses.clear();

    auto roots = phase::findEquilibriaGridNewton(st.sys(), st.worldBox,
        45, 45,
        1e-2,
        1e-3);

    double cross = 0.06 * st.cam.worldHeight;
    for (auto& r : roots) {
        auto info = phase::analyzeEquilibrium(st.sys(), r);
        st.eqInfos.push_back(info);

        glm::vec3 col = eqColor(info.type);
        glm::vec2 p = glm::vec2((float)r.x, (float)r.y);

        st.equilibriaCrosses.addSegment(p + glm::vec2(-(float)cross, 0), p + glm::vec2((float)cross, 0), col);
        st.equilibriaCrosses.addSegment(p + glm::vec2(0, -(float)cross), p + glm::vec2(0, (float)cross), col);
    }

    st.equilibriaCrosses.upload(GL_DYNAMIC_DRAW);
}

static void rebuildAll(AppState& st) {
    rebuildVectorField(st);
    rebuildEquilibria(st);
    rebuildGridAxes(st);
}

static void clearTrajectories(AppState& st) {
    st.trajectories.clear();
    st.trajectories.upload(GL_DYNAMIC_DRAW);
}

// ---------- trajectories ----------
static void addTrajectoryFromSeed(AppState& st, glm::dvec2 seedWorld, bool bothDirections = true) {
    st.trajS.bounds = st.worldBox;

    auto buildVertices = [&](const std::vector<phase::Vec2>& pts, bool backward) {
        std::vector<VertexPC> v;
        v.reserve(pts.size());

        double vmax = 0.0;
        for (auto& p : pts) vmax = std::max(vmax, (double)glm::length(st.sys().F(p)));
        if (vmax < 1e-12) vmax = 1.0;

        for (auto& p : pts) {
            double speed = glm::length(st.sys().F(p));
            float t = (float)(speed / vmax);
            glm::vec3 col = heatColor(t);
            if (backward) col *= 0.65f;
            v.push_back({ glm::vec2((float)p.x, (float)p.y), col });
        }
        return v;
        };

    st.trajS.dir = phase::TimeDirection::Forward;
    auto trF = phase::integrateTrajectory(st.sys(), phase::Vec2(seedWorld), st.trajS);
    if (trF.points.size() >= 2) st.trajectories.addPolylineVertices(buildVertices(trF.points, false));

    if (bothDirections) {
        st.trajS.dir = phase::TimeDirection::Backward;
        auto trB = phase::integrateTrajectory(st.sys(), phase::Vec2(seedWorld), st.trajS);
        if (trB.points.size() >= 2) st.trajectories.addPolylineVertices(buildVertices(trB.points, true));
    }

    st.trajectories.upload(GL_DYNAMIC_DRAW);
}

static void addSeedGrid(AppState& st, int nx = 10, int ny = 10, bool both = true) {
    double mx = 0.05 * (st.worldBox.max.x - st.worldBox.min.x);
    double my = 0.05 * (st.worldBox.max.y - st.worldBox.min.y);

    for (int j = 0; j < ny; ++j) {
        double ty = (ny == 1) ? 0.5 : double(j) / double(ny - 1);
        double y = (st.worldBox.min.y + my) + ty * ((st.worldBox.max.y - my) - (st.worldBox.min.y + my));

        for (int i = 0; i < nx; ++i) {
            double tx = (nx == 1) ? 0.5 : double(i) / double(nx - 1);
            double x = (st.worldBox.min.x + mx) + tx * ((st.worldBox.max.x - mx) - (st.worldBox.min.x + mx));
            addTrajectoryFromSeed(st, { x,y }, both);
        }
    }
}

int main() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1100, 800, "Phase portraits + ImGui", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to init GLAD\n";
        return -1;
    }

    // ---- ImGui init ----
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, false);
    ImGui_ImplOpenGL3_Init("#version 330");

    AppState st;
    glfwGetFramebufferSize(window, &st.cam.viewportW, &st.cam.viewportH);
    st.cam.center = { 0,0 };
    st.cam.worldHeight = 10.0;

    // default demo parameters
    st.linear.a = 0; st.linear.b = 1; st.linear.c = -1; st.linear.d = 0; // rotation
    st.vdp.mu = 2.0;

    st.trajS.h = 0.01;
    st.trajS.maxSteps = 15000;
    st.trajS.maxDelta = 0.8;
    st.trajS.stopSpeedEps = 1e-6;

    // shaders/program
    GLuint vs = compileShader(GL_VERTEX_SHADER, kLineVS);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, kLineFS);
    st.program = linkProgram(vs, fs);
    st.uVP = glGetUniformLocation(st.program, "uVP");

    st.trajectories.initGL();
    st.vectorField.initGL();
    st.gridAxes.initGL();
    st.equilibriaCrosses.initGL();

    // init custom buffers with defaults
    strncpy_s(st.fxBuf, sizeof(st.fxBuf), st.custom.fxStr.c_str(), _TRUNCATE);
    strncpy_s(st.fyBuf, sizeof(st.fyBuf), st.custom.fyStr.c_str(), _TRUNCATE);
    st.fxBuf[sizeof(st.fxBuf) - 1] = 0;
    st.fyBuf[sizeof(st.fyBuf) - 1] = 0;

    rebuildAll(st);

    glfwSetWindowUserPointer(window, &st);

    glfwSetMouseButtonCallback(window, [](GLFWwindow* w, int button, int action, int mods)
        {
            ImGui_ImplGlfw_MouseButtonCallback(w, button, action, mods);

            if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) return;
            if (ImGui::GetIO().WantCaptureMouse) return;

            auto* st = (AppState*)glfwGetWindowUserPointer(w);
            double mx, my;
            glfwGetCursorPos(w, &mx, &my);
            glm::dvec2 seed = st->cam.screenToWorld(mx, my);
            addTrajectoryFromSeed(*st, seed, st->bothDirections);
        });

    glfwSetScrollCallback(window, [](GLFWwindow* w, double xoff, double yoff)
        {
            ImGui_ImplGlfw_ScrollCallback(w, xoff, yoff); // <-- важно
            if (ImGui::GetIO().WantCaptureMouse) return;

            auto* st = (AppState*)glfwGetWindowUserPointer(w);
            if (yoff > 0) st->cam.zoom(0.9);
            if (yoff < 0) st->cam.zoom(1.1);
            rebuildGridAxes(*st);
            rebuildEquilibria(*st);
        });

    glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int scancode, int action, int mods)
        {
            ImGui_ImplGlfw_KeyCallback(w, key, scancode, action, mods);

            if (ImGui::GetIO().WantCaptureKeyboard)
                return;

            if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
                glfwSetWindowShouldClose(w, true);
        });

    glfwSetCharCallback(window, [](GLFWwindow* w, unsigned int c)
        {
            ImGui_ImplGlfw_CharCallback(w, c);
        });

    glfwSetCursorPosCallback(window, [](GLFWwindow* w, double x, double y)
        {
            ImGui_ImplGlfw_CursorPosCallback(w, x, y);
        });

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    while (!glfwWindowShouldClose(window)) {
        glfwGetFramebufferSize(window, &st.cam.viewportW, &st.cam.viewportH);

        // camera pan (оставил WASD — удобно даже с GUI)
        double panStep = 0.02 * st.cam.worldHeight;
        if (!ImGui::GetIO().WantCaptureKeyboard) {
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) st.cam.pan({ -panStep, 0 });
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) st.cam.pan({ panStep, 0 });
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) st.cam.pan({ 0, panStep });
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) st.cam.pan({ 0,-panStep });
        }

        // ----- ImGui frame -----
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Once);
        ImGui::Begin("Controls");

        const char* items[] = { "Linear", "Van der Pol", "Lotka-Volterra", "Custom" };
        int cur = (int)st.current;
        if (ImGui::Combo("System", &cur, items, IM_ARRAYSIZE(items))) {
            st.current = (SystemKind)cur;
            clearTrajectories(st);
            rebuildAll(st);
        }

        ImGui::Separator();

        if (st.current == SystemKind::VanDerPol) {
            ImGui::Separator();
            ImGui::Text("Presets");

            if (ImGui::Button("mu = 0.5")) { st.vdp.mu = 0.5; clearTrajectories(st); rebuildAll(st); }
            ImGui::SameLine();
            if (ImGui::Button("mu = 2")) { st.vdp.mu = 2.0; clearTrajectories(st); rebuildAll(st); }
            ImGui::SameLine();
            if (ImGui::Button("mu = 5")) { st.vdp.mu = 5.0; clearTrajectories(st); rebuildAll(st); }
        }

        if (st.current == SystemKind::Linear) {
            ImGui::Separator();
            ImGui::Text("Presets");

            auto applyLinear = [&](double a, double b, double c, double d) {
                st.linear.a = a; st.linear.b = b; st.linear.c = c; st.linear.d = d;
                clearTrajectories(st);
                rebuildAll(st);
                };

            if (ImGui::Button("Center (rotation)")) {
                applyLinear(0, 1, -1, 0);
            }
            ImGui::SameLine();
            if (ImGui::Button("Saddle")) {
                applyLinear(1, 0, 0, -1);
            }

            if (ImGui::Button("Stable node")) {
                applyLinear(-1, 0, 0, -0.4);
            }
            ImGui::SameLine();
            if (ImGui::Button("Unstable node")) {
                applyLinear(1, 0, 0, 0.5);
            }

            if (ImGui::Button("Stable focus")) {
                applyLinear(-0.2, 1.0, -1.0, -0.2);
            }
            ImGui::SameLine();
            if (ImGui::Button("Unstable focus")) {
                applyLinear(0.2, 1.0, -1.0, 0.2);
            }
        }

        if (st.current == SystemKind::Custom)
        {
            ImGui::TextUnformatted("Custom system");
            ImGui::TextUnformatted("Vars: x, y. Params: a b c d e k m n p q");
            ImGui::Separator();

            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::TextUnformatted(
                    "Operators: +  -  *  /  ^\n"
                    "Functions: sin cos tan asin acos atan\n"
                    "           sqrt abs exp log\n"
                    "Constants: pi, e (if your tinyexpr build supports them)\n"
                    "Examples:\n"
                    "  dx/dt = y\n"
                    "  dy/dt = -x + a*(1-x^2)*y"
                );
                ImGui::EndTooltip();
            }

            // Формулы
            ImGui::InputText("dx/dt", st.fxBuf, IM_ARRAYSIZE(st.fxBuf));
            ImGui::InputText("dy/dt", st.fyBuf, IM_ARRAYSIZE(st.fyBuf));

            // (опционально) авто-применение по Enter
            // if (ImGui::IsItemDeactivatedAfterEdit()) { ... }  // можно позже сделать аккуратно

            bool applyPressed = ImGui::Button("Apply (compile)");
            ImGui::SameLine();
            if (ImGui::Button("Clear error")) st.customError.clear();

            if (applyPressed)
            {
                std::string err;
                const bool ok = st.custom.compile(st.fxBuf, st.fyBuf, err);
                st.customError = ok ? "" : err;

                if (ok)
                {
                    clearTrajectories(st);
                    rebuildAll(st);
                }
            }

            if (!st.customError.empty())
            {
                ImGui::TextColored(ImVec4(1, 0.35f, 0.35f, 1), "Error: %s", st.customError.c_str());
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Parameters");
            bool pChanged = false;

            // параметры через Scalar-обёртку
            pChanged |= ImGui::InputDouble("a", &st.custom.a, 0.0, 0.0, "%.6f");
            pChanged |= ImGui::InputDouble("b", &st.custom.b, 0.0, 0.0, "%.6f");
            pChanged |= ImGui::InputDouble("c", &st.custom.c, 0.0, 0.0, "%.6f");
            pChanged |= ImGui::InputDouble("d", &st.custom.d, 0.0, 0.0, "%.6f");
            pChanged |= ImGui::InputDouble("e", &st.custom.e, 0.0, 0.0, "%.6f");
            pChanged |= ImGui::InputDouble("k", &st.custom.k, 0.0, 0.0, "%.6f");
            pChanged |= ImGui::InputDouble("m", &st.custom.m, 0.0, 0.0, "%.6f");
            pChanged |= ImGui::InputDouble("n", &st.custom.n, 0.0, 0.0, "%.6f");
            pChanged |= ImGui::InputDouble("p", &st.custom.p, 0.0, 0.0, "%.6f");
            pChanged |= ImGui::InputDouble("q", &st.custom.q, 0.0, 0.0, "%.6f");

            if (pChanged)
            {
                // параметры влияют на поле/равновесия/траектории
                clearTrajectories(st);
                rebuildAll(st);
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Presets");

            auto setCustom = [&](const char* fx, const char* fy)
                {
                    // заполняем поля ввода
                    snprintf(st.fxBuf, IM_ARRAYSIZE(st.fxBuf), "%s", fx);
                    snprintf(st.fyBuf, IM_ARRAYSIZE(st.fyBuf), "%s", fy);

                    std::string err;
                    bool ok = st.custom.compile(st.fxBuf, st.fyBuf, err);
                    st.customError = ok ? "" : err;

                    clearTrajectories(st);
                    rebuildAll(st);
                };

            if (ImGui::Button("Harmonic (x'=y, y'=-x)"))
                setCustom("y", "-x");

            if (ImGui::Button("Van der Pol (x'=y, y'=-x + a*(1-x^2)*y)"))
            {
                st.custom.a = 2.0;
                setCustom("y", "-x + a*(1 - x^2)*y");
            }

            if (ImGui::Button("Lotka-Volterra (a,b,c,d)"))
            {
                st.custom.a = 1.5; st.custom.b = 1.0; st.custom.c = 1.0; st.custom.d = 1.0;
                setCustom("a*x - b*x*y", "-c*y + d*x*y");
            }
        }

        ImGui::Separator();
        ImGui::Checkbox("Traj both directions", &st.bothDirections);

        ImGui::DragInt("Demo Nx", &st.demoNx, 1, 2, 40);
        ImGui::DragInt("Demo Ny", &st.demoNy, 1, 2, 40);

        if (ImGui::Button("Demo (grid)")) {
            addSeedGrid(st, st.demoNx, st.demoNy, st.bothDirections);
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear trajectories")) {
            clearTrajectories(st);
        }
        ImGui::SameLine();
        if (ImGui::Button("Rebuild field")) {
            rebuildAll(st);
        }

        ImGui::Separator();
        ImGui::Text("Mouse: click to add trajectory");
        ImGui::Text("WASD: pan, Wheel: zoom, Esc: exit");

        ImGui::End();

        // rebuild grid every frame (as before)
        rebuildGridAxes(st);

        // ----- render -----
        glClearColor(0.07f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(st.program);
        glm::mat4 VP = st.cam.VP();
        glUniformMatrix4fv(st.uVP, 1, GL_FALSE, &VP[0][0]);

        st.gridAxes.draw();
        st.vectorField.draw();
        st.equilibriaCrosses.draw();
        st.trajectories.draw();

        // ImGui draw on top
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);
    }

    // shutdown
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    st.trajectories.destroy();
    st.vectorField.destroy();
    st.gridAxes.destroy();
    st.equilibriaCrosses.destroy();
    glDeleteProgram(st.program);

    glfwTerminate();
    return 0;
}