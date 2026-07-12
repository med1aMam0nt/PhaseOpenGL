#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <functional>
#include <optional>
#include <cmath>
#include <cstddef>
#include <limits>

namespace phase {

    // ------------------------- базовые типы -------------------------

    using Vec2 = glm::dvec2;
    using Mat2 = glm::dmat2;

    struct AABB {
        Vec2 min{ -10.0, -10.0 };
        Vec2 max{ 10.0,  10.0 };

        bool contains(Vec2 p) const {
            return (p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y);
        }
    };

    // ------------------------- система ОДУ -------------------------
    // Автономная система: p' = F(p), p=(x,y)

    struct ISystem2D {
        virtual ~ISystem2D() = default;
        virtual Vec2 F(Vec2 p) const = 0;
        virtual const char* name() const = 0;
    };

    // Удобный адаптер: система как std::function (для быстрого прототипа)
    struct FunctionSystem2D final : ISystem2D {
        std::function<Vec2(Vec2)> f;
        const char* sysName = "FunctionSystem2D";

        Vec2 F(Vec2 p) const override { return f(p); }
        const char* name() const override { return sysName; }
    };

    // ------------------------- интегратор RK4 -------------------------

    struct RK4 {
        const ISystem2D* sys = nullptr;
        double h = 0.01;

        Vec2 step(Vec2 p) const {
            // p_{n+1} = p_n + (h/6)(k1 + 2k2 + 2k3 + k4)
            const Vec2 k1 = sys->F(p);
            const Vec2 k2 = sys->F(p + 0.5 * h * k1);
            const Vec2 k3 = sys->F(p + 0.5 * h * k2);
            const Vec2 k4 = sys->F(p + 1.0 * h * k3);
            return p + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        }
    };

    // ------------------------- траектории -------------------------

    enum class TimeDirection { Forward, Backward };

    struct TrajectorySettings {
        double h = 0.01;                // шаг
        std::size_t maxSteps = 5000;     // максимум итераций
        AABB bounds;                    // область отображения/интегрирования
        double maxDelta = 1.0;           // если |p_{n+1}-p_n| слишком велик — остановить
        double stopSpeedEps = 1e-6;      // если |F(p)| < eps — считаем, что дошли до равновесия
        TimeDirection dir = TimeDirection::Forward;
    };

    struct Trajectory {
        std::vector<Vec2> points;
        // Можно расширить: хранить скорость/время на вершину.
    };

    inline Trajectory integrateTrajectory(const ISystem2D& sys, Vec2 p0, const TrajectorySettings& s)
    {
        Trajectory tr;
        tr.points.reserve(s.maxSteps + 1);

        RK4 rk;
        rk.sys = &sys;
        rk.h = (s.dir == TimeDirection::Forward) ? s.h : -s.h;

        Vec2 p = p0;
        if (!s.bounds.contains(p)) return tr;

        tr.points.push_back(p);

        for (std::size_t i = 0; i < s.maxSteps; ++i) {
            const Vec2 v = sys.F(p);
            const double speed = glm::length(v);
            if (speed < s.stopSpeedEps) break;

            const Vec2 pn = rk.step(p);

            const double delta = glm::length(pn - p);
            if (!std::isfinite(pn.x) || !std::isfinite(pn.y)) break;
            if (delta > s.maxDelta) break;
            if (!s.bounds.contains(pn)) break;

            tr.points.push_back(pn);
            p = pn;
        }

        return tr;
    }

    // ------------------------- векторное поле на сетке -------------------------

    struct FieldSample {
        Vec2 p;   // точка (x,y)
        Vec2 v;   // вектор F(p)
    };

    inline std::vector<FieldSample> sampleVectorField(
        const ISystem2D& sys, const AABB& box, int nx, int ny)
    {
        std::vector<FieldSample> out;
        out.reserve(std::size_t(nx) * std::size_t(ny));

        for (int j = 0; j < ny; ++j) {
            const double ty = (ny == 1) ? 0.0 : double(j) / double(ny - 1);
            const double y = box.min.y + ty * (box.max.y - box.min.y);

            for (int i = 0; i < nx; ++i) {
                const double tx = (nx == 1) ? 0.0 : double(i) / double(nx - 1);
                const double x = box.min.x + tx * (box.max.x - box.min.x);

                Vec2 p{ x, y };
                Vec2 v = sys.F(p);
                out.push_back({ p, v });
            }
        }
        return out;
    }

    // ------------------------- Якобиан и классификация равновесий -------------------------

    inline Mat2 jacobianCentralDiff(const ISystem2D& sys, Vec2 p, double h = 1e-5)
    {
        // J = [df/dx df/dy; dg/dx dg/dy]
        const Vec2 ex{ 1.0, 0.0 };
        const Vec2 ey{ 0.0, 1.0 };

        const Vec2 Fx1 = sys.F(p + h * ex);
        const Vec2 Fx0 = sys.F(p - h * ex);
        const Vec2 Fy1 = sys.F(p + h * ey);
        const Vec2 Fy0 = sys.F(p - h * ey);

        const Vec2 dFdx = (Fx1 - Fx0) / (2.0 * h);
        const Vec2 dFdy = (Fy1 - Fy0) / (2.0 * h);

        // glm матрица по столбцам: mat2(col0, col1)
        // col0 = (df/dx, dg/dx), col1 = (df/dy, dg/dy)
        return Mat2(dFdx, dFdy);
    }

    enum class EquilibriumType {
        Unknown,
        Saddle,
        StableNode,
        UnstableNode,
        StableFocus,
        UnstableFocus,
        Center,        // чисто мнимые, tr ~ 0, det > 0, D < 0
        Degenerate     // det ~ 0 или дискриминант ~ 0: требуется доп. анализ
    };

    struct EquilibriumInfo {
        Vec2 p;
        Mat2 J;
        double tr = 0.0;
        double det = 0.0;
        double disc = 0.0; // tr^2 - 4 det
        EquilibriumType type = EquilibriumType::Unknown;
    };

    inline EquilibriumType classifyByTrDet(double tr, double det, double disc,
        double eps = 1e-9, double centerTrEps = 1e-6)
    {
        // Классическая классификация для 2D линеризации
        if (std::abs(det) < eps) return EquilibriumType::Degenerate;

        if (det < 0.0) return EquilibriumType::Saddle;

        // det > 0
        if (std::abs(disc) < eps) {
            // кратный корень
            if (tr < 0.0) return EquilibriumType::StableNode;
            if (tr > 0.0) return EquilibriumType::UnstableNode;
            return EquilibriumType::Degenerate;
        }

        if (disc > 0.0) {
            // два действительных
            if (tr < 0.0) return EquilibriumType::StableNode;
            if (tr > 0.0) return EquilibriumType::UnstableNode;
            return EquilibriumType::Degenerate;
        }
        else {
            // комплексно-сопряженные
            if (std::abs(tr) < centerTrEps) return EquilibriumType::Center;
            if (tr < 0.0) return EquilibriumType::StableFocus;
            if (tr > 0.0) return EquilibriumType::UnstableFocus;
            return EquilibriumType::Unknown;
        }
    }

    inline EquilibriumInfo analyzeEquilibrium(const ISystem2D& sys, Vec2 p,
        double jacH = 1e-5)
    {
        EquilibriumInfo info;
        info.p = p;
        info.J = jacobianCentralDiff(sys, p, jacH);
        info.tr = info.J[0][0] + info.J[1][1]; // trace
        info.det = info.J[0][0] * info.J[1][1] - info.J[1][0] * info.J[0][1];
        info.disc = info.tr * info.tr - 4.0 * info.det;
        info.type = classifyByTrDet(info.tr, info.det, info.disc);
        return info;
    }

    // ------------------------- поиск равновесий (опционально) -------------------------

    inline bool newtonSolveEquilibrium(
        const ISystem2D& sys,
        Vec2& p,                    // in: начальное; out: решение
        int iters = 30,
        double epsF = 1e-10,
        double epsStep = 1e-12,
        double jacH = 1e-6)
    {
        for (int k = 0; k < iters; ++k) {
            const Vec2 F = sys.F(p);
            if (glm::length(F) < epsF) return true;

            const Mat2 J = jacobianCentralDiff(sys, p, jacH);

            // Решаем J * dp = -F для dp (2x2 вручную)
            const double a = J[0][0], b = J[1][0];
            const double c = J[0][1], d = J[1][1];
            const double det = a * d - b * c;
            if (std::abs(det) < 1e-14) return false;

            const Vec2 rhs = -F;
            const Vec2 dp{
                (d * rhs.x - c * rhs.y) / det,
                (-b * rhs.x + a * rhs.y) / det
            };

            p += dp;
            if (glm::length(dp) < epsStep) return true;
        }
        return false;
    }

    // Грубый поиск: пробежка по сетке -> кандидат где |F| маленькое -> Ньютон
    inline std::vector<Vec2> findEquilibriaGridNewton(
        const ISystem2D& sys,
        const AABB& box,
        int nx, int ny,
        double seedSpeedThresh = 1e-2,
        double mergeDist = 1e-3)
    {
        std::vector<Vec2> roots;
        roots.reserve(16);

        auto tooClose = [&](Vec2 r) {
            for (auto& q : roots)
                if (glm::length(r - q) < mergeDist) return true;
            return false;
            };

        for (int j = 0; j < ny; ++j) {
            const double ty = (ny == 1) ? 0.0 : double(j) / double(ny - 1);
            const double y = box.min.y + ty * (box.max.y - box.min.y);

            for (int i = 0; i < nx; ++i) {
                const double tx = (nx == 1) ? 0.0 : double(i) / double(nx - 1);
                const double x = box.min.x + tx * (box.max.x - box.min.x);

                Vec2 p{ x,y };
                const double speed = glm::length(sys.F(p));
                if (speed > seedSpeedThresh) continue;

                Vec2 r = p;
                if (!newtonSolveEquilibrium(sys, r)) continue;
                if (!box.contains(r)) continue;
                if (tooClose(r)) continue;

                roots.push_back(r);
            }
        }
        return roots;
    }

} // namespace phase