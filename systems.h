#pragma once
#include "phase_math.h"

namespace phase {

    struct LinearSystem : ISystem2D {
        // x' = a x + b y
        // y' = c x + d y
        double a = 1, b = 0, c = 0, d = -1;

        Vec2 F(Vec2 p) const override {
            return { a * p.x + b * p.y, c * p.x + d * p.y };
        }
        const char* name() const override { return "Linear 2x2"; }
    };

    struct VanDerPol : ISystem2D {
        double mu = 1.0;
        Vec2 F(Vec2 p) const override {
            double x = p.x, y = p.y;
            return { y, mu * (1.0 - x * x) * y - x };
        }
        const char* name() const override { return "Van der Pol"; }
    };

    struct LotkaVolterra : ISystem2D {
        // x' = ax - bxy
        // y' = -cy + dxy
        double a = 1.5, b = 1.0, c = 3.0, d = 1.0;

        Vec2 F(Vec2 p) const override {
            double x = p.x, y = p.y;
            return { a * x - b * x * y, -c * y + d * x * y };
        }
        const char* name() const override { return "Lotka-Volterra"; }
    };

} // namespace phase