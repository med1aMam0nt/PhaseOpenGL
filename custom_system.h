#pragma once
#include <string>
#include <array>
#include <cstring>
#include <cmath>

#include "phase_math.h"

// tinyexpr
#include "tinyexpr.h"

namespace phase {

    struct CustomSystem final : ISystem2D {
        // Текст формул
        std::string fxStr = "y";
        std::string fyStr = "-x";

        // Параметры (фиксированный набор, удобно для GUI)
        // В формулах доступны: a,b,c,d,e,k,m,n,p,q + x,y
        double a = 1, b = 1, c = 1, d = 1, e = 0, k = 1, m = 1, n = 1, p = 0, q = 0;

        // переменные состояния для вычисления выражения
        mutable double xVar = 0.0;
        mutable double yVar = 0.0;

        // tinyexpr compiled
        te_expr* fx = nullptr;
        te_expr* fy = nullptr;

        // таблица переменных для tinyexpr (должна жить дольше выражения)
        std::array<te_variable, 12> vars{};

        CustomSystem() {
            // заполним vars с адресами
            vars = { {
                {"x", &xVar},
                {"y", &yVar},
                {"a", &a},
                {"b", &b},
                {"c", &c},
                {"d", &d},
                {"e", &e},
                {"k", &k},
                {"m", &m},
                {"n", &n},
                {"p", &p},
                {"q", &q},
            } };
            // попытаемся скомпилировать дефолт
            std::string err;
            compile(fxStr, fyStr, err);
        }

        ~CustomSystem() override {
            clearCompiled();
        }

        void clearCompiled() {
            if (fx) { te_free(fx); fx = nullptr; }
            if (fy) { te_free(fy); fy = nullptr; }
        }

        bool compile(const std::string& newFx, const std::string& newFy, std::string& outError) {
            outError.clear();
            fxStr = newFx;
            fyStr = newFy;

            clearCompiled();

            int errPos = 0;
            fx = te_compile(fxStr.c_str(), vars.data(), (int)vars.size(), &errPos);
            if (!fx) {
                outError = "dx/dt parse error at position " + std::to_string(errPos);
                return false;
            }

            errPos = 0;
            fy = te_compile(fyStr.c_str(), vars.data(), (int)vars.size(), &errPos);
            if (!fy) {
                te_free(fx); fx = nullptr;
                outError = "dy/dt parse error at position " + std::to_string(errPos);
                return false;
            }

            return true;
        }

        Vec2 F(Vec2 p) const override {
            // подставляем x,y
            xVar = p.x;
            yVar = p.y;

            // если по какой-то причине не скомпилировано — безопасный ноль
            if (!fx || !fy) return { 0.0, 0.0 };

            double dx = te_eval(fx);
            double dy = te_eval(fy);

            // защита от NaN/Inf: чтобы не ломать интегратор/поиск равновесий
            if (!std::isfinite(dx) || !std::isfinite(dy)) return { 0.0, 0.0 };
            return { dx, dy };
        }

        const char* name() const override { return "Custom"; }
    };

} // namespace phase