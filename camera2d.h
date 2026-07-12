#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct Camera2D {
    // Центр камеры в мировых координатах
    glm::dvec2 center{ 0.0, 0.0 };

    // "высота" видимой области мира (world units). Ширина зависит от aspect.
    double worldHeight = 10.0;

    int viewportW = 800;
    int viewportH = 600;

    double aspect() const {
        return (viewportH == 0) ? 1.0 : double(viewportW) / double(viewportH);
    }

    glm::dvec2 halfExtents() const {
        double hh = worldHeight * 0.5;
        double hw = hh * aspect();
        return { hw, hh };
    }

    glm::mat4 VP() const {
        auto he = halfExtents();
        double l = center.x - he.x;
        double r = center.x + he.x;
        double b = center.y - he.y;
        double t = center.y + he.y;
        // z нам не важен, но пусть будет [-1,1]
        return glm::ortho(float(l), float(r), float(b), float(t), -1.0f, 1.0f);
    }

    // из пикселей окна -> мировые координаты (z=0)
    glm::dvec2 screenToWorld(double sx, double sy) const {
        // sx in [0..W], sy in [0..H], sy сверху вниз
        double nx = (sx / double(viewportW)) * 2.0 - 1.0;
        double ny = 1.0 - (sy / double(viewportH)) * 2.0;

        auto he = halfExtents();
        return {
            center.x + nx * he.x,
            center.y + ny * he.y
        };
    }

    void zoom(double factor) {
        worldHeight *= factor;
        if (worldHeight < 1e-3) worldHeight = 1e-3;
        if (worldHeight > 1e6)  worldHeight = 1e6;
    }

    void pan(glm::dvec2 deltaWorld) {
        center += deltaWorld;
    }
};