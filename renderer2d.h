#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <cstddef>

struct VertexPC {
    glm::vec2 pos;
    glm::vec3 color;
};

struct DrawRange {
    GLint first = 0;
    GLsizei count = 0;
};

class PolylineBatch {
public:
    GLuint vao = 0, vbo = 0;
    std::vector<VertexPC> cpuVertices;
    std::vector<DrawRange> ranges;

    void initGL() {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPC), (void*)offsetof(VertexPC, pos));

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPC), (void*)offsetof(VertexPC, color));

        glBindVertexArray(0);
    }

    void clear() {
        cpuVertices.clear();
        ranges.clear();
    }

    void addPolylineVertices(const std::vector<VertexPC>& verts) {
        if (verts.size() < 2) return;
        DrawRange r;
        r.first = (GLint)cpuVertices.size();
        r.count = (GLsizei)verts.size();
        ranges.push_back(r);

        cpuVertices.insert(cpuVertices.end(), verts.begin(), verts.end());
    }

    // удобный вариант "один цвет на линию"
    void addPolyline(const std::vector<glm::vec2>& pts, glm::vec3 color) {
        if (pts.size() < 2) return;
        std::vector<VertexPC> v;
        v.reserve(pts.size());
        for (auto& p : pts) v.push_back({ p, color });
        addPolylineVertices(v);
    }

    void upload(GLenum usage = GL_DYNAMIC_DRAW) {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
            cpuVertices.size() * sizeof(VertexPC),
            cpuVertices.empty() ? nullptr : cpuVertices.data(),
            usage);
    }

    void draw() const {
        glBindVertexArray(vao);
        for (auto& r : ranges) glDrawArrays(GL_LINE_STRIP, r.first, r.count);
        glBindVertexArray(0);
    }

    void destroy() {
        if (vbo) glDeleteBuffers(1, &vbo);
        if (vao) glDeleteVertexArrays(1, &vao);
        vao = vbo = 0;
    }
};

class LinesBatch {
public:
    GLuint vao = 0, vbo = 0;
    std::vector<VertexPC> cpuVertices; // пары вершин

    void initGL() {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPC), (void*)offsetof(VertexPC, pos));

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPC), (void*)offsetof(VertexPC, color));

        glBindVertexArray(0);
    }

    void clear() { cpuVertices.clear(); }

    void addSegment(glm::vec2 a, glm::vec2 b, glm::vec3 c) {
        cpuVertices.push_back({ a,c });
        cpuVertices.push_back({ b,c });
    }

    void upload(GLenum usage = GL_DYNAMIC_DRAW) {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
            cpuVertices.size() * sizeof(VertexPC),
            cpuVertices.empty() ? nullptr : cpuVertices.data(),
            usage);
    }

    void draw() const {
        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, (GLsizei)cpuVertices.size());
        glBindVertexArray(0);
    }

    void destroy() {
        if (vbo) glDeleteBuffers(1, &vbo);
        if (vao) glDeleteVertexArrays(1, &vao);
        vao = vbo = 0;
    }
};