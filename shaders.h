#pragma once

static const char* kLineVS = R"GLSL(
#version 330 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec3 aColor;

uniform mat4 uVP;

out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uVP * vec4(aPos, 0.0, 1.0);
}
)GLSL";

static const char* kLineFS = R"GLSL(
#version 330 core
in vec3 vColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(vColor, 1.0);
}
)GLSL";