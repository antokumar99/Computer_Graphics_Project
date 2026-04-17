#version 330 core
layout(location = 0) in vec3 aPos;

out vec3 TexDir;

uniform mat4 projection;
uniform mat4 view;          // rotation-only (no translation)

void main()
{
    TexDir      = aPos;
    vec4 pos    = projection * view * vec4(aPos, 1.0);
    // Set z = w so after perspective divide depth = 1.0 (far plane – always behind scene)
    gl_Position = pos.xyww;
}
