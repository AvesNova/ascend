#version 460

in vec2 position;

out vec3 Color;

void main()
{
    Color = vec3(0.0, 0.0, 0.0);
    gl_Position = vec4(position, 0.0, 1.0);
}