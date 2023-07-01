#version 460

in vec3 Color;
uniform vec4 tex;

layout (std140) uniform Camera 
{
    vec4 c1;
    vec4 c2;
};

out vec4 outColor;

void main()
{
    outColor = tex;
}