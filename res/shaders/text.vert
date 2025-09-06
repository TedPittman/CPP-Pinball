#version 330 core
layout (location = 0) in vec4 vertex; // <vec2 pos, vec2 tex>
out vec2 TexCoords;
out vec4 TextColor;

uniform mat4 projection;
uniform vec3 textColor;

void main()
{
    gl_Position = projection * vec4(vertex.xy, 0.0, 1.0);
    TexCoords = vertex.zw;
    TextColor = vec4(textColor, 1.0);
}  