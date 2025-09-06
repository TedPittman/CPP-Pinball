#version 330 core
in vec2 TexCoords;
in vec4 TextColor;
out vec4 FragColor;

uniform sampler2D text;

void main()
{    
    vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, TexCoords).r);
    FragColor = TextColor * sampled;
}  