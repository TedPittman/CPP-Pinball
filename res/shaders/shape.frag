#version 330 core
in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D textureSampler;
uniform vec4 shapeColor;
uniform bool useTexture;

void main()
{
    if (useTexture) {
        FragColor = texture(textureSampler, TexCoord) * shapeColor;
    } else {
        FragColor = shapeColor; // Use solid color when not textured
    }
}