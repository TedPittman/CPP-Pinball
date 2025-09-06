#version 330 core
in vec2 TexCoords;
out vec4 color;

uniform sampler2D image;
uniform vec3 spriteColor;

void main()
{
    // A uniform color vector allows us to easily change the color of our sprite from outside the shader.
    // We calculate the final color by multiplying the texture by the sprite color vector.
    color = vec4(spriteColor, 1.0) * texture(image, TexCoords);
}