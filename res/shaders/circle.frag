#version 330 core
in vec2 FragPos;
out vec4 FragColor;

uniform vec3 Color;
uniform vec2 CircleCenter;
uniform float Radius;

void main()
{
    // Calculate the distance from the center of the circle to the fragment
    float distance = length(FragPos - CircleCenter);

    // If the distance is greater than the radius of the circle, discard the fragment
    if (distance > Radius)
    {
        discard;
    }

    // If fragment is inside the circle, color it
    FragColor = vec4(Color, 1.0f);
}
