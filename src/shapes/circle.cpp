#include "circle.h"
#include "rect.h"


Circle::~Circle() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void Circle::setUniforms() const {
    Shape::setUniforms(); // Sets model and shapeColor uniforms
    shader.setFloat("radius", radius);
    shader.setVector2f("center", pos.x, pos.y);
}

void Circle::draw() const {
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, segments + 2); // +2 for center and last vertex
    glBindVertexArray(0);
}

void Circle::initVectors() {
    // center x, y, u, v
    vertices.push_back(0.0f);
    vertices.push_back(0.0f);
    vertices.push_back(0.5f);
    vertices.push_back(0.5f);

    // ring vertices
    for (int i = 0; i <= segments; ++i) {
        float theta = 2.0f * 3.1415926f * (float)i / (float)segments;
        float x = radius * cosf(theta);
        float y = radius * sinf(theta);
        // texture coordinates (-radius, radius) -> (0, 1)
        float u = 0.5f + x / (2.0f * radius);
        float v = 0.5f + y / (2.0f * radius);
        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(u);
        vertices.push_back(v);
    }
}

void Circle::setRadius(float radius) {
    this->radius = radius;
    size = vec2(radius * 2, radius * 2);
}

float Circle::getRadius() const { return radius; }

float Circle::getLeft() const   { return pos.x - radius; }
float Circle::getRight() const  { return pos.x + radius; }
float Circle::getTop() const    { return pos.y + radius; }
float Circle::getBottom() const { return pos.y - radius; }

bool Circle::isOverlapping(const Circle &c) const {
    // Check if the distance between the centers of the circles is less than the sum of their radii
    // distance = sqrt((x2 - x1)^2 + (y2 - y1)^2)
    float dist = distance(pos, c.getPos());
    float radiusSum = radius + c.getRadius();
    return dist < radiusSum;
}

bool Circle::isOverlapping(const Shape& other) const {
    return false; // placeholder value
}