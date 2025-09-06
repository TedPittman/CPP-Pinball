#include "rect.h"

Rect::Rect(Shader & shader, vec2 pos, vec2 size, struct color color)
    : Shape(shader, pos, size, color) {
    initVectors();
    initVAO();
    initVBO();
    initEBO();
}

Rect::Rect(Rect const& other) : Shape(other) {
    initVectors();
    initVAO();
    initVBO();
    initEBO();
}

Rect::~Rect() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void Rect::draw() const {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Rect::initVectors() {
    this->vertices.insert(vertices.end(), {
        -0.5f, 0.5f, 0.0f, 1.0f,  // Top left
        0.5f, 0.5f, 1.0f, 1.0f,  // Top right
        -0.5f, -0.5f, 0.0f, 0.0f, // Bottom left
        0.5f, -0.5f, 1.0f, 0.0f   // Bottom right
    });

    this->indices.insert(indices.end(), {
        0, 1, 2, // First triangle
        1, 2, 3  // Second triangle
    });
}
// Overridden Getters from Shape
float Rect::getLeft() const        { return pos.x - (size.x / 2); }
float Rect::getRight() const       { return pos.x + (size.x / 2); }
float Rect::getTop() const         { return pos.y + (size.y / 2); }
float Rect::getBottom() const      { return pos.y - (size.y / 2); }

bool Rect::isOverlapping(const Rect &r1, const Rect &r2) {
    // There are only two cases when rectangles are *not* overlapping:
    //    1. when one is to the left of the other
    //    2. when one is above the other
    // r1 left of r2
    if (r1.getRight() <= r2.getLeft()) {
        return false;
    }
    // r1 right of r2
    if (r1.getLeft() >= r2.getRight()) {
        return false;
    }
    // r1 above r2
    if (r1.getBottom() >= r2.getTop()) {
        return false;
    }
    // r1 below r2
    if (r1.getTop() <= r2.getBottom()) {
        return false;
    }

    return true;
}

bool Rect::isOverlapping(const Rect &other) const {
    return isOverlapping(*this, other);
}

bool Rect::isOverlapping(const Shape &other) const {
    // Dynamic cast to check if the other shape is a Rect
    const Rect* otherRect = dynamic_cast<const Rect*>(&other);
    if (otherRect) {
        return isOverlapping(*this, *otherRect);
    }
    return false;
}