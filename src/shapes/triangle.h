#ifndef GRAPHICS_TRIANGLE_H
#define GRAPHICS_TRIANGLE_H

#include "shape.h"
#include "../shader/shader.h"
#include <iostream>
using glm::vec2, glm::vec3;

class Triangle : public Shape {
public:
    /// @brief Construct a new Triangle object
    /// @details This constructor will call the InitRenderData function.
    /// @param shader The shader to use
    /// @param pos The position of the triangle
    /// @param size The size of the triangle
    /// @param color The color of the triangle
    Triangle(Shader & shader, vec2 pos, vec2 size, struct color fill);

    /// @brief Destroy the Triangle object and delete its VAO and VBO
    ~Triangle();

    /// @brief Binds the VAO and calls the virtual draw function
    void draw() const override;

    /// @brief Populates the vertices and indices vectors
    void initVectors() override;

    float getLeft() const override;
    float getRight() const override;
    float getTop() const override;
    float getBottom() const override;

    bool isOverlapping(const Shape& other) const override;
};

#endif //GRAPHICS_TRIANGLE_H
