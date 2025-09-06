#include "shape.h"

Shape::Shape(Shader &shader, glm::vec2 pos, glm::vec2 size, color c) :
    shader(shader), pos(pos), size(size), fill(c) {}

Shape::Shape(Shape const& other) :
    shader(other.shader), pos(other.pos), size(other.size), fill(other.fill),
    rotationAngle(other.rotationAngle), rotationPivot(other.rotationPivot) {}

// Initialize VAO
unsigned int Shape::initVAO() {
    glGenVertexArrays(1, &VAO); // Generate VAO
    glBindVertexArray(VAO); // Bind VAO
    return VAO;
}

// Initialize VBO
void Shape::initVBO() {
    // Generate VBO, bind it to VAO, and copy vertices data into it
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Set the vertex attribute pointers (2 floats per vertex (x, y))
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0); // Enable the vertex attribute at location 0
    // Texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
}

// Initialize EBO
void Shape::initEBO() {
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(float), indices.data(), GL_STATIC_DRAW);
    // Don't unbind EBO because it's bound to VAO
}

void Shape::setUniforms() const {
    // If you want to use a custom shader, you have to set it and call it's Use() function here.
    // Since we are using the same shader for all shapes, we can just set it once in the constructor.
    //this->shader.use();

    // Define the model matrix for the shape as a 4x4 identity matrix
    mat4 model = mat4(1.0f);

    // translate to the flipper’s center pos
    model = translate(model, vec3(pos, 0.0f));
    // apply translation by local pivot offset.
    // rotationPivot here is an offset vector relative to pos
    model = translate(model, vec3(rotationPivot, 0.0f));
    model = rotate(model, glm::radians(rotationAngle), vec3(0.0f, 0.0f, 1.0f));
    // undo extra translation.
    model = translate(model, vec3(-rotationPivot, 0.0f));
    // scale shape.
    model = scale(model, vec3(size, 1.0f));

    // // The model matrix is used to transform the vertices of the shape in relation to the world space.
    // model = translate(model, vec3(pos, 1.0f));
    // // The size of the shape is scaled by the model matrix to make the shape larger or smaller.
    // model = scale(model, vec3(size, 1.0f));

    // Set the model matrix and color uniform variables in the shader
    this->shader.setMatrix4("model", model);
    this->shader.setVector4f("shapeColor", fill.vec);
}

// Setters
void Shape::move(vec2 offset)         { pos += offset; }
void Shape::moveX(float x)            { pos.x += x; }
void Shape::moveY(float y)            { pos.y += y; }
void Shape::setPos(vec2 pos)          { this->pos = pos; }
void Shape::setPosX(float x)          { pos.x = x; }
void Shape::setPosY(float y)          { pos.y = y; }
void Shape::setVelocity(vec2 v)       { velocity = v; }
void Shape::addVelocity(vec2 v)       { velocity += v; }
void Shape::setRotation(float degrees, float pivotX, float pivotY) {
    rotationAngle = degrees;
    rotationPivot = vec2(pivotX, pivotY);
}
void Shape::setRotationDegrees(float degrees) {
    rotationAngle = degrees;
}
void Shape::setRotationPivot(float pivotX, float pivotY) {
    rotationPivot.x = pivotX;
    rotationPivot.y = pivotY;
}

void Shape::setColor(color c)    { fill = c; }
void Shape::setColor(vec4 c)     { fill.vec = c; }
void Shape::setColor(vec3 c)     { fill.vec = vec4(c, 1.0); }
void Shape::setRed(float r)      { fill.red = r; }
void Shape::setGreen(float g)    { fill.green = g; }
void Shape::setBlue(float b)     { fill.blue = b; }
void Shape::setOpacity(float a)  { fill.alpha = a; }

void Shape::setSize(vec2 size) { this->size = size; }
void Shape::setSizeX(float x)  { size.x = x; }
void Shape::setSizeY(float y)  { size.y = y; }

void move(vec2 deltaPos);
void moveX(float deltaWidth);
void moveY(float deltaHeight);

// Getters
vec2 Shape::getPos() const      { return pos; }
float Shape::getPosX() const    { return pos.x; }
float Shape::getPosY() const    { return pos.y; }
vec2 Shape::getSize() const     { return size; }
vec2 Shape::getVelocity() const { return velocity; }
vec3 Shape::getColor3() const   { return {fill.red, fill.green, fill.blue}; }
vec4 Shape::getColor4() const   { return fill.vec; }
float Shape::getRed() const     { return fill.red; }
float Shape::getGreen() const   { return fill.green; }
float Shape::getBlue() const    { return fill.blue; }
float Shape::getOpacity() const { return fill.alpha; }

void Shape::update(float deltaTime) {}