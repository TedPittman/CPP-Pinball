#include "engine.h"

int main() {
    Engine engine;

    while (!engine.shouldClose()) {
        engine.processInput();
        engine.update();
        engine.render();
    }
    glfwTerminate();
    return 0;
}
