#ifndef GRAPHICS_ENGINE_H
#define GRAPHICS_ENGINE_H

#include <vector>
#include <memory>
#include <random>
#include <algorithm>
#include <cmath>
#include <GLFW/glfw3.h>
#include <Box2D/Box2D.h>

#include "GLDebugDraw.h"
#include "shader/shaderManager.h"
#include "font/fontRenderer.h"
#include "shapes/shape.h"
#include "shapes/rect.h"

using namespace std;
using glm::ortho, glm::mat4, glm::vec2, glm::vec3, glm::vec4, glm::length, glm::translate, glm::dot, glm::normalize, glm::radians;

/**
 * @brief The Engine class.
 * @details The Engine class is responsible for initializing the GLFW window, loading shaders, and rendering the game state.
 */
class Engine {
    private:
        // track last frame key state
        bool prevKeys[1024] = {false};

        struct PinballState {
            bool active = false;
            unsigned long score = 0;
            int ballsLeft = 3;
            bool ballLost = false;
            vector<bool> targetHit;
            int targetsHitCount = 0;
            int initialNumTargets = 4;
            bool fullReset = true;

            float ballRadius = 30.0f;
            float ballDensity = 1.0f;
            float ballMaxSpeed = 20.0f;
            float gravity = -1200.0f;
            // Table boundaries
            float leftWallX = 800.0f;
            float rightWallX = 1800.0f;
            float topWallY = 1700.0f;
            float bottomWallY = 150.0f;
            // flippers
            float flipperWidth = 180.0f;
            float flipperHeight = 20.0f;
            float leftFlipperAngle = 0.0f; // 0 means resting
            float rightFlipperAngle = 0.0f; // 0 means resting
            float flipperAngularSpeed = 1000.0f; // degrees/sec
            float flipperMaxAngle = 45.0f; // angle it goes up

            // plunger
            float plungerWidth = 60.0f;
            float plungerHeight = 20.0f;
            float plungerStroke = 200.0f;
            float plungerPullSpeed = 200.0f;
            float plungerPushSpeed = 3500.0f; // px/sec when returning
            float plungerMotorForce = 1000.0f;
            float plungerPullForce = 500.0f;
        } pbState;

        /// @brief The actual GLFW window.
        GLFWwindow* window{};

        /// @brief The width and height of the window.
        const unsigned int width = 2000, height = 1800; // Window dimensions

        /// @brief The projection matrix
        const mat4 projection = ortho(0.0f, (float)width, 0.0f, (float)height);
        /// @brief Keyboard state (True if pressed, false if not pressed).
        /// @details Index this array with GLFW_KEY_{key} to get the state of a key.
        bool keys[1024];

        /// @brief Responsible for loading and storing all the shaders used in the project.
        /// @details Initialized in initShaders()
        unique_ptr<ShaderManager> shaderManager;
        /// @brief Responsible for rendering text on the screen.
        /// @details Initialized in initShaders()
        unique_ptr<FontRenderer> fontRenderer;
        Shader shapeShader;
        Shader textShader;

        double MouseX, MouseY;
        bool mousePressedLastFrame = false;

        // Box2D physics objects
        unique_ptr<b2World> world = make_unique<b2World>(b2Vec2(0.0f, pbState.gravity / SCALE)); // Box2D world
        vec2 leftFlipperPos{800.0f, 190.0f};
        vec2 rightFlipperPos{1200.0f, 190.0f};
        b2Vec2 leftGuardCenter;
        b2Vec2 rightGuardCenter;
        b2Fixture* trapSensor = nullptr;
        vector<b2Fixture*> bumperFixtures;
        vector<b2Fixture*> targetFixtures;
        vector<b2Body*> targetsToDestroy;
        b2Body* ballBody = nullptr;
        b2Body* leftFlipperBody = nullptr;
        b2Body* rightFlipperBody = nullptr;
        b2Body* leftGuardBody  = nullptr;
        b2Body* rightGuardBody = nullptr;
        b2Body* backboardBody = nullptr;
        b2RevoluteJoint* leftFlipperJoint  = nullptr;
        b2RevoluteJoint* rightFlipperJoint = nullptr;
        b2Body* plungerBody = nullptr;
        b2PrismaticJoint* plungerJoint = nullptr;
        static constexpr float SCALE = 100.0f;
        unique_ptr<GLDebugDraw> debugDraw;

        struct TrapListener : b2ContactListener {
            Engine* engine;
            TrapListener(Engine* e) : engine(e) {}
            void BeginContact(b2Contact* c) override {
                auto A = c->GetFixtureA();
                auto B = c->GetFixtureB();
                // if A is trapSensor and B is the ball or vice versa then update state
                if ((A == engine->trapSensor && B->GetBody() == engine->ballBody) ||
                    (B == engine->trapSensor && A->GetBody() == engine->ballBody)) {
                    engine->pbState.ballLost = true;
                }
                // if ball touches bumper
                for (auto f : engine->bumperFixtures) {
                    if ((A == f && B->GetBody() == engine->ballBody) ||
                        (B == f && A->GetBody() == engine->ballBody)) {
                        engine->pbState.score += 100; // award 100 points
                    }
                }
                // if ball and target make contact then call helper function
                for (int i = 0; i < engine->targetFixtures.size(); ++i) {
                    // need fixture instance as param to call function for target handling
                    b2Fixture* tf = engine->targetFixtures[i];
                    if (!engine->pbState.targetHit[i] &&
                        ((A == tf && B->GetBody() == engine->ballBody) ||
                        (B == tf && A->GetBody() == engine->ballBody))) {
                        engine->handleTargetHit(tf);
                    }
                }
            }
            // tweak ball to plunger bounces
            void PreSolve(b2Contact* c, const b2Manifold*) override {
                b2Fixture* A = c->GetFixtureA();
                b2Fixture* B = c->GetFixtureB();
                bool hit = (A->GetBody()==engine->plungerBody && B->GetBody()==engine->ballBody)
                        || (B->GetBody()==engine->plungerBody && A->GetBody()==engine->ballBody);
                if (!hit) return;

                float ms = engine->plungerJoint->GetMotorSpeed();
                if (ms < 0.0f) {
                    c->SetRestitution(0.0f);
                } else {
                    // when returning or idle absorb most energy
                    c->SetRestitution(0.2f);
                }
                // high friction
                c->SetFriction(0.0f);
            }
        } contactListener{this};



        // Initialize Box2D bodies/fixtures
        void initPhysics();
        void spawnBall(); // spawn one ball
        void spawnTargets(bool fullReset);
        void handleTargetHit(b2Fixture* f);
        void resetGame();


    public:
        /// @brief Constructor for the Engine class.
        /// @details Initializes window and shaders.
        Engine();

        /// @brief Destructor for the Engine class.
        ~Engine();

        /// @brief Initializes the GLFW window.
        /// @return 0 if successful, -1 otherwise.
        unsigned int initWindow(bool debug = false);

        /// @brief Loads shaders from files and stores them in the shaderManager.
        /// @details Renderers are initialized here.
        void initShaders();

        /// @brief Initializes the shapes to be rendered.
        void initShapes();

        /// @brief Processes input from the user.
        /// @details (e.g. keyboard input, mouse input, etc.)
        void processInput();

        void onBallTrap(); // called when the ball hits the trap

        /// @brief Updates the game state.
        /// @details (e.g. collision detection, delta time, etc.)
        void update();

        /// @brief Renders the game state.
        /// @details Displays/renders objects on the screen.
        void render();

        /* deltaTime variables */
        float deltaTime = 0.0f; // Time between current frame and last frame
        float lastFrame = 0.0f; // Time of last frame (used to calculate deltaTime)

        /// @brief Returns true if the window should close.
        /// @details (Wrapper for glfwWindowShouldClose()).
        /// @return true if the window should close
        /// @return false if the window should not close
        bool shouldClose();

        /// Projection matrix used for 2D rendering (orthographic projection).
        /// We don't have to change this matrix since the screen size never changes.
        /// OpenGL uses the projection matrix to map the 3D scene to a 2D viewport.
        /// The projection matrix transforms coordinates in the camera space into normalized device coordinates (view space to clip space).
        // 4th quadrant
        mat4 PROJECTION = ortho(0.0f, static_cast<float>(width), 0.0f, static_cast<float>(height), -1.0f, 1.0f);
        // 1st quadrant
        //        mat4 PROJECTION = ortho(0.0f, static_cast<float>(width), 0.0f, static_cast<float>(height));

        /// @brief Debug function to check for OpenGL errors.
        GLenum glCheckError_(const char *file, int line);
        /// @brief Macro for glCheckError_ function. Used for debugging.
        #define glCheckError() glCheckError_(__FILE__, __LINE__)
};

#endif //GRAPHICS_ENGINE_H
