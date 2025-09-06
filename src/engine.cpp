#include "engine.h"
#include "GLDebugDraw.h"
#include <iostream>

/**
 * @class Engine
 * @brief engine for the pinball game.
 * Manages window creation, OpenGL setup, shader compilation, Box2D physics,
 * input processing, game updating, and rendering
 */
Engine::Engine() : keys() {
    this->initWindow();
    this->initShaders();
    this->initPhysics();
}

Engine::~Engine() = default;

unsigned int Engine::initWindow(bool debug) {
    // glfw: initialize and configure
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
#endif
    glfwWindowHint(GLFW_RESIZABLE, false);

    window = glfwCreateWindow(width, height, "engine", nullptr, nullptr);
    glfwMakeContextCurrent(window);

    // glad: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        cout << "Failed to initialize GLAD" << endl;
        return -1;
    }

    // OpenGL configuration
    glViewport(0, 0, width, height);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glfwSwapInterval(1);

    return 0;
}

void Engine::initShaders() {
    // load shader manager
    shaderManager = make_unique<ShaderManager>();
    // load shader into shader manager and retrieve it
    shapeShader = shaderManager->loadShader("../res/shaders/shape.vert", "../res/shaders/shape.frag",  nullptr, "shape");
    textShader = shaderManager->loadShader("../res/shaders/text.vert","../res/shaders/text.frag",nullptr,"text");
    fontRenderer = make_unique<FontRenderer>(shaderManager->getShader("text"), "../res/fonts/MxPlus_IBM_BIOS.ttf", 24);
    // uniforms
    shapeShader.use();
    shapeShader.setMatrix4("projection", this->PROJECTION);
}



void Engine::initPhysics() {
    // World with gravity
    // debug draw
    debugDraw = make_unique<GLDebugDraw>(SCALE);
    world->SetDebugDraw(debugDraw.get());
    world->SetContactListener(&contactListener);

    // ball body & fixture
    spawnBall();

    // walls as single static body with four thin‐box fixtures, transparent
    b2BodyDef wallDef;
    wallDef.position.Set(0.0f, 0.0f);
    b2Body* wallBody = world->CreateBody(&wallDef);

    float thickness = pbState.ballRadius / SCALE;
    float halfW_pix = (pbState.rightWallX - pbState.leftWallX) * 0.5f;
    float halfH_pix = (pbState.topWallY - pbState.bottomWallY) * 0.5f;
    float halfW = halfW_pix / SCALE;
    float halfH = halfH_pix / SCALE;

    b2PolygonShape wallBox;
    b2FixtureDef wallFix;
    wallFix.shape = &wallBox;
    wallFix.density = 0.0f;
    wallFix.restitution = 0.7f;
    wallFix.friction = 0.2f;

    // Bottom
    wallBox.SetAsBox(
        halfW, thickness,
        b2Vec2((pbState.leftWallX + pbState.rightWallX) * 0.5f / SCALE,
                (pbState.bottomWallY - pbState.ballRadius) / SCALE ),0.0f
    );
    b2Fixture* bottomWallFixture = wallBody->CreateFixture(&wallFix);
    debugDraw->SetFillColor(bottomWallFixture, b2Color(1.0f, 1.0f, 1.0f, 0.0f));
    // Top
    wallBox.SetAsBox(
        halfW, thickness,
        b2Vec2((pbState.leftWallX + pbState.rightWallX) * 0.5f / SCALE,
                (pbState.topWallY + pbState.ballRadius) / SCALE),0.0f
    );
    b2Fixture* topWallFixture = wallBody->CreateFixture(&wallFix);
    debugDraw->SetFillColor(topWallFixture, b2Color(1.0f, 1.0f, 1.0f, 0.0f));
    // Left
    wallBox.SetAsBox(
        thickness, halfH,
        b2Vec2((pbState.leftWallX - pbState.ballRadius) / SCALE,
               (pbState.bottomWallY + pbState.topWallY) * 0.5f / SCALE),0.0f
    );
    b2Fixture* leftWallFixture =  wallBody->CreateFixture(&wallFix);
    debugDraw->SetFillColor(leftWallFixture, b2Color(1.0f, 1.0f, 1.0f, 0.0f));
    // Right
    wallBox.SetAsBox(
        thickness, halfH,
        b2Vec2((pbState.rightWallX + pbState.ballRadius) / SCALE,
               (pbState.bottomWallY + pbState.topWallY) * 0.5f / SCALE),0.0f
    );
    b2Fixture* rightWallFixture = wallBody->CreateFixture(&wallFix);
    debugDraw->SetFillColor(rightWallFixture, b2Color(1.0f, 1.0f, 1.0f, 0.0f));


    // Flippers
    // angle at - 45 towards center
    pbState.leftFlipperAngle  = -pbState.flipperMaxAngle;
    pbState.rightFlipperAngle =  pbState.flipperMaxAngle;
    // screen positions
    float baseY = pbState.bottomWallY + (5.5f * pbState.flipperHeight);
    float inset = 350.0f;
    leftFlipperPos  = vec2(pbState.leftWallX  + inset - 40.0f, baseY);
    rightFlipperPos = vec2(pbState.rightWallX - inset - 40.0f, baseY);

    // lambda for making a flipper body, use bool for position differences from L/R
    auto createFlipper = [&](bool isLeft) {
        b2BodyDef def;
        def.type = b2_dynamicBody;
        float px = isLeft ? leftFlipperPos.x : rightFlipperPos.x;
        float py = leftFlipperPos.y;
        def.position.Set(px/SCALE, py/SCALE);
        def.angle = radians(isLeft ? pbState.leftFlipperAngle : pbState.rightFlipperAngle);
        b2Body* body = world->CreateBody(&def);

        b2PolygonShape box;
        box.SetAsBox(pbState.flipperWidth * 0.5f / SCALE,pbState.flipperHeight * 0.5f / SCALE);
        b2FixtureDef fix;
        fix.shape = &box;
        fix.density = 100.0f;
        fix.restitution = 0.0001f;
        fix.friction = 0.1f;
        b2Fixture* fixture = body->CreateFixture(&fix);
        body->ResetMassData();
        body->SetGravityScale(0.0f);
        if (isLeft) {
            debugDraw->SetFillColor(fixture, b2Color(1.0f, 0.0f, 0.0f));  // red
        } else {
            debugDraw->SetFillColor(fixture, b2Color(0.0f, 0.0f, 1.0f));  // blue
        }
        return body;
    };

    leftFlipperBody = createFlipper(true);
    rightFlipperBody = createFlipper(false);

    // create joints to turn flipper bodies
    b2BodyDef groundDef;
    groundDef.type = b2_staticBody;
    b2Body* ground = world->CreateBody(&groundDef);

    const float fHW = pbState.flipperWidth * 0.5f / SCALE;
    const float fHH = pbState.flipperHeight * 0.5f / SCALE;
    b2Vec2 leftPivot = leftFlipperBody->GetWorldPoint(b2Vec2(-fHW, fHH));
    b2Vec2 rightPivot = rightFlipperBody->GetWorldPoint(b2Vec2(fHW, fHH));

    auto createJoint = [&](b2Body* fl, const b2Vec2& pivot, bool isLeft) {
        b2RevoluteJointDef jd;
        jd.Initialize(ground, fl, pivot);
        jd.collideConnected = false;
        jd.enableLimit = true;
        float limit = radians(pbState.flipperMaxAngle);
        if (isLeft) {
            jd.lowerAngle = 0.0f;
            jd.upperAngle = 2.0f * limit;
        } else {
            jd.lowerAngle = -2.0f * limit;
            jd.upperAngle = 0.0f;
        }
        jd.enableMotor = true;
        jd.maxMotorTorque = 20000.0f;
        return (b2RevoluteJoint*)world->CreateJoint(&jd);
    };
    leftFlipperJoint = createJoint(leftFlipperBody, leftPivot, true);
    rightFlipperJoint = createJoint(rightFlipperBody, rightPivot, false);


    // Guards and edge pieces
    // static body for all guard edges
    b2BodyDef guardDef;
    guardDef.type = b2_staticBody;
    guardDef.position.Set(0.0f, 0.0f);
    b2Body* guardBody = world->CreateBody(&guardDef);

    float guardW = pbState.flipperWidth  * 1.2f;
    float guardH = pbState.flipperHeight * 1.2f;

    // LEFT GUARD
    {
        // flipper rest angle
        float angleL = radians(-45.0f);
        // tip of the flipper in world coords
        vec2 tipPx = leftFlipperPos + vec2(-pbState.flipperWidth * 0.5f, pbState.flipperHeight * 0.5f);
        b2Vec2 tipWorld(tipPx.x / SCALE, tipPx.y / SCALE);

        // push guard rectangle outward along flippers long axis
        // offset to set exact position next to flippers
        b2Vec2 offset(
            cos(angleL) * -((pbState.flipperWidth * 0.43f) / SCALE),
            sin(angleL) * -((pbState.flipperWidth * 0.5f + guardW * 0.44f) / SCALE)
        );
        leftGuardCenter = tipWorld + offset;

        // create and position a box around that center rotated to match the flipper
        b2PolygonShape guardRect;
        guardRect.SetAsBox(guardW * 0.5f / SCALE,guardH * 0.5f / SCALE, leftGuardCenter, angleL);
        // static
        guardBody->CreateFixture(&guardRect, 0.0f);
    }
    // RIGHT GUARD
    {
        // flipper rest angle in radians
        float angleL = radians(45.0f);
        // tip of the flipper in world coords
        vec2 tipPx = rightFlipperPos + vec2(pbState.flipperWidth * 0.5f, pbState.flipperHeight * 0.5f);
        b2Vec2 tipWorld(tipPx.x / SCALE, tipPx.y / SCALE);
        // push guard rectangle outward along the flippers long axis
        b2Vec2 offset(
            cos(angleL) * ((pbState.flipperWidth * 0.43f) / SCALE),
            sin(angleL) * ((pbState.flipperWidth * 0.5f + guardW * 0.44f) / SCALE)
        );
        rightGuardCenter = tipWorld + offset;
        // position box around center rotated to match flipper
        b2PolygonShape guardRect;
        guardRect.SetAsBox(guardW * 0.5f / SCALE,guardH * 0.5f / SCALE, rightGuardCenter, angleL);
        // static
        guardBody->CreateFixture(&guardRect, 0.0f);
    }

    // LEFT TRAPEZOID GUARD
    {
        float angleL = radians(pbState.leftFlipperAngle);
        b2Vec2 center = leftGuardCenter;
        // half sizes in world coords
        float hx = guardW * 0.5f / SCALE;
        float hy = guardH * 0.5f / SCALE;
        // rotation
        b2Rot rotL(angleL);
        // bottom left (pBL) and top left (pTL) of the guard
        b2Vec2 pBL = b2Mul(rotL, b2Vec2(-hx, -hy)) + center;
        b2Vec2 pTL = b2Mul(rotL, b2Vec2(-hx, +hy)) + center;
        // build the flat top
        float yTop = (pbState.topWallY - 900.0f) / SCALE;
        b2Vec2 topR(pTL.x, yTop);
        b2Vec2 topL(pBL.x, yTop);

        // create the convex quad
        b2PolygonShape trapezoid;
        b2Vec2 verts[4] = {pBL, pTL, topR, topL};
        trapezoid.Set(verts, 4);
        guardBody->CreateFixture(&trapezoid, 0.0f);
    }
    // RIGHT TRAPEZOID GUARD
    {
        float angleR = radians(pbState.rightFlipperAngle);
        b2Vec2 center = rightGuardCenter;
        // half sizes in world coords
        float hx = guardW * 0.5f / SCALE;
        float hy = guardH * 0.5f / SCALE;
        // rotation
        b2Rot rotR(angleR);
        // bottom right (pBR) and top‐right (pTR) of the guard rectangle
        b2Vec2 pBR = b2Mul(rotR, b2Vec2(+hx, -hy)) + center;
        b2Vec2 pTR = b2Mul(rotR, b2Vec2(+hx, +hy)) + center;
        // flat roof at the same y as the left trapezoid
        float yTop = (pbState.topWallY - 900.0f) / SCALE;
        b2Vec2 topR(pTR.x, yTop);
        b2Vec2 topL(pBR.x, yTop);

        // build convex quad
        b2PolygonShape trapezoid;
        b2Vec2 verts[4] = {pBR, pTR, topR, topL};
        trapezoid.Set(verts, 4);
        guardBody->CreateFixture(&trapezoid, 0.0f);
    }

    // compute the trap center
    float trapX = ((leftFlipperPos.x + rightFlipperPos.x) * 0.5f) / SCALE;
    float trapY = (pbState.bottomWallY) / SCALE;
    b2Vec2 trapPtL(trapX - 2.0f, trapY);
    b2Vec2 trapPtR(trapX + 2.0f, trapY);

    b2Vec2 leftWallPt(pbState.leftWallX  / SCALE, (pbState.bottomWallY + 250.0f) / SCALE);
    b2Vec2 rightWallPt((pbState.rightWallX - 80.0f ) / SCALE, (pbState.bottomWallY + 250.0f) / SCALE);

    // bottom left and bottom right wall points
    b2Vec2 leftTriBase(pbState.leftWallX  / SCALE, pbState.bottomWallY / SCALE);
    b2Vec2 rightTriBase((pbState.rightWallX - 80.0f) / SCALE, pbState.bottomWallY / SCALE);

    // LEFT corner triangle
    {
        b2Vec2 tri[3] = {
            leftWallPt,
            leftTriBase,
            trapPtL
        };
        b2PolygonShape shape;
        shape.Set(tri, 3);
        b2Fixture* f = guardBody->CreateFixture(&shape, 0.0f);
        debugDraw->SetFillColor(f, b2Color(0.2f, 0.8f, 0.6f, 1.0f));
    }

    // RIGHT corner triangle
    {
        b2Vec2 tri[3] = {
            trapPtR,
            rightTriBase,
            rightWallPt
        };
        b2PolygonShape shape;
        shape.Set(tri, 3);
        b2Fixture* f = guardBody->CreateFixture(&shape, 0.0f);
        debugDraw->SetFillColor(f, b2Color(0.2f, 0.8f, 0.6f, 1.0f));
    }


    // spring shoot funnel
    {
        const b2Vec2 RFpts[] = {
            // start at bottom right to form shoot tunnel
            b2Vec2((pbState.rightWallX - 80.0f) / SCALE,(pbState.bottomWallY) / SCALE),
            // top
            b2Vec2((pbState.rightWallX - 80.0f) / SCALE,(pbState.topWallY - 250.0f) / SCALE),
            // back to start
            b2Vec2((pbState.rightWallX - 80.0f) / SCALE,(pbState.bottomWallY) / SCALE)
        };
        b2ChainShape shootFunnel;
        shootFunnel.CreateChain(RFpts, 3, RFpts[0], RFpts[2]);
        guardBody->CreateFixture(&shootFunnel, 0.0f);
    }
    // shoot plunger and spring joint
    {
        // dynamic body for plunger
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        // center it between
        float px = pbState.rightWallX - 40.0f;
        float py = pbState.bottomWallY + 200.0f;
        bd.position.Set(px / SCALE, py / SCALE);
        bd.fixedRotation = true; // keep upright
        plungerBody = world->CreateBody(&bd);
        plungerBody->SetGravityScale(0.0f); // turn off gravity

        // fixture
        b2PolygonShape box;
        box.SetAsBox((pbState.plungerWidth  * 0.5f) / SCALE,(pbState.plungerHeight * 0.5f) / SCALE);
        b2FixtureDef fd;
        fd.shape = &box;
        fd.density = 1.0f;
        fd.friction = 0.1f;
        plungerBody->CreateFixture(&fd);
        debugDraw->SetFillColor(plungerBody->GetFixtureList(), b2Color(1.0f,0.2f,0.1f));

        // static ground to anchor slider
        b2BodyDef staticDef;
        staticDef.type = b2_staticBody;
        b2Body* sliderGround = world->CreateBody(&staticDef);
        // prismatic joint vertical slider with motor and limits
        b2PrismaticJointDef pjd;
        b2Vec2 worldAnchor = plungerBody->GetWorldCenter();
        // Initialize prismatic joint at anchor, vertical axis
        pjd.Initialize(sliderGround, plungerBody, worldAnchor, b2Vec2(0,1));
        pjd.collideConnected = false;

        // pull down by plungerStroke px
        pjd.enableLimit = true;
        pjd.lowerTranslation = -pbState.plungerStroke / SCALE;
        pjd.upperTranslation = 0.0f;
        pjd.enableMotor = true;
        pjd.maxMotorForce = pbState.plungerMotorForce; // force to hold a ball
        pjd.motorSpeed = 0.0f; // start idle
        plungerJoint = (b2PrismaticJoint*)world->CreateJoint(&pjd);
    }

    // center bottom ball pit sensor
    {
        float trapWidth = 200.0f / SCALE;
        float trapHeight = 10.0f / SCALE;
        // centered between leftWallX & rightWallX
        b2BodyDef bd;
        bd.position.Set((leftFlipperPos.x + rightFlipperPos.x) * 0.5f / SCALE,(pbState.bottomWallY + 1 * 0.5f) / SCALE);
        b2Body* trapBody = world->CreateBody(&bd);

        b2PolygonShape box;
        box.SetAsBox(trapWidth, trapHeight);
        b2FixtureDef fd;
        fd.shape = &box;
        fd.isSensor = true;
        trapSensor = trapBody->CreateFixture(&fd);
        debugDraw->SetFillColor(trapSensor, b2Color(0.0f, 0.0f, 0.0f, 1.0f));
    }


    // top of board curved guard
    // sample half circle points
    const int OV_VERTS = 16;
    vector<b2Vec2> oval(OV_VERTS);
    float centerX = (pbState.leftWallX + pbState.rightWallX) * 0.5f / SCALE;
    float ovalbaseY = (pbState.topWallY - 250.0f) / SCALE;
    float radY = 250.0f / SCALE;

    // rebuild oval points
    for (int i = 0; i < OV_VERTS; ++i) {
        float t = float(i) / float(OV_VERTS - 1);
        float theta = M_PI * (1.0f - t);
        oval[i].Set(
            centerX + cos(theta) * halfW,
            ovalbaseY + sin(theta) * radY
        );
    }
    // half ellipse chain
    b2ChainShape topCurve;
    topCurve.CreateChain(oval.data(), OV_VERTS, oval.front(), oval.back());
    guardBody->CreateFixture(&topCurve, 0.0f);
    // helper line segment to prevent re-entry to shoot funnel
    // can enter machine from the shoot tunnel, but acts as barrier when hit from other side
    // top of shoot funnel
    b2Vec2 A((pbState.rightWallX - 80.0f) / SCALE,(pbState.topWallY - 250.0f) / SCALE);
    // the very rightmost point on half circle
    b2Vec2 B = oval[OV_VERTS - 2];

    // Ghosts for one sided normals
    b2Vec2 prev((pbState.rightWallX - 80.0f) / SCALE, pbState.bottomWallY / SCALE);
    b2Vec2 next = oval[OV_VERTS - 2];

    b2ChainShape blocker;
    b2Vec2 seg[2] = {B, A};
    blocker.CreateChain(seg, 2, next, prev);
    b2Fixture* blockerF = guardBody->CreateFixture(&blocker, 0.0f);
    debugDraw->SetFillColor(blockerF, b2Color(0.2f, 0.8f, 0.6f, 1.0f));


    // Fill shape of oval curve at top
    // coordinates of the two wall corners at the very top
    b2Vec2 leftTop(pbState.leftWallX  / SCALE, pbState.topWallY / SCALE);
    b2Vec2 rightTop(pbState.rightWallX / SCALE, pbState.topWallY / SCALE);

    // index to split left vs right halves
    int mid = OV_VERTS / 2;
    // LEFT half-fan anchor at left wall corner
    for (int i = 0; i < mid; ++i) {
        b2Vec2 tri[3] = {
            leftTop,
            oval[i],
            oval[i+1]
        };
        b2PolygonShape shape;
        shape.Set(tri, 3);
        b2Fixture* f = guardBody->CreateFixture(&shape, 0.0f);
        debugDraw->SetFillColor(f, b2Color(0.2f, 0.8f, 0.6f, 1.0f));
    }

    // RIGHT half-fan anchor at right wall corner
    for (int i = mid; i < OV_VERTS - 1; ++i) {
        b2Vec2 tri[3] = {
            rightTop,
            oval[i+1],
            oval[i]
        };
        b2PolygonShape shape;
        shape.Set(tri, 3);
        b2Fixture* f = guardBody->CreateFixture(&shape, 0.0f);
        debugDraw->SetFillColor(f, b2Color(0.2f, 0.8f, 0.6f, 1.0f));
    }


    // SCORING BUMPERS 3 pink circles at mid right
    vector<vec2> bumperPositions = {
        { pbState.rightWallX - 200.0f, pbState.topWallY - 700.0f},
        { pbState.rightWallX - 320.0f, pbState.topWallY - 550.0f},
        { pbState.rightWallX - 400.0f, pbState.topWallY - 750.0f}
    };

    for (auto& pos : bumperPositions) {
        // static body for bumper
        b2BodyDef bd;
        bd.position.Set(pos.x / SCALE, pos.y / SCALE);
        b2Body* bump = world->CreateBody(&bd);

        // circle shape
        b2CircleShape circle;
        circle.m_radius = (pbState.ballRadius * 1.5f) / SCALE;
        b2FixtureDef fd;
        fd.shape = &circle;
        fd.density = 0.0f;
        fd.restitution = 1.2f; // bouncy
        fd.friction = 0.0f;
        b2Fixture* fx = bump->CreateFixture(&fd);

        // pink/purple
        debugDraw->SetFillColor(fx, b2Color(1.0f, 0.0f, 1.0f));
        // remember fixture for scoring
        bumperFixtures.push_back(fx);
    }

    // SCORING TARGET BOARD 4 box targets
    {
        // backboard parameters
        float boardW = 350.0f;
        float boardH = 20.0f;
        float boardTilt = 15.0f;
        float boardX = (pbState.leftWallX + pbState.rightWallX) * 0.5f - 175.0f;
        float boardY = pbState.topWallY - 400.0f;
        // backboard body
        b2BodyDef bbDef;
        bbDef.type = b2_staticBody;
        bbDef.position.Set(boardX / SCALE, boardY / SCALE);
        bbDef.angle = radians(boardTilt);
        backboardBody = world->CreateBody(&bbDef);
        b2PolygonShape bbBox;
        bbBox.SetAsBox((boardW * 0.5f) / SCALE, (boardH * 0.5f) / SCALE);
        backboardBody->CreateFixture(&bbBox, 0.0f);

        spawnTargets(true);
    }


    // TRIANGLE BUMPERS above flippers
    // compute the trap center
    centerX = ((leftFlipperPos.x + rightFlipperPos.x) * 0.5f) / SCALE;
    b2Vec2 triBumpPtL(centerX - 2.0f, (pbState.bottomWallY + 280.0f) / SCALE);
    b2Vec2 triBumpPtR(centerX + 2.0f, (pbState.bottomWallY + 280.0f) / SCALE);

    // top left and top right
    b2Vec2 triBumpPtTL ((pbState.leftWallX + 170.0f) / SCALE, (pbState.topWallY - 1000.0f) / SCALE);
    b2Vec2 triBumpPtTR((pbState.rightWallX - 250.0f) / SCALE, (pbState.topWallY - 1000.0f) / SCALE);

    // the points directly below each flipper
    b2Vec2 triBumpBaseL ((pbState.leftWallX + 170.0f) / SCALE, (pbState.bottomWallY  + 370.0f) / SCALE);
    b2Vec2 triBumpBaseR((pbState.rightWallX - 250.0f) / SCALE, (pbState.bottomWallY + 370.0f) / SCALE);

    // LEFT triangle
    {
        b2Vec2 tri[3] = {
            triBumpPtTL,
            triBumpPtL,
            triBumpBaseL
        };
        b2PolygonShape shape;
        shape.Set(tri, 3);
        b2FixtureDef bumperDef;
        bumperDef.shape = &shape;
        bumperDef.density = 0.0f;
        bumperDef.restitution = 0.8f;
        bumperDef.friction = 0.2f;
        b2Fixture* f = guardBody->CreateFixture(&bumperDef);
        debugDraw->SetFillColor(f, b2Color(0.2f, 0.8f, 0.6f, 1.0f));
    }

    // RIGHT triangle
    {
        b2Vec2 tri[3] = {
            triBumpPtTR,
            triBumpPtR,
            triBumpBaseR
        };
        b2PolygonShape shape;
        shape.Set(tri, 3);
        b2FixtureDef bumperDef;
        bumperDef.shape = &shape;
        bumperDef.density = 0.0f;
        bumperDef.restitution = 1.1f;
        bumperDef.friction = 0.0f;
        b2Fixture* f = guardBody->CreateFixture(&bumperDef);
        debugDraw->SetFillColor(f, b2Color(0.2f, 0.8f, 0.6f, 1.0f));
    }
}

void Engine::spawnBall() {
    b2BodyDef ballDef;
    ballDef.type = b2_dynamicBody;
    ballDef.position.Set(
        1750.0f / SCALE,
        700.0f / SCALE
    );
    ballBody = world->CreateBody(&ballDef);
    ballBody->SetBullet(true);

    b2CircleShape ballShape;
    ballShape.m_radius = pbState.ballRadius / SCALE;
    b2FixtureDef ballFix;
    ballFix.shape = &ballShape;
    ballFix.density = pbState.ballDensity;
    ballFix.restitution = 0.7f;
    ballFix.friction = 0.3f;
    b2Fixture* ballF = ballBody->CreateFixture(&ballFix);
    ballBody->ResetMassData();
    debugDraw->SetFillColor(ballF, b2Color(1.0f, 0.9f, 0.0f)); // bright yellow
}

void Engine::spawnTargets(bool fullReset) {
    // destroy leftover bodies
    if (fullReset) {
        // all old target bodies
        for (b2Fixture* f : targetFixtures) {
            // make sure body still exists
            if (f && f->GetBody()) {
                world->DestroyBody(f->GetBody());
            }
        }
    } else {
        // ones queued for destruction
        for (b2Body* b : targetsToDestroy) {
            world->DestroyBody(b);
        }
    }

    // clear both lists
    targetsToDestroy.clear();
    targetFixtures.clear();
    pbState.targetHit.clear();
    pbState.targetsHitCount = 0;

    for (int i = 0; i < pbState.initialNumTargets; ++i) {
        float boardTilt = 15.0f;
        float boardH = 20.0f;
        float spacing = 20.0f;
        float targetH = 30.0f;
        float targetW = 60.0f;
        // X offset from board center
        float xOffset = (i - (pbState.initialNumTargets - 1) / 2.0f) * (targetW + spacing);
        // Y offset to left
        float yOffset = -(boardH * 0.5f + targetH * 0.5f + 5.0f);

        b2Vec2 worldPos = backboardBody->GetWorldPoint(
            b2Vec2(xOffset / SCALE, yOffset / SCALE)
        );
        b2BodyDef tDef;
        tDef.type = b2_staticBody;
        tDef.position = worldPos;
        tDef.angle = radians(boardTilt);
        b2Body* tBody = world->CreateBody(&tDef);

        b2PolygonShape tBox;
        tBox.SetAsBox((targetW * 0.5f) / SCALE, (targetH * 0.5f) / SCALE);
        b2Fixture* tFix = tBody->CreateFixture(&tBox, 0.0f);

        debugDraw->SetFillColor(tFix, b2Color(1.0f,0.5f,0.0f,1.0f));
        targetFixtures.push_back(tFix);
        pbState.targetHit.push_back(false);
    }
}


void Engine::handleTargetHit(b2Fixture* f) {
    // mark it and despawn
    for (int i = 0; i < targetFixtures.size(); ++i) {
        if (targetFixtures[i] == f && !pbState.targetHit[i]) {
            pbState.targetHit[i] = true;
            // hold off destruction until after Step
            targetsToDestroy.push_back(f->GetBody());
            pbState.targetsHitCount++;
            break;
        }
    }
}

void Engine::resetGame() {
    // clear ball
    if (ballBody) {
        world->DestroyBody(ballBody);
        ballBody = nullptr;
    }
    // reset counters
    pbState.score = 0;
    pbState.ballsLeft = 3;
    pbState.active = true;
    // respawn targets and new ball
    spawnTargets(true);
    spawnBall();
}


void Engine::processInput() {
    static float lastTime = glfwGetTime();
    float now = glfwGetTime();
    deltaTime = now - lastTime;
    lastTime = now;
    glfwPollEvents();
    for(int i = 0; i < 1024; ++i) {
        keys[i] = (glfwGetKey(window, i) == GLFW_PRESS);
    }

    static bool prevEsc = false;
    bool curEsc = keys[GLFW_KEY_ESCAPE];
    if(curEsc && !prevEsc) {
        glfwSetWindowShouldClose(window,true);
    }
    prevEsc = curEsc;

    // offer restart
    if (keys[GLFW_KEY_P] && !prevKeys[GLFW_KEY_P]) {
        resetGame();
    }

    float speed = radians(pbState.flipperAngularSpeed);
    // Left flipper
    float la = leftFlipperJoint->GetJointAngle();
    if (keys[GLFW_KEY_A]) {
        // only push up until upper limit
        if (la < leftFlipperJoint->GetUpperLimit()) {
            leftFlipperJoint->SetMotorSpeed(speed);
        } else {
        leftFlipperJoint->SetMotorSpeed(0.0f);
        }
    } else {
        // push down until lower limit
        if (la > leftFlipperJoint->GetLowerLimit()) {
            leftFlipperJoint->SetMotorSpeed(-speed);
        } else {
            leftFlipperJoint->SetMotorSpeed(0.0f);
        }
    }

    // Right flipper
    float ra = rightFlipperJoint->GetJointAngle();
    if (keys[GLFW_KEY_D]) {
        // push until lower limit –maxAngle
        if (ra > rightFlipperJoint->GetLowerLimit()) {
            rightFlipperJoint->SetMotorSpeed(-speed);
        } else {
            rightFlipperJoint->SetMotorSpeed(0.0f);
        }
    } else {
        // push back to rest
        if (ra < rightFlipperJoint->GetUpperLimit()) {
            rightFlipperJoint->SetMotorSpeed(speed);
        } else {
            rightFlipperJoint->SetMotorSpeed(0.0f);
        }
    }

    if (keys[GLFW_KEY_SPACE]) {
        // pull down slowly
        plungerJoint->SetMotorSpeed(-pbState.plungerPullSpeed / SCALE);
    } else {
        // push back up quickly
        plungerJoint->SetMotorSpeed( pbState.plungerPushSpeed / SCALE);
    }
}

void Engine::onBallTrap() {
    // remove old ball body
    if (ballBody) {
        world->DestroyBody(ballBody);
        ballBody = nullptr;
    }

    // decrement remaining balls
    pbState.ballsLeft--;

    if (pbState.ballsLeft > 0) {
        // reset targets for each ball
        spawnTargets(true);
        // next ball
        spawnBall();
    } else {
        // all balls used so round over
        pbState.active = false;
    }
}



void Engine::update() {
    const float ts = 1.0f / 60.0f;
    static float acc = 0;
    acc += deltaTime;
    while(acc >= ts) {
        world->Step(ts,8,3);
        acc -= ts;
    }

    if (pbState.targetsHitCount >= pbState.initialNumTargets) {
        // all were hit so destroy and respawn
        pbState.score += 1000;
        spawnTargets(false);
    } else {
        // some targets were hit so destroy those bodies
        for (b2Body* b : targetsToDestroy) {
            // find the fixture for this body
            auto it = find_if(
                targetFixtures.begin(), targetFixtures.end(),
                [&](b2Fixture* f){ return f->GetBody() == b; }
            );
            if (it != targetFixtures.end()) {
                int idx = it - targetFixtures.begin();
                targetFixtures.erase(it);
                pbState.targetHit.erase(pbState.targetHit.begin() + idx);
            }
            world->DestroyBody(b);
        }
        targetsToDestroy.clear();
    }

    if (ballBody) {
        b2Vec2 v = ballBody->GetLinearVelocity();
        float speed = v.Length();
        if (speed > pbState.ballMaxSpeed) {
            v *= (pbState.ballMaxSpeed / speed);
            ballBody->SetLinearVelocity(v);
        }
    }
    // safe to modify b2world after stepping without lock
    if (pbState.ballLost) {
        pbState.ballLost = false;
        onBallTrap(); // destroy old ballBody, decrement ballsLeft, spawnBall() if any remain
    }
}

void Engine::render() {
    glClearColor(0.1f, 0.3f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    shapeShader.use();
    // Machine background
    color machineColor(0.0f, 0.2f, 0.0f, 1.0f);
    Rect machineBg(shapeShader,
        vec2((pbState.leftWallX + pbState.rightWallX) / 2.0f,
                  (pbState.bottomWallY + pbState.topWallY) / 2.0f),
        vec2(pbState.rightWallX - pbState.leftWallX,
                  pbState.topWallY - pbState.bottomWallY),
        machineColor
    );
    shapeShader.setBool("useTexture", false);
    machineBg.setUniforms();
    machineBg.draw();

    // text instructions
    fontRenderer->renderText(
        "Press A/D for flippers",
        50.0f, height - 100.0f, projection,1.2f,vec3(1.0f, 1.0f, 1.0f)
    );
    fontRenderer->renderText(
        "ESC to quit",
        50.0f, height - 150.0f, projection,1.2f,vec3(1.0f, 1.0f, 1.0f)
    );
    fontRenderer->renderText(
        "Score: " + to_string(pbState.score),
        50.0f, height - 200.0f, projection,1.2f,vec3(1.0f, 1.0f, 1.0f)
    );
    fontRenderer->renderText(
        "Remaining balls: " + to_string(pbState.ballsLeft),
        50.0f, height - 250.0f, projection,1.2f,vec3(1.0f, 1.0f, 1.0f)
    );
    fontRenderer->renderText(
        "Press (p) to restart",
        50.0f, height - 300.0f, projection,1.2f,vec3(1.0f, 1.0f, 1.0f)
    );


    // for debug draw of box2d objects
    glUseProgram(0);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(value_ptr(projection));
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    debugDraw->DrawDebugData(world.get());

    glfwSwapBuffers(window);
}


bool Engine::shouldClose() {
    return glfwWindowShouldClose(window);
}