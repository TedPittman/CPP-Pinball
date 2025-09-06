#ifndef GLDEBUGDRAW_H
#define GLDEBUGDRAW_H

#include <Box2D/Box2D.h>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <unordered_map>
using namespace std;

// GLDebugDraw: a Box2D debug draw implementation using OpenGL fixed-function pipeline
// draws wireframe outlines and filled shapes
// converts Box2D's meters to pixels via pixelsPerMeter scale
class GLDebugDraw : public b2Draw {
public:
    // specify scale for drawing
    explicit GLDebugDraw(float ppm) : pixelsPerMeter(ppm) {
        SetFlags(e_shapeBit);
    }

    // allow user to pick fill color for any fixture:
    void SetFillColor(const b2Fixture* f, const b2Color& c) {
        m_fillColors[f] = c;
    }
    void SetOutlineColor(const b2Fixture* f, const b2Color& c) {
        m_outlineColors[f] = c;
    }

    // instead of world->DrawDebugData() so that we can set color fill
    void DrawDebugData(b2World* world) {
        for (b2Body* b = world->GetBodyList(); b; b = b->GetNext()) {
            for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()) {
                b2Shape* s = f->GetShape();
                // look up fill color (default to gray/green if none set)
                b2Color fill;
                auto it = m_fillColors.find(f);
                if (it != m_fillColors.end()) {
                    fill = it->second;
                } else {
                    // use Box2D default dynamic/static color:
                    // green for dynamic, gray for static
                    fill = (b->GetType()==b2_dynamicBody)
                           ? b2Color(0.3f,1.0f,0.3f, 1.0f)   // green
                           : b2Color(0.9f,0.9f,0.9f, 1.0f);  // gray
                }

                switch (s->GetType()) {
                    case b2Shape::e_polygon: {
                        auto poly = static_cast<b2PolygonShape*>(s);
                        b2Transform xf = f->GetBody()->GetTransform();
                        // transform each local vertex into world space
                        vector<b2Vec2> worldVerts(poly->m_count);
                        for(int i = 0; i < poly->m_count; ++i) {
                            worldVerts[i] = b2Mul(xf, poly->m_vertices[i]);
                        }
                        DrawSolidPolygon(worldVerts.data(), poly->m_count, fill);
                        break;
                    }
                    case b2Shape::e_circle: {
                        auto circ = static_cast<b2CircleShape*>(s);
                        b2Transform xf = f->GetBody()->GetTransform();
                        // circ->m_p is the circle center in body local coords
                        b2Vec2 worldCenter = b2Mul(xf, circ->m_p);
                        DrawSolidCircle(worldCenter, circ->m_radius, b2Vec2(), fill);
                        break;
                    }
                    case b2Shape::e_chain: {
                        // draw each edge of the chain
                        auto chain = static_cast<b2ChainShape*>(s);
                        b2Transform xf = f->GetBody()->GetTransform();
                        for (int32 i = 0; i < chain->m_count - 1; ++i) {
                            b2Vec2 v1 = b2Mul(xf, chain->m_vertices[i]);
                            b2Vec2 v2 = b2Mul(xf, chain->m_vertices[i+1]);
                            DrawSegment(v1, v2, fill);
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
        }
    }

    // Draw an unfilled polygon outline
    void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override {
        glBegin(GL_LINE_LOOP); // start line loop
        glColor4f(color.r, color.g, color.b, color.a); // set outline color
        for(int32 i = 0; i < vertexCount; ++i) {
            // convert Box2D coordinates to screen pixels
            glVertex2f(vertices[i].x * pixelsPerMeter,
                       vertices[i].y * pixelsPerMeter);
        }
        glEnd();
    }

    // draw and fill a convex polygon
    void DrawSolidPolygon(const b2Vec2* vertices, int32 count, const b2Color& color) override {
        // fill interior with color
        glColor4f(color.r, color.g, color.b, color.a);
        glBegin(GL_TRIANGLE_FAN); // triangle fan for convex fill
        for(int32 i = 0; i < count; ++i) {
            glVertex2f(vertices[i].x * pixelsPerMeter,
                       vertices[i].y * pixelsPerMeter);
        }
        glEnd();

        // outline for edges
        glColor4f(color.r, color.g, color.b, color.a);
        glBegin(GL_LINE_LOOP);
        for(int32 i = 0; i < count; ++i) {
            glVertex2f(vertices[i].x * pixelsPerMeter,
                       vertices[i].y * pixelsPerMeter);
        }
        glEnd();
    }

    // draw a circle outline with segments
    void DrawCircle(const b2Vec2& center, float radius, const b2Color& color) override {
        constexpr int SEGMENTS = 20;
        glBegin(GL_LINE_LOOP);
        glColor4f(color.r, color.g, color.b, color.a);
        for(int i = 0; i < SEGMENTS; ++i) {
            float theta = 2.0f * M_PI * float(i) / SEGMENTS;
            float x = center.x + cosf(theta) * radius;
            float y = center.y + sinf(theta) * radius;
            glVertex2f(x * pixelsPerMeter, y * pixelsPerMeter);
        }
        glEnd();
    }

    // draw and fill a circle
    void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2&, const b2Color& color) override {
        constexpr int SEGMENTS = 20;
        // fill
        glColor4f(color.r, color.g, color.b, color.a);
        glBegin(GL_TRIANGLE_FAN);
        for(int i = 0; i <= SEGMENTS; ++i) {
            float theta = 2.0f * M_PI * float(i) / SEGMENTS;
            float x = center.x + cosf(theta) * radius;
            float y = center.y + sinf(theta) * radius;
            glVertex2f(x * pixelsPerMeter, y * pixelsPerMeter);
        }
        glEnd();
        // outline
        glColor4f(color.r, color.g, color.b, color.a);
        glBegin(GL_LINE_LOOP);
        for(int i = 0; i <= SEGMENTS; ++i) {
            float theta = 2.0f * M_PI * float(i) / SEGMENTS;
            float x = center.x + cosf(theta) * radius;
            float y = center.y + sinf(theta) * radius;
            glVertex2f(x * pixelsPerMeter, y * pixelsPerMeter);
        }
        glEnd();
    }

    // draw line segment between two points
    void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) override {
        glBegin(GL_LINES);
        glColor4f(color.r, color.g, color.b, color.a);
        glVertex2f(p1.x * pixelsPerMeter, p1.y * pixelsPerMeter);
        glVertex2f(p2.x * pixelsPerMeter, p2.y * pixelsPerMeter);
        glEnd();
    }

    void DrawTransform(const b2Transform& xf) override {}
    void DrawPoint(const b2Vec2& p, float size, const b2Color& color) override {}

private:
    float pixelsPerMeter; // conversion factor from Box2D meters to screen pixels
    unordered_map<const b2Fixture*, b2Color> m_fillColors;
    unordered_map<const b2Fixture*, b2Color> m_outlineColors;
    const b2Fixture* m_currentFixture = nullptr;
};


#endif //GLDEBUGDRAW_H
