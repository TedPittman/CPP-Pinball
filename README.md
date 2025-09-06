# Pinball
## Ted Pittman
## Project Summary
This project is a Pinball machine game where a user has three balls available per round, and attempts to get as high a score as possible by controlling the flippers to hit the targets and bouncers.
* The engine class handles the full gameplay and graphics rendering using OpenGL and Box2D physics engine. OpenGL does the active rendering, and an overridden debugger has been implemented to directly draw Box2D objects.
This allows for outline drawing, solid fill, and color change for both the outline and fill. This is the main method of drawing shapes, other than the Rect class inheriting from Shape to draw a visual table background also using OpenGL.
* On running the program, the game has already started and a ball is rendered in to be played. A user may restart at any time by pressing (p), resetting points and balls left. The game instructions are also rendered in the window,
but (space) activates the plunger to be slowly pulled down, and pushes the ball into the machine on release. The (a) and (d) keys activate the left and right flippers respectively, and must be strategically activated to keep the ball up to get points.
Each time the ball makes contact with one of the pink bouncers, 100 points are earned. When all 4 targets are hit, 1000 points are earned, with the targets resetting after all have been hit from one ball instance or on ball death.
* This program has only been executed on Windows 10 x64

## added installations / libraries
used freetype for text rendering, GLAD/GLFW/GLM for graphics rendering and manipulation, and Box2D to handle physics interaction and combine with OpenGL rendering, all included in [CMakeLists.txt](CMakeLists.txt)

## Outside code
Below are explanations of concepts and outside code, with contexts

I thank Lisa Dion of UVM CS 2300 who developed the basis of the OpenGL graphics engine structure seen in this project, along with shaders

https://medium.com/@aghajari/shading-the-canvas-a-beginners-guide-to-vertex-and-fragment-shaders-f2a0b446294f
for help with understanding .frag and .vert

https://learnopengl.com/In-Practice/2D-Game/Collisions/Collision-detection
understanding world position and standard OpenGL collision and restitution

https://www.iforce2d.net/b2dtut/debug-draw
for understanding existing DebugDraw (b2Draw) for box2d 

https://box2d.org/documentation/md_collision.html
for understanding function implementation, syntax, and engine logic 




# Program view
![image](https://github.com/user-attachments/assets/905fdd58-caae-4926-ae4e-d5e31140c60e)
