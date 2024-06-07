#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. OPTIONAL: Animate proppelers (extra points)
 * #### TODO: Audio: sound separately for each ear?, pitch is changing 
 * #### TODO: Connect speed of propellers to propeller animation
 * #### TODO: Rotate propellers in XY axis?
 */

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    Sint16 q_x, q_y, q_theta;
    double q_speedL, q_speedP; // Speed of propellers, for sound and speed of animation

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];


    bool DEBUG = true;
    if (DEBUG) {
        q_x = 100;
        q_y = 100;
        q_speedL = 10;
        q_speedP = 10;
        q_theta = 6;
    }

    // Rotation from https://en.wikipedia.org/wiki/Rotation_matrix and https://stackoverflow.com/questions/644378/drawing-a-rotated-rectangle?rq=3
    Sint16 body_points[4][2] = {{-50, -10}, {50, -10}, {50, 10}, {-50, 10}};

    Sint16 body_x[4] = {q_x + cos(q_theta)*body_points[0][0] - sin(q_theta)*body_points[0][1], 
                        q_x + cos(q_theta)*body_points[1][0] - sin(q_theta)*body_points[1][1], 
                        q_x + cos(q_theta)*body_points[2][0] - sin(q_theta)*body_points[2][1], 
                        q_x + cos(q_theta)*body_points[3][0] - sin(q_theta)*body_points[3][1]};
    
    Sint16 body_y[4] = {q_y + sin(q_theta)*body_points[0][0] + cos(q_theta)*body_points[0][1], 
                        q_y + sin(q_theta)*body_points[1][0] + cos(q_theta)*body_points[1][1], 
                        q_y + sin(q_theta)*body_points[2][0] + cos(q_theta)*body_points[2][1], 
                        q_y + sin(q_theta)*body_points[3][0] + cos(q_theta)*body_points[3][1]};

    Uint32 ticks = SDL_GetTicks();
    ticks /= 1;
    Sint16 q_x_propL = body_x[0];
    Sint16 q_y_propL = body_y[0];
    Sint16 q_x_propP = body_x[1];
    Sint16 q_y_propP = body_y[1];
    
    Sint16 propellerL1_x[4] = {q_x_propL + 0*sin(ticks), q_x_propL + 60*sin(ticks), q_x_propL + 60*sin(ticks), q_x_propL + 0*sin(ticks)};
    Sint16 propellerL1_y[4] = {q_y_propL + 0, q_y_propL + 20, q_y_propL + -20, q_y_propL + 0};
    Sint16 propellerL2_x[4] = {q_x_propL + 0*sin(ticks), q_x_propL - 60*sin(ticks), q_x_propL - 60*sin(ticks), q_x_propL + 0*sin(ticks)};
    Sint16 propellerL2_y[4] = {q_y_propL + 0, q_y_propL + 20, q_y_propL + -20, q_y_propL + 0};

    Sint16 propellerP1_x[4] = {q_x_propP + 0*sin(ticks), q_x_propP + 60*sin(ticks), q_x_propP + 60*sin(ticks), q_x_propP + 0*sin(ticks)};
    Sint16 propellerP1_y[4] = {q_y_propP + 0, q_y_propP + 20, q_y_propP + -20, q_y_propP + 0};
    Sint16 propellerP2_x[4] = {q_x_propP + 0*sin(ticks), q_x_propP - 60*sin(ticks), q_x_propP - 60*sin(ticks), q_x_propP + 0*sin(ticks)};
    Sint16 propellerP2_y[4] = {q_y_propP + 0, q_y_propP + 20, q_y_propP + -20, q_y_propP + 0};

    // SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x33, 0x34, 0xFF);
    // SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x57, 0x57, 0x57); // AA, RR, GG, BB
    // filledCircleColor(gRenderer.get(), q_x, q_y, 30, 0xFF0000FF); // 0xRRGGBBAA 
    filledPolygonColor(gRenderer.get(), body_x, body_y, 4, 0xFF575757);
    bezierColor(gRenderer.get(), propellerL1_x, propellerL1_y, 4, 10, 0xFF3333FF);
    bezierColor(gRenderer.get(), propellerL2_x, propellerL2_y, 4, 10, 0xFF3333FF);
    bezierColor(gRenderer.get(), propellerP1_x, propellerP1_y, 4, 10, 0xFF3333FF);
    bezierColor(gRenderer.get(), propellerP2_x, propellerP2_y, 4, 10, 0xFF3333FF);
    
    
}