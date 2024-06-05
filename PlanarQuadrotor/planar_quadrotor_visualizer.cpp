#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. OPTIONAL: Animate proppelers (extra points)
 * # TODO: Connect rotation to speed of propellers
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    Sint16 body_x[4] = {q_x + 0, q_x + 100, q_x + 100, q_x + 0};
    Sint16 body_y[4] = {q_y + 0, q_y + 0, q_y + 20, q_y + 20};
    Sint16 propellerL_x[7] = {q_x + 0, q_x + 60, q_x + 60, q_x + 0, q_x + -60, q_x + -60, q_x + 0};
    Sint16 propellerL_y[7] = {q_y + -20, q_y + -40, q_y + 0, q_y + -20, q_y + -40, q_y + 0, q_y + -20};
    Sint16 propellerP_x[7] = {q_x + 100, q_x + 40, q_x + 40, q_x + 100, q_x + 160, q_x + 160, q_x + 100};
    Sint16 propellerP_y[7] = {q_y + -20, q_y + -40, q_y + 0, q_y + -20, q_y + -40, q_y + 0, q_y + -20};

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x33, 0x33, 0xFF);
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x57, 0x57, 0x57); // AA, RR, GG, BB
    // filledCircleColor(gRenderer.get(), q_x, q_y, 30, 0xFF0000FF); // 0xRRGGBBAA 
    filledPolygonColor(gRenderer.get(), body_x, body_y, 4, 0xFF575757);
    bezierColor(gRenderer.get(), propellerL_x, propellerL_y, 7, 5, 0xFF3333FF);
    bezierColor(gRenderer.get(), propellerP_x, propellerP_y, 7, 5, 0xFF3333FF);
}