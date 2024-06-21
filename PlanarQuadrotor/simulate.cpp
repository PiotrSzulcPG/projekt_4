/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <set>
#include <irrKlang.h> // https://www.ambiera.com/irrklang/

using namespace irrklang;
// #pragma comment(lib, "irrKlang.lib") // link with irrKlang.dll, copied from tutorial

Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 4e-3, 4e-3, 4e2, 8e-3, 4.5e-2, 2 / 2 / M_PI;
    R.row(0) << 3e1, 7;
    R.row(1) << 7, 3e1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{  
    ISoundEngine* audio_engine = createIrrKlangDevice();
    if (!audio_engine) {
        throw std::runtime_error("Couldn't load audio engine (createIrrKlangDevice() failed)");
    }
    // TODO: Change sound pitch based on rotor speed
    ISound* rotor_sound_3D = audio_engine->play3D("../../PlanarQuadrotor/droneFlyingSound.wav", vec3df(0, 0, 0), true, false, true);
    bool mute = false;

    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;
    int x, y;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    initial_state<<SCREEN_WIDTH/2,SCREEN_HEIGHT/2,0,0,0,0;
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 250, 100, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);
    
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            // get data from GetState and history
            state = quadrotor.GetState();
            int q_x = state[0];
            int q_y = state[1];
            int q_theta = state[2];

            x_history.push_back(q_x);
            y_history.push_back(q_y);
            theta_history.push_back(q_theta);

            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                    SDL_GetKeyboardState(NULL); // 'p' key is 'P'
                    std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    std::cout << "MOUSE CLICKED!! Position of click: "<< x << ", " << y << std::endl;
                    float new_x =static_cast<float>(x);
                    float new_y = static_cast<float>(y);
                    goal_state << new_x, new_y, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p) 
                { // plot based on https://github.com/alandefreitas/matplotplusplus/blob/master/examples/line_plot/plot/plot_9.cpp
                    std::cout << "KEY CLICKED (P): " << SDL_GetKeyName('p') << std::endl;
                    std::cout << "Simulation ended. Number of data to process: " << x_history.size() <<". Plotting... ";
                    std::set<std::vector<float>> XY = {x_history, y_history};
                    matplot::tiledlayout(2, 1);;
                    auto xy = matplot::nexttile();
                    auto plotted = matplot::plot(xy, XY);
                    matplot::ylabel("X, Y");
                    auto th = matplot::nexttile();
                    matplot::plot(th, theta_history);
                    matplot::xlabel("ticks");
                    matplot::ylabel("theta");
                    matplot::show();
                    std::cout << "Plotting finished";
                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_m) 
                {
                    if (mute) {
                        mute = false;
                        rotor_sound_3D->setVolume(1);
                    } else {
                        mute = true;
                        rotor_sound_3D->setVolume(0);
                    }
                }
                // rotor_sound_3D->setPlaybackSpeed(2);
            }

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);

            
        }
    } else {
        throw std::runtime_error("Failed to initialize SDL_Init");
    }
    SDL_Quit();
    rotor_sound_3D->drop();
    audio_engine->drop();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
