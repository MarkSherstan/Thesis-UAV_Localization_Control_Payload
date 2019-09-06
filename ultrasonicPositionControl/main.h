#include "vector3.h"
#include <ctime>
#include <chrono>
#include <string>
#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

#define RESET_STATE_DT 0.1
#define MAX_HORZ_VEL 0.50
#define MAX_VERT_VEL 0.25
#define MAX_ANGLE 5
#define POS_AVGS 3
#define VEL_AVGS 5

class AttitudeController {
    private:

        std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> last_run_time = std::chrono::high_resolution_clock::now();
        std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> current_run_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_run_time - last_run_time;

        float dt = 0.1;

        int vel_avg_ind = 0;
        Vector3f vel_avg[VEL_AVGS] = {{0, 0, 0},{0, 0, 0},{0, 0, 0}};
        Vector3f vel_curr_avg = {0,0,0};

        int pos_avg_ind = 0;
        Vector3f pos_avg[POS_AVGS] = {{0, 0, 0},{0, 0, 0},{0, 0, 0}};
        Vector3f pos_curr_avg = {0,0,0};

        void calc_average_position();
        void calc_average_velocity();
        void update_dt();
        void update_velocity_state();
        void update_velocity_err_integration();
        float bind_max_value(float val, float max_val);
        float bind_max_value(float val, float max_val, float min_val);

        void updateQuaternion();
        float q1,q2,q3,q4;

    public:
        AttitudeController();
        void run_loop(Vector3f current_pos, Vector3f desired_pos);
        void acceleration_to_attitude(float forward_acc, float right_acc, float rot, float desir_yaw = 0);
        Vector3f get_desired_velocity(void );
        string get_state_string(void );

        bool reinitialize_state = true;

        Vector3f pos_curr = {0, 0, 0};
        Vector3f pos_last = {0, 0, 0};
        Vector3f pos_err  = {0, 0, 0};

        Vector3f pos_desi = {0, 0, 0};

        Vector3f vel_curr = {0, 0, 0};
        Vector3f vel_last = {0, 0, 0};

        Vector3f vel_desi = {0, 0, 0};
        Vector3f vel_err  = {0, 0, 0};
        Vector3f vel_int  = {0, 0, 0};

        Vector3f acc_desi = {0, 0, 0};

        Vector3f pos_pgain = {0.50, 0.50, 0.50};
        Vector3f vel_pgain = {1.5, 1.5, 1.5};

        Vector3f vel_igain = {0.00000, 0.00000, 0.00000};
        Vector3f max_vel = {MAX_HORZ_VEL, MAX_HORZ_VEL, MAX_VERT_VEL};

        float forward_acc;
        float right_acc;
        float rot_forward_acc;
        float rot_right_acc;

        float hover_throttle = 0.5;
        float pitch_target;
        float cos_pitch_target;
        float roll_target;
        float yaw_target;
        float thrust;
};
