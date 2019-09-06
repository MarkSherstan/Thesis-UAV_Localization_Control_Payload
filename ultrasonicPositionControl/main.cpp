#include "attitude_controller.h"

int i1 = 0;
int i2 = 0;

void AttitudeController::calc_average_velocity(){
    vel_avg[vel_avg_ind % 3] = vel_curr;
    if (reinitialize_state || i1 < VEL_AVGS) {
        if (reinitialize_state) i1 = 0;
        i1++;
        vel_curr_avg = vel_curr;
    } else {
        for (int i = 0 ; i < VEL_AVGS; i++){
            vel_curr_avg = vel_avg[i];
        }
        vel_curr_avg = vel_curr_avg/ VEL_AVGS;
    }
    vel_avg_ind++;
}

void AttitudeController::calc_average_position(){
    pos_avg[pos_avg_ind % 3] = pos_curr;
    if (reinitialize_state || i2 < POS_AVGS) {
        if (reinitialize_state) i2 = 0;
        i2++;
        pos_curr_avg = pos_curr;
    } else {
        pos_curr_avg = (pos_avg[0] + pos_avg[1] + pos_avg[2])/3;
    }
    pos_avg_ind++;
}

void AttitudeController::update_dt()  {
    current_run_time = std::chrono::high_resolution_clock::now();
    elapsed = current_run_time - last_run_time;
    dt = elapsed.count();
    last_run_time = current_run_time;

    reinitialize_state = (dt < RESET_STATE_DT) ? false : true;
}

void AttitudeController::update_velocity_state()  {
    if (!reinitialize_state){
        vel_curr = (pos_curr - pos_last)/dt;
    }
    pos_last = pos_curr;
}

void AttitudeController::update_velocity_err_integration()  {
    if (reinitialize_state) {
        vel_int = {0, 0, 0};
    } else {
        vel_int += vel_err*dt;
    }
}

float AttitudeController::bind_max_value(float val, float max_val){
    if (val > max_val)
        return max_val;
    else if (val < -max_val)
        return -max_val;
    return val;
}

void AttitudeController::updateQuaternion(){
    float cr2 = cosf(roll_target*M_PI/180.0f*0.5f);
    float cp2 = cosf(pitch_target*M_PI/180.0f*0.5f);
    float cy2 = cosf(yaw_target*M_PI/180.0f*0.5f);
    float sr2 = sinf(roll_target*M_PI/180.0f*0.5f);
    float sp2 = sinf(pitch_target*M_PI/180.0f*0.5f);
    float sy2 = sinf(yaw_target*M_PI/180.0f*0.5f);

    q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}




float AttitudeController::map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float AttitudeController::constrain(float val, float max_val, float min_val) {
    if (val > max_val)
        return max_val;
    else if (val < min_val)
        return min_val;
    return val;
}

float AttitudeController::PD(float error, float errorPrev, float dt) {
  float kp = 0.3;
  float kd = 0.2;

  float P = kp * error;
  float D = kd * ((error - errorPrev) / dt);

  return (P + D);
}

void AttitudeController::positionControl(Vector3f current_pos, Vector3f desired_pos) {
  // Maybe move this to the header
  float kYaw = 2;
  float kThrottle = 0.5;
  float angleLimit = 3.1415/8;   // 22.5 degrees
  float yawRateLimit = 3.1415/4; // 45.0 degrees
  Vector3f errorPrev; // This should be initialized to zero

  // Average position data
  calc_average_position();

  // Error calculation
  error = desired_pos - current_pos;

  // Time elapsed
  update_dt();

  // Run some control
  rollControl = PD(error[0], errorPrev[0], dt);           // East
  pitchControl = PD(error[1], errorPrev[1], dt);          // North
  yawControl = constrain(heading * kYaw, -yawRateLimit, yawRateLimit);
  thrustControl = constrain(error[2] * kThrottle, -1, 1); // Down

  // Update previous error
  errorPrev = error;

  // Set the controller values
  rollAngle = -constrain(rollControl, -angleLimit, angleLimit);  // Radians
  pitchAngle = constrain(pitchControl, -angleLimit, angleLimit); // Radians
  yawRate = yawControl;                                          // Radians per second
  thrust = map(thrustControl, -1, 1, 0, 1);                      // Normalized thrust relative to waypoint speed up

  // Print data to screen
  cout << rollAngle << pitchAngle << yawRate << thrust << endl
}


void AttitudeController::run_loop(Vector3f current_pos, Vector3f desired_pos){
    update_dt();

    pos_curr = current_pos;

    //updating current velocity
    update_velocity_state();
    calc_average_velocity();
    calc_average_position();

    //Calculating position error
    pos_desi = desired_pos;
    pos_err = pos_desi - pos_curr_avg;

    //Calculating desired velocity
    vel_desi = pos_err.multiplyGain(pos_pgain);

    //bind velocity to max
    vel_desi.bindToMaxVal(max_vel);

    //calculating velocity error
    vel_err = vel_desi - vel_curr;

    //updating velocity error integration
    update_velocity_err_integration();

    //calculating desired acceleration
    acc_desi = vel_err.multiplyGain(vel_pgain) + vel_int.multiplyGain(vel_igain);
    reinitialize_state = false;
}

void AttitudeController::acceleration_to_attitude(float forward_acc, float right_acc, float rot, float desir_yaw){
    this->forward_acc = forward_acc;
    this->right_acc = right_acc;

    rot  = rot * M_PI/180.0f;

    rot_right_acc = right_acc * cos(rot) - forward_acc * sin(rot);
    rot_forward_acc = right_acc * sin(rot) + forward_acc * cos(rot);

    pitch_target = atanf(-rot_forward_acc/(9.806))*(180.0f/M_PI);
    cos_pitch_target = cosf(pitch_target*M_PI/180.0f);
    roll_target = atanf(rot_right_acc*cos_pitch_target/(9.806))*(180.0f/M_PI);

    pitch_target = bind_max_value(pitch_target, MAX_ANGLE);
    roll_target = bind_max_value(roll_target, MAX_ANGLE);
    yaw_target = desir_yaw;
}

Vector3f AttitudeController::get_desired_velocity(){
    return vel_desi;
}

string AttitudeController::get_state_string(){
    stringstream output;
    output << std::fixed;
    output << std::setprecision(5);

    output << pos_curr.x << ',' << pos_curr.y << ',' << pos_curr.z << ',';
    output << pos_last.x << ',' << pos_last.y << ',' << pos_last.z << ',';
    output << pos_err.x  << ',' << pos_err.y  << ',' << pos_err.z  << ',';
    output << pos_desi.x << ',' << pos_desi.y << ',' << pos_desi.z << ',';
    output << vel_curr.x << ',' << vel_curr.y << ',' << vel_curr.z << ',';
    output << vel_desi.x << ',' << vel_desi.y << ',' << vel_desi.z << ',';
    output << vel_err.x  << ',' << vel_err.y  << ',' << vel_err.z  << ',';
    output << vel_int.x  << ',' << vel_int.y  << ',' << vel_int.z  << ',';
    output << acc_desi.x << ',' << acc_desi.y << ',' << acc_desi.z << ',';

    output << forward_acc << ',' << right_acc << ',';
    output << rot_forward_acc << ',' << rot_right_acc << ',';

    output << pitch_target << ',';
    output << roll_target << ',';
    output << yaw_target << ',';
    output << thrust;
    return output.str();
}

//To do
//test what happens if the uav doesn't recevie messages frequently enough.
//Have fun in life.
//Set wp speed up to 10cm/s or change thrust math
//check avg velcotiy calc
