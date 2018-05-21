//
// Created by alsaibie on 5/18/18.
//


#pragma once

#include <uORB/topics/
class AttitudeController {

public:
    AttitudeController();
    ~AttitudeController() {};


    void updateState();
    void updateSetpoint();
    void controlAttitude(); /**/
    void controlRates();
    void mixOutput();
    void compensateThrusterDynamics();

private:

    /* States */
    matrix::Quaternionf _att_q{};
    matrix::Vector3f    _att_rates{};

    /* Setpoints */
    matrix::Quaternionf _att_sp_q{};
    matrix::Vector3f    _att_rates_sp{};
    float               _thrust_sp{};

    /* Parameter Handles */


    /* Parameters */
    // Gains and stuff




};



