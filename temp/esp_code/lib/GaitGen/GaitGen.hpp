#ifndef GAITGEN_HPP
#define GAITGEN_HPP


#define ENABLE_DEBUG
#include <MacroDebugger.h>
#include <ArduinoEigen.h>
#include "SpotModel.hpp"

#define DEFAULT_T           0.12 // seconds
#define DEFAULT_T_STANCE    0.06 // seconds
#define DEFAULT_T_SWING     0.06 // seconds
#define DEFAULT_DUTY_CYCLE  0.5   // %
#define DEFAULT_PHASE_DIFF  0.5   // %

class GaitGen {
public:
    GaitGen(double T        = DEFAULT_T,
            double Tstance  = DEFAULT_T_STANCE,
            double Tswing   = DEFAULT_T_SWING,
            double DutyCycle = DEFAULT_DUTY_CYCLE,
            double PhaseDiff = DEFAULT_PHASE_DIFF);

    void Init(SpotModel model);
    
    Eigen::Vector3d bezier(double t,
                           Eigen::Vector3d p0,
                           Eigen::Vector3d p1,
                           Eigen::Vector3d p2,
                           Eigen::Vector3d p3   );

private:

    double Tstance;
    double Tswing;
    double T;
    double DutyCycle;
    double PhaseDiff;  

    SpotModel model;
};

#endif