#include "GaitGen.hpp"

GaitGen::GaitGen(double T,
                 double Tstance,
                 double Tswing,
                 double DutyCycle,
                 double PhaseDiff)
    : T(T),
      Tstance(Tstance),
      Tswing(Tswing),
      DutyCycle(DutyCycle),
      PhaseDiff(PhaseDiff)      
{
    if (T != Tstance + Tswing) {
        DEBUG_E("T must be equal to the sum of Tstance and Tswing, %f != %f + %f = %f",
                T, Tstance, Tswing, Tstance + Tswing);
    }
}
void GaitGen::Init(SpotModel model){
    model = model;
}

Eigen::Vector3d GaitGen::bezier(double t,
                           Eigen::Vector3d p0,
                           Eigen::Vector3d p1,
                           Eigen::Vector3d p2,
                           Eigen::Vector3d p3
                           )
{
    return pow(1-t, 3)*p0 +
           3*pow(1-t, 2)*t*p1 +
           3*(1-t)*pow(t, 2)*p2 +
           pow(t, 3)*p3;
}