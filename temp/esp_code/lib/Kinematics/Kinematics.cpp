#include "Kinematics.hpp"

Kinematics::Kinematics(const double & shoulder_length, const double & elbow_length, const double & wrist_length):shoulder_length(shoulder_length), elbow_length(elbow_length), wrist_length(wrist_length) {}

void Kinematics::Initialize(const double & shoulder_length_, const double & elbow_length_, const double & wrist_length_)
{
	shoulder_length = shoulder_length_;
    elbow_length = elbow_length_;
    wrist_length = wrist_length_;
}

double Kinematics::GetDomain(const double & x, const double & y, const double & z)
{
	double D = (pow(y, 2) + pow(-z, 2) - pow(shoulder_length, 2) +
             	pow(-x, 2) - pow(elbow_length, 2) - pow(wrist_length, 2)) / (
                2.0 * wrist_length * elbow_length);
    if (D > 1.0)
    {
        D = 1.0;
    } if (D < -1.0)
    {
        D = -1.0;
    }

    return D;
}

void Kinematics::RightIK(const double & x, const double & y, const double & z, const double & D, double (& angles) [3])
{
	double wrist_angle = atan2(-sqrt(1.0 - pow(D, 2)), D);
	double sqrt_component = pow(y, 2) + pow(-z, 2) - pow(shoulder_length, 2);
	if (sqrt_component < 0.0)
	{
		sqrt_component = 0.0;
	}
	double shoulder_angle = -atan2(z, y) - atan2(
				            sqrt(sqrt_component), -shoulder_length);
	double elbow_angle = atan2(-x, sqrt(sqrt_component)) - atan2(
            			 wrist_length * sin(wrist_angle),
            			 elbow_length + wrist_length * cos(wrist_angle));

	angles[0] = NormalizeAngle(-shoulder_angle);
    angles[1] = NormalizeAngle(elbow_angle);
    angles[2] = NormalizeAngle(wrist_angle);
}

void Kinematics::LeftIK(const double & x, const double & y, const double & z, const double & D, double (& angles) [3])
{
	double wrist_angle = atan2(-sqrt(1.0 - pow(D, 2)), D);
	double sqrt_component = pow(y, 2) + pow(-z, 2) - pow(shoulder_length, 2);
	if (sqrt_component < 0.0)
	{
		sqrt_component = 0.0;
	}
	double shoulder_angle = -atan2(z, y) - atan2(
            				sqrt(sqrt_component), shoulder_length);
	double elbow_angle = atan2(-x, sqrt(sqrt_component)) - atan2(
            			 wrist_length * sin(wrist_angle),
            			 elbow_length + wrist_length * cos(wrist_angle));

	angles[0] = NormalizeAngle(-shoulder_angle);
    angles[1] = NormalizeAngle(-elbow_angle);
    angles[2] = NormalizeAngle(-wrist_angle);
}

double Kinematics::GetJointAngles(const double & x, const double & y, const double & z, const LegQuadrant & legquad, double (& angles) [3])
{
	double D = GetDomain(x, y, z);
	if (legquad == Right)
	{
		RightIK(x, y, z, D, angles);
	} else
	{
		LeftIK(x, y, z, D, angles);
	}
	return D;
}

double Kinematics::NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void Kinematics::ForwardKinematics(const double (&a)[3],
                                   const LegQuadrant &legquad,
                                   double (&p)[3])
{
    const double hip   = a[0];
    const double elbow = a[1];
    const double wrist = a[2];

    // Convert to the internal angles used by the IK derivation
    double s = -hip;                           // shoulder roll
    double e =  (legquad == Right) ?  elbow    // hip pitch
                                   : -elbow;   // (sign-flip left)
    double w =  (legquad == Right) ?  wrist
                                   : -wrist;   // knee/ankle

    // Pre-compute helper terms
    const double phi = atan2(wrist_length * sin(w),
                             elbow_length + wrist_length * cos(w));
    const double theta = e + phi;
    const double d = sqrt(pow(elbow_length,2) +
                          pow(wrist_length,2) +
                          2.0 * elbow_length * wrist_length * cos(w));

    const double r = fabs(d * cos(theta));     // ‖projection in YZ‖
    const double x = -r * tan(theta);          // forward (+) / back (-)

    // Shoulder offset sign differs between legs
    const double B = atan2(r,
                           (legquad == Right ? -shoulder_length
                                             :  shoulder_length));
    const double A = -s - B;
    const double yz_norm = sqrt(pow(shoulder_length,2) + r*r);

    const double y = yz_norm * cos(A);
    const double z = yz_norm * sin(A);

    p[0] = x;
    p[1] = y;
    p[2] = z;
}