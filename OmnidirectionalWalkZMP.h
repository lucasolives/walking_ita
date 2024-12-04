/*
 * OmnidirectionalWalkZMP.h
 *
 *  Created on: Sep 14, 2014
 *      Author: mmaximo
 */

#ifndef OMNIDIRECTIONALWALKZMP_H_
#define OMNIDIRECTIONALWALKZMP_H_

#include "math/Vector3.h"
#include "math/Pose2D.h"
#include "representations/Joints.h"
#include "InverseKinematics.h"

namespace control {

namespace walking {

using namespace representations;

enum StoppingState {
    NONE,
    STOP_REQUESTED,
    ADJUSTING_1ST,
    ADJUSTING_2ND,
    STOPPED
};

struct OmnidirectionalWalkZMPParams {
    double period;
    double zCom;
    double zStep;
    double doubleSupportRatio;
    double ySeparation;
    double armsCompensationFactor;
    double comSwingFactor;

    OmnidirectionalWalkZMPParams();
    static OmnidirectionalWalkZMPParams getDefaultOmnidirecionalWalkZMPParams();
};

class OmnidirectionalWalkZMP {
public:
    bool leftFootIsSwing;
    itandroids_lib::math::Vector3<double> torso;

    OmnidirectionalWalkZMP(OmnidirectionalWalkZMPParams walkParams,
                           RobotPhysicalParameters physicalParams,
                           InverseKinematics *inverseKinematics);

    virtual ~OmnidirectionalWalkZMP();

    void restart();

    void update(double elapsedTime, Joints &joints);

    void setDesiredVelocity(itandroids_lib::math::Pose2D velocity);

    void requestStop();

    bool hasStopped();

    itandroids_lib::math::Vector3<double> getOdometry();

private:
    const double GRAVITY;

    StoppingState stoppingState;

    OmnidirectionalWalkZMPParams walkParams;
    InverseKinematics *inverseKinematics;
    RobotPhysicalParameters physicalParams;

    double tZMP;

    itandroids_lib::math::Vector3<double> swingFootBegin;
    itandroids_lib::math::Vector3<double> swingFootEnd;
    itandroids_lib::math::Vector3<double> torsoBegin;
    itandroids_lib::math::Vector3<double> torsoEnd;

    double walkTime;

    itandroids_lib::math::Vector3<double> velocity;

    itandroids_lib::math::Vector3<double> previousTorso;
    itandroids_lib::math::Vector3<double> odometry;

    void switchSwingFoot();

    void solveZMP(double zs, double z1, double z2, double x1, double x2, double &aP,
                  double &aN);

    double getCoM(double t, double aP, double aN, double zs, double z1, double z2);

    void computeNewStep();

    itandroids_lib::math::Vector3<double> poseGlobal(itandroids_lib::math::Vector3<double> poseRelative,
                                         itandroids_lib::math::Vector3<double> pose);

    itandroids_lib::math::Vector3<double> poseRelative(itandroids_lib::math::Vector3<double> poseGlobal,
                                           itandroids_lib::math::Vector3<double> pose);

    itandroids_lib::math::Vector3<double> getDesiredTorsoDisplacement(itandroids_lib::math::Vector3<double> torsoBegin);

    itandroids_lib::math::Vector3<double>
    limitTorso(itandroids_lib::math::Vector3<double> beginTorso, itandroids_lib::math::Vector3<double> torsoDisplacement);

    double computeXFrac(double t);

    double computeYFrac(double t);

    double computeZFrac(double t);

    double computeThetaFrac(double t);
};

} /* namespace walking*/

} /* namespace control */

#endif /* OMNIDIRECTIONALWALKZMP_H_ */
