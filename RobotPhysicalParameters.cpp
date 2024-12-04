/*
 * RobotPhysicalParameters.cpp
 *
 *  Created on: Sep 6, 2014
 *      Author: mmaximo
 */

#include "RobotPhysicalParameters.h"

#include <math.h>
#include <stdio.h>

namespace representations {

RobotPhysicalParameters RobotPhysicalParameters::getBioloidPhysicalParameters() {
    RobotPhysicalParameters physicalParameters;

    physicalParameters.torsoToLeftHip = itandroids_lib::math::Vector3<double>(0.0164, 0.033, -0.0619);
    //physicalParameters.torsoToLeftHip = Vector3D(0.0164f, 0.033f, -0.0919f);
    physicalParameters.leftAnkleToLeftFoot = itandroids_lib::math::Vector3<double>(-0.0103, 0.008, -0.03195);

    physicalParameters.armLength = 0.175;

    physicalParameters.upperLegLength = sqrt(0.015 * 0.015 + 0.075 * 0.075);
    physicalParameters.lowerLegLength = physicalParameters.upperLegLength;
    double legHeight0 = 0.15;
    double a = physicalParameters.upperLegLength;
    double b = physicalParameters.lowerLegLength;
    double c = legHeight0;

    double gamma0 = acos((a * a + b * b - c * c) / (2.0 * a * b));
    double beta0 = acos((a * a + c * c - b * b) / (2.0 * a * c));
    double alpha0 = acos((b * b + c * c - a * a) / (2.0 * b * c));

    physicalParameters.hipPitch0 = beta0;
    physicalParameters.kneePitch0 = -(M_PI - gamma0);
    physicalParameters.anklePitch0 = alpha0;

    printf("hipPitch0: %f, kneePitch0: %f, anklePitch0: %f",
           physicalParameters.hipPitch0, physicalParameters.kneePitch0,
           physicalParameters.anklePitch0);

    return physicalParameters;
}

// TODO: update this method with Darwin parameters
RobotPhysicalParameters RobotPhysicalParameters::getDarwinPhysicalParameters() {
    using itandroids_lib::math::Vector3;

    RobotPhysicalParameters physicalParameters;

    physicalParameters.armLength = 0.189;

    physicalParameters.upperLegLength = 0.093;
    physicalParameters.lowerLegLength = physicalParameters.upperLegLength;

    physicalParameters.hipPitch0 = 0.0;
    physicalParameters.kneePitch0 = 0.0;
    physicalParameters.anklePitch0 = 0.0;

    physicalParameters.torsoToLeftHip =
            Vector3<double>(0.02, 0.037, /*-0.1222*/0.02);//-0.1222);
    physicalParameters.leftAnkleToLeftFoot = Vector3<double>(0.0, 0.0105/*0.02*/, -0.0335);

//    printf("hipPitch0: %f, kneePitch0: %f, anklePitch0: %f",
//           physicalParameters.hipPitch0, physicalParameters.kneePitch0,
//           physicalParameters.anklePitch0);

    return physicalParameters;
}

} /* namespace representations */
