/*
 * RobotPhysicalParameters.h
 *
 *  Created on: Sep 6, 2014
 *      Author: mmaximo
 */

#ifndef ROBOTPHYSICALPARAMETERS_H_
#define ROBOTPHYSICALPARAMETERS_H_

#include "math/Vector3.h"

namespace representations {

struct RobotPhysicalParameters {
	itandroids_lib::math::Vector3<double> torsoToLeftHip;
	itandroids_lib::math::Vector3<double> leftAnkleToLeftFoot;
	double upperLegLength;
	double lowerLegLength;
	double hipPitch0;
	double kneePitch0;
	double anklePitch0;

	double armLength;

	static RobotPhysicalParameters getBioloidPhysicalParameters();
	static RobotPhysicalParameters getDarwinPhysicalParameters();
};

} /* namespace representations */

#endif /* ROBOTPHYSICALPARAMETERS_H_ */
