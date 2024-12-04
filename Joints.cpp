/*
 * Joints.cpp
 *
 *  Created on: Mar 2, 2014
 *      Author: itandroids
 */

#include "Joints.h"

#include <string.h>
#include <math.h>
#include "communication/gazebo/DarwinGazeboJoint.h"

namespace representations {

Joints::Joints(std::vector<double> joints) {
    clear();
    for (int i = 0; i < joints.size(); ++i) {
        setValue(static_cast<HINGE_JOINTS>(i), joints[i]);
    }
}

void Joints::clear() {
    neckPitch = 0.0;
    neckYaw = 0.0;

	leftShoulderPitch = 0.0;
	leftShoulderRoll = 0.0;
	leftElbowYaw = 0.0;

	rightShoulderPitch = 0.0;
	rightShoulderRoll = 0.0;
	rightElbowYaw = 0.0;

	leftHipYaw = 0.0;
	leftHipRoll = 0.0;
	leftHipPitch = 0.0;
	leftKneePitch = 0.0;
	leftAnklePitch = 0.0;
	leftAnkleRoll = 0.0;

	rightHipYaw = 0.0;
	rightHipRoll = 0.0;
	rightHipPitch = 0.0;
	rightKneePitch = 0.0;
	rightAnklePitch = 0.0;
	rightAnkleRoll = 0.0;
}

double Joints::getValueByName(std::string jointName) {
	if (strcmp("leftShoulderPitch", jointName.c_str()) == 0) {
		return leftShoulderPitch;
	} else if (strcmp("leftShoulderRoll", jointName.c_str()) == 0) {
		return leftShoulderRoll;
	} else if (strcmp("leftElbowYaw", jointName.c_str()) == 0) {
		return leftElbowYaw;
	} else if (strcmp("rightShoulderPitch", jointName.c_str()) == 0) {
		return rightShoulderPitch;
	} else if (strcmp("rightShoulderRoll", jointName.c_str()) == 0) {
		return rightShoulderRoll;
	} else if (strcmp("rightElbowYaw", jointName.c_str()) == 0) {
		return rightElbowYaw;
	} else if (strcmp("leftHipYaw", jointName.c_str()) == 0) {
		return leftHipYaw;
	} else if (strcmp("leftHipRoll", jointName.c_str()) == 0) {
		return leftHipRoll;
	} else if (strcmp("leftHipPitch", jointName.c_str()) == 0) {
		return leftHipPitch;
	} else if (strcmp("leftKneePitch", jointName.c_str()) == 0) {
		return leftKneePitch;
	} else if (strcmp("leftAnklePitch", jointName.c_str()) == 0) {
		return leftAnklePitch;
	} else if (strcmp("leftAnkleRoll", jointName.c_str()) == 0) {
		return leftAnkleRoll;
	} else if (strcmp("rightHipYaw", jointName.c_str()) == 0) {
		return rightHipYaw;
	} else if (strcmp("rightHipRoll", jointName.c_str()) == 0) {
		return rightHipRoll;
	} else if (strcmp("rightHipPitch", jointName.c_str()) == 0) {
		return rightHipPitch;
	} else if (strcmp("rightKneePitch", jointName.c_str()) == 0) {
		return rightKneePitch;
	} else if (strcmp("rightAnklePitch", jointName.c_str()) == 0) {
		return rightAnklePitch;
	} else if (strcmp("rightAnkleRoll", jointName.c_str()) == 0) {
		return rightAnkleRoll;
	} else if (strcmp("neckYaw", jointName.c_str()) == 0) {
		return neckYaw;
	} else if (strcmp("neckPitch", jointName.c_str()) == 0) {
		return neckPitch;
	}

	return 0.0;
}

void Joints::setValueByName(std::string jointName, double value) {
	if (strcmp("leftShoulderPitch", jointName.c_str()) == 0) {
		leftShoulderPitch = value;
	} else if (strcmp("leftShoulderRoll", jointName.c_str()) == 0) {
		leftShoulderRoll = value;
	} else if (strcmp("leftElbowYaw", jointName.c_str()) == 0) {
		leftElbowYaw = value;
	} else if (strcmp("rightShoulderPitch", jointName.c_str()) == 0) {
		rightShoulderPitch = value;
	} else if (strcmp("rightShoulderRoll", jointName.c_str()) == 0) {
		rightShoulderRoll = value;
	} else if (strcmp("rightElbowYaw", jointName.c_str()) == 0) {
		rightElbowYaw = value;
	} else if (strcmp("leftHipYaw", jointName.c_str()) == 0) {
		leftHipYaw = value;
	} else if (strcmp("leftHipRoll", jointName.c_str()) == 0) {
		leftHipRoll = value;
	} else if (strcmp("leftHipPitch", jointName.c_str()) == 0) {
		leftHipPitch = value;
	} else if (strcmp("leftKneePitch", jointName.c_str()) == 0) {
		leftKneePitch = value;
	} else if (strcmp("leftAnklePitch", jointName.c_str()) == 0) {
		leftAnklePitch = value;
	} else if (strcmp("leftAnkleRoll", jointName.c_str()) == 0) {
		leftAnkleRoll = value;
	} else if (strcmp("rightHipYaw", jointName.c_str()) == 0) {
		rightHipYaw = value;
	} else if (strcmp("rightHipRoll", jointName.c_str()) == 0) {
		rightHipRoll = value;
	} else if (strcmp("rightHipPitch", jointName.c_str()) == 0) {
		rightHipPitch = value;
	} else if (strcmp("rightKneePitch", jointName.c_str()) == 0) {
		rightKneePitch = value;
	} else if (strcmp("rightAnklePitch", jointName.c_str()) == 0) {
		rightAnklePitch = value;
	} else if (strcmp("rightAnkleRoll", jointName.c_str()) == 0) {
		rightAnkleRoll = value;
	} else if (strcmp("neckYaw", jointName.c_str()) == 0) {
		neckYaw = value;
	} else if (strcmp("neckPitch", jointName.c_str()) == 0) {
		neckPitch = value;
	}
}

Joints Joints::operator+(const Joints& other) {
	Joints result;

	result.leftShoulderPitch = this->leftShoulderPitch
			+ other.leftShoulderPitch;
	result.leftShoulderRoll = this->leftShoulderRoll + other.leftShoulderRoll;
	result.leftElbowYaw = this->leftElbowYaw + other.leftElbowYaw;

	result.rightShoulderPitch = this->rightShoulderPitch
			+ other.rightShoulderPitch;
	result.rightShoulderRoll = this->rightShoulderRoll + other.rightShoulderRoll;
	result.rightElbowYaw = this->rightElbowYaw + other.rightElbowYaw;

	result.leftHipYaw = this->leftHipYaw + other.leftHipYaw;
	result.leftHipRoll = this->leftHipRoll + other.leftHipRoll;
	result.leftHipPitch = this->leftHipPitch + other.leftHipPitch;
	result.leftKneePitch = this->leftKneePitch + other.leftKneePitch;
	result.leftAnklePitch = this->leftAnklePitch + other.leftAnklePitch;
	result.leftAnkleRoll = this->leftAnkleRoll + other.leftAnkleRoll;

	result.rightHipYaw = this->rightHipYaw + other.rightHipYaw;
	result.rightHipRoll = this->rightHipRoll + other.rightHipRoll;
	result.rightHipPitch = this->rightHipPitch + other.rightHipPitch;
	result.rightKneePitch = this->rightKneePitch + other.rightKneePitch;
	result.rightAnklePitch = this->rightAnklePitch + other.rightAnklePitch;
	result.rightAnkleRoll = this->rightAnkleRoll + other.rightAnkleRoll;

	result.neckYaw = this->neckYaw + other.neckYaw;
	result.neckPitch = this->neckPitch + other.neckPitch;

	return result;
}

Joints Joints::operator-(const Joints& other) {
	Joints result;

	result.leftShoulderPitch = this->leftShoulderPitch
			- other.leftShoulderPitch;
	result.leftShoulderRoll = this->leftShoulderRoll - other.leftShoulderRoll;
	result.leftElbowYaw = this->leftElbowYaw - other.leftElbowYaw;

	result.rightShoulderPitch = this->rightShoulderPitch
			- other.rightShoulderPitch;
	result.rightShoulderRoll = this->rightShoulderRoll - other.rightShoulderRoll;
	result.rightElbowYaw = this->rightElbowYaw - other.rightElbowYaw;

	result.leftHipYaw = this->leftHipYaw - other.leftHipYaw;
	result.leftHipRoll = this->leftHipRoll - other.leftHipRoll;
	result.leftHipPitch = this->leftHipPitch - other.leftHipPitch;
	result.leftKneePitch = this->leftKneePitch - other.leftKneePitch;
	result.leftAnklePitch = this->leftAnklePitch - other.leftAnklePitch;
	result.leftAnkleRoll = this->leftAnkleRoll - other.leftAnkleRoll;

	result.rightHipYaw = this->rightHipYaw - other.rightHipYaw;
	result.rightHipRoll = this->rightHipRoll - other.rightHipRoll;
	result.rightHipPitch = this->rightHipPitch - other.rightHipPitch;
	result.rightKneePitch = this->rightKneePitch - other.rightKneePitch;
	result.rightAnklePitch = this->rightAnklePitch - other.rightAnklePitch;
	result.rightAnkleRoll = this->rightAnkleRoll - other.rightAnkleRoll;

	result.neckYaw = this->neckYaw - other.neckYaw;
	result.neckPitch = this->neckPitch - other.neckPitch;

	return result;
}

std::string Joints::toString() {
	std::stringstream stream;

	stream << leftShoulderPitch << " ";
	stream << leftShoulderRoll << " ";
	stream << leftElbowYaw << " ";

	stream << rightShoulderPitch << " ";
	stream << rightShoulderRoll << " ";
	stream << rightElbowYaw << " ";

	stream << leftHipYaw << " ";
	stream << leftHipRoll << " ";
	stream << leftHipPitch << " ";
	stream << leftKneePitch << " ";
	stream << leftAnklePitch << " ";
	stream << leftAnkleRoll << " ";

	stream << rightHipYaw << " ";
	stream << rightHipRoll << " ";
	stream << rightHipPitch << " ";
	stream << rightKneePitch << " ";
	stream << rightAnklePitch << " ";
	stream << rightAnkleRoll << " ";

	stream << neckYaw << " ";
	stream << neckPitch << " ";

	return stream.str();
}

Joints operator*(double scalar, const Joints& jointsTargets) {
	Joints result;

	result.leftShoulderPitch = scalar * jointsTargets.leftShoulderPitch;
	result.leftShoulderRoll = scalar * jointsTargets.leftShoulderRoll;
	result.leftElbowYaw = scalar * jointsTargets.leftElbowYaw;

	result.rightShoulderPitch = scalar * jointsTargets.rightShoulderPitch;
	result.rightShoulderRoll = scalar * jointsTargets.rightShoulderRoll;
	result.rightElbowYaw = scalar * jointsTargets.rightElbowYaw;

	result.leftHipYaw = scalar * jointsTargets.leftHipYaw;
	result.leftHipRoll = scalar * jointsTargets.leftHipRoll;
	result.leftHipPitch = scalar * jointsTargets.leftHipPitch;
	result.leftKneePitch = scalar * jointsTargets.leftKneePitch;
	result.leftAnklePitch = scalar * jointsTargets.leftAnklePitch;
	result.leftAnkleRoll = scalar * jointsTargets.leftAnkleRoll;

	result.rightHipYaw = scalar * jointsTargets.rightHipYaw;
	result.rightHipRoll = scalar * jointsTargets.rightHipRoll;
	result.rightHipPitch = scalar * jointsTargets.rightHipPitch;
	result.rightKneePitch = scalar * jointsTargets.rightKneePitch;
	result.rightAnklePitch = scalar * jointsTargets.rightAnklePitch;
	result.rightAnkleRoll = scalar * jointsTargets.rightAnkleRoll;

	result.neckYaw = scalar * jointsTargets.neckYaw;
	result.neckPitch = scalar * jointsTargets.neckPitch;

	return result;
}

Joints Joints::operator*(const Joints& other) {
	Joints result;

	result.leftShoulderPitch = this->leftShoulderPitch
			* other.leftShoulderPitch;
	result.leftShoulderRoll = this->leftShoulderRoll * other.leftShoulderRoll;
	result.leftElbowYaw = this->leftElbowYaw * other.leftElbowYaw;

	result.rightShoulderPitch = this->rightShoulderPitch
			* other.rightShoulderPitch;
	result.rightShoulderRoll = this->rightShoulderRoll * other.rightShoulderRoll;
	result.rightElbowYaw = this->rightElbowYaw * other.rightElbowYaw;

	result.leftHipYaw = this->leftHipYaw * other.leftHipYaw;
	result.leftHipRoll = this->leftHipRoll * other.leftHipRoll;
	result.leftHipPitch = this->leftHipPitch * other.leftHipPitch;
	result.leftKneePitch = this->leftKneePitch * other.leftKneePitch;
	result.leftAnklePitch = this->leftAnklePitch * other.leftAnklePitch;
	result.leftAnkleRoll = this->leftAnkleRoll * other.leftAnkleRoll;

	result.rightHipYaw = this->rightHipYaw * other.rightHipYaw;
	result.rightHipRoll = this->rightHipRoll * other.rightHipRoll;
	result.rightHipPitch = this->rightHipPitch * other.rightHipPitch;
	result.rightKneePitch = this->rightKneePitch * other.rightKneePitch;
	result.rightAnklePitch = this->rightAnklePitch * other.rightAnklePitch;
	result.rightAnkleRoll = this->rightAnkleRoll * other.rightAnkleRoll;

	result.neckYaw = this->neckYaw * other.neckYaw;
	result.neckPitch = this->neckPitch * other.neckPitch;

	return result;
}

void Joints::scale(double scalar) {
    neckPitch *= scalar;
    neckYaw *= scalar;

    leftShoulderPitch *= scalar;
    leftShoulderRoll *= scalar;
    leftElbowYaw *= scalar;

    rightShoulderPitch *= scalar;
    rightShoulderRoll *= scalar;
    rightElbowYaw *= scalar;

    leftHipYaw *= scalar;
    leftHipRoll *= scalar;
    leftHipPitch *= scalar;
    leftKneePitch *= scalar;
    leftAnklePitch *= scalar;
    leftAnkleRoll *= scalar;

    rightHipYaw *= scalar;
    rightHipRoll *= scalar;
    rightHipPitch *= scalar;
    rightKneePitch *= scalar;
    rightAnklePitch *= scalar;
    rightAnkleRoll *= scalar;
}

double Joints::lInfiniteNorm() {
	double norm = 0.0;

    norm = std::max(fabs(neckYaw), norm);
    norm = std::max(fabs(neckPitch), norm);

	norm = std::max(fabs(leftShoulderPitch), norm);
	norm = std::max(fabs(leftShoulderRoll), norm);
	norm = std::max(fabs(leftElbowYaw), norm);

	norm = std::max(fabs(rightShoulderPitch), norm);
	norm = std::max(fabs(rightShoulderRoll), norm);
	norm = std::max(fabs(rightElbowYaw), norm);

	norm = std::max(fabs(leftHipYaw), norm);
	norm = std::max(fabs(leftHipRoll), norm);
	norm = std::max(fabs(leftHipPitch), norm);
	norm = std::max(fabs(leftKneePitch), norm);
	norm = std::max(fabs(leftAnklePitch), norm);
	norm = std::max(fabs(leftAnkleRoll), norm);

	norm = std::max(fabs(rightHipYaw), norm);
	norm = std::max(fabs(rightHipRoll), norm);
	norm = std::max(fabs(rightHipPitch), norm);
	norm = std::max(fabs(rightKneePitch), norm);
	norm = std::max(fabs(rightAnklePitch), norm);
	norm = std::max(fabs(rightAnkleRoll), norm);

	return norm;
}

double Joints::getValue(HINGE_JOINTS joint) {
    switch (joint) {
        case NECK_YAW:
            return neckYaw;
        case NECK_PITCH:
            return neckPitch;
        case LEFT_SHOULDER_PITCH:
            return leftShoulderPitch;
        case LEFT_SHOULDER_YAW:
            return leftShoulderRoll;
		case LEFT_ELBOW_YAW:
			return leftElbowYaw;
        case LEFT_HIP_ROLL:
            return leftHipRoll;
        case LEFT_HIP_PITCH:
            return leftHipPitch;
        case LEFT_KNEE_PITCH:
            return leftKneePitch;
        case LEFT_ANKLE_PITCH:
            return leftAnklePitch;
        case LEFT_ANKLE_ROLL:
            return leftAnkleRoll;
        case RIGHT_SHOULDER_PITCH:
            return rightShoulderPitch;
        case RIGHT_SHOULDER_YAW:
            return rightShoulderRoll;
		case RIGHT_ELBOW_YAW:
			return rightElbowYaw;
        case RIGHT_HIP_YAW:
            return rightHipYaw;
        case RIGHT_HIP_ROLL:
            return rightHipRoll;
        case RIGHT_HIP_PITCH:
            return rightHipPitch;
        case RIGHT_KNEE_PITCH:
            return rightKneePitch;
        case RIGHT_ANKLE_PITCH:
            return rightAnklePitch;
        case RIGHT_ANKLE_ROLL:
            return rightAnkleRoll;
    }
    return 0.0;
}

void Joints::setValue(HINGE_JOINTS joint, double value) {
    switch (joint) {
        case NECK_YAW:
            neckYaw = value;
            break;
        case NECK_PITCH:
            neckPitch = value;
            break;
        case LEFT_SHOULDER_PITCH:
            leftShoulderPitch = value;
            break;
        case LEFT_SHOULDER_YAW:
            leftShoulderRoll = value;
            break;
		case LEFT_ELBOW_YAW:
			leftElbowYaw = value;
			break;
        case LEFT_HIP_YAW:
            leftHipYaw = value;
            break;
        case LEFT_HIP_ROLL:
            leftHipRoll = value;
            break;
        case LEFT_HIP_PITCH:
            leftHipPitch = value;
            break;
        case LEFT_KNEE_PITCH:
            leftKneePitch = value;
            break;
        case LEFT_ANKLE_PITCH:
            leftAnklePitch = value;
            break;
        case LEFT_ANKLE_ROLL:
            leftAnkleRoll = value;
            break;
        case RIGHT_SHOULDER_PITCH:
            rightShoulderPitch = value;
            break;
        case RIGHT_SHOULDER_YAW:
            rightShoulderRoll = value;
            break;
		case RIGHT_ELBOW_YAW:
			rightElbowYaw = value;
        case RIGHT_HIP_YAW:
            rightHipYaw = value;
            break;
        case RIGHT_HIP_ROLL:
            rightHipRoll = value;
            break;
        case RIGHT_HIP_PITCH:
            rightHipPitch = value;
            break;
        case RIGHT_KNEE_PITCH:
            rightKneePitch = value;
            break;
        case RIGHT_ANKLE_PITCH:
            rightAnklePitch = value;
            break;
        case RIGHT_ANKLE_ROLL:
            rightAnkleRoll = value;
            break;
    }
}

std::map<Joints::HINGE_JOINTS, double> Joints::getAsMap() {
    std::map<HINGE_JOINTS, double> map;
    map[NECK_PITCH] = neckPitch;
    map[NECK_YAW] = neckYaw;
    map[LEFT_SHOULDER_PITCH] = leftShoulderPitch;
    map[LEFT_SHOULDER_YAW] = leftShoulderRoll;
	map[LEFT_ELBOW_YAW] = leftElbowYaw;
    map[LEFT_HIP_YAW] = leftHipYaw;
    map[LEFT_HIP_ROLL] = leftHipRoll;
    map[LEFT_HIP_PITCH] = leftHipPitch;
    map[LEFT_KNEE_PITCH] = leftKneePitch;
    map[LEFT_ANKLE_PITCH] = leftAnklePitch;
    map[LEFT_ANKLE_ROLL] = leftAnkleRoll;
    map[RIGHT_SHOULDER_PITCH] = rightShoulderPitch;
    map[RIGHT_SHOULDER_YAW] = rightShoulderRoll;
	map[RIGHT_ELBOW_YAW] = rightElbowYaw;
    map[RIGHT_HIP_YAW] = rightHipYaw;
    map[RIGHT_HIP_ROLL] = rightHipRoll;
    map[RIGHT_HIP_PITCH] = rightHipPitch;
    map[RIGHT_KNEE_PITCH] = rightKneePitch;
    map[RIGHT_ANKLE_PITCH] = rightAnklePitch;
    map[RIGHT_ANKLE_ROLL] = rightAnkleRoll;
    return map;
}

// TODO: Consider directions fixing if needed
Joints Joints::getDarwinDirectionsFixing() {
    Joints joints;

    joints.neckPitch = 1.0;
    joints.neckYaw = 1.0;

    joints.leftShoulderPitch = 1.0;
    joints.leftShoulderRoll = 1.0;
    joints.leftElbowYaw = 1.0;

    joints.rightShoulderPitch = 1.0;
    joints.rightShoulderRoll = 1.0;
    joints.rightElbowYaw = 1.0;

    joints.leftHipYaw = 1.0;
    joints.leftHipRoll = 1.0;
    joints.leftHipPitch = 1.0;
    joints.leftKneePitch = 1.0;
    joints.leftAnklePitch = 1.0;
    joints.leftAnkleRoll = 1.0;

    joints.rightHipYaw = 1.0;
    joints.rightHipRoll = 1.0;
    joints.rightHipPitch = 1.0;
    joints.rightKneePitch = 1.0;
    joints.rightAnklePitch = 1.0;
    joints.rightAnkleRoll = 1.0;

    return joints;
}

Joints Joints::getSymmetricJoints(Joints joints) {
	std::cout << "chamou getSymmetricJoints" << std::endl;

    Joints retJoints;

    retJoints.rightShoulderPitch = joints.leftShoulderPitch;
    retJoints.rightShoulderRoll = joints.leftShoulderRoll;
	retJoints.rightElbowYaw = joints.leftElbowYaw;
    retJoints.rightAnklePitch = joints.leftAnklePitch;
    retJoints.rightAnkleRoll = -joints.leftAnkleRoll;
    retJoints.rightHipPitch = joints.leftHipPitch;
    retJoints.rightHipRoll = -joints.leftHipRoll;
    retJoints.rightHipYaw = -joints.leftHipYaw;
    retJoints.rightKneePitch = joints.leftKneePitch;
    retJoints.leftShoulderPitch = joints.rightShoulderPitch;
    retJoints.leftShoulderRoll = joints.rightShoulderRoll;
	retJoints.leftElbowYaw = joints.rightElbowYaw;
    retJoints.leftAnklePitch = joints.rightAnklePitch;
    retJoints.leftAnkleRoll = -joints.rightAnkleRoll;
    retJoints.leftHipPitch = joints.rightHipPitch;
    retJoints.leftHipRoll = -joints.rightHipRoll;
    retJoints.leftHipYaw = -joints.rightHipYaw;
    retJoints.leftKneePitch = joints.rightKneePitch;
    retJoints.leftShoulderPitch = joints.rightShoulderPitch;
    retJoints.leftShoulderRoll = -joints.rightShoulderRoll;

    return retJoints;
}

} /* namespace representations */
