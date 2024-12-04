//#include <ros/ros.h>
//#include <sensor_msgs/JointState.h>
#include "RobotPhysicalParameters.h"
#include "OmnidirectionalWalkZMP.h"
#include "InverseKinematics.h"
#include "Joints.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle nh;

    // Criar publisher
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    using namespace control::walking;
    using namespace representations;

    // Configurar parâmetros físicos do robô
    RobotPhysicalParameters robotParams = RobotPhysicalParameters::getBioloidPhysicalParameters();

    // Configurar parâmetros de caminhada
    OmnidirectionalWalkZMPParams walkParams = OmnidirectionalWalkZMPParams::getDefaultOmnidirecionalWalkZMPParams();

    // Inicializar cinemática inversa
    InverseKinematics inverseKinematics(robotParams);

    // Inicializar controle de caminhada
    OmnidirectionalWalkZMP walker(walkParams, robotParams, &inverseKinematics);

    // Criar instância de juntas do robô
    Joints robotJoints;

    // Configurar velocidade desejada
    itandroids_lib::math::Pose2D desiredVelocity(0.1, 0.0, 0.0); // Velocidade em metros por segundo
    walker.setDesiredVelocity(desiredVelocity);

    ros::Rate rate(100); // Taxa de 100 Hz

    while (ros::ok()) {
        // Simular o tempo decorrido (10 ms por ciclo)
        double elapsedTime = 0.01;

        // Atualizar o controle de caminhada
        walker.update(elapsedTime, robotJoints);

        // Criar e publicar a mensagem JointState
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
        joint_msg.name = {
            "neck_pitch", "neck_yaw",
            "left_shoulder_pitch", "left_shoulder_roll", "left_elbow_yaw",
            "right_shoulder_pitch", "right_shoulder_roll", "right_elbow_yaw",
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch",
            "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
            "right_knee_pitch", "right_ankle_pitch", "right_ankle_roll"
        };
        joint_msg.position = {
            robotJoints.neckPitch, robotJoints.neckYaw,
            robotJoints.leftShoulderPitch, robotJoints.leftShoulderRoll, robotJoints.leftElbowYaw,
            robotJoints.rightShoulderPitch, robotJoints.rightShoulderRoll, robotJoints.rightElbowYaw,
            robotJoints.leftHipYaw, robotJoints.leftHipRoll, robotJoints.leftHipPitch,
            robotJoints.leftKneePitch, robotJoints.leftAnklePitch, robotJoints.leftAnkleRoll,
            robotJoints.rightHipYaw, robotJoints.rightHipRoll, robotJoints.rightHipPitch,
            robotJoints.rightKneePitch, robotJoints.rightAnklePitch, robotJoints.rightAnkleRoll
        };

        joint_pub.publish(joint_msg);

        // Simular parada (opcional)
        walker.requestStop();

        ros::spinOnce();
        rate.sleep();

        if (walker.hasStopped()) {
            ROS_INFO("O robô parou de caminhar.");
            break;
        }
    }

    return 0;
}
