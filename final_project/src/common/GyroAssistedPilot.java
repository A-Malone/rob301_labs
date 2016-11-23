package common;

import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.localization.OdometryPoseProvider;

public class GyroAssistedPilot extends DifferentialPilot
{

    public GyroAssistedPilot(double leftWheelDiameter, double rightWheelDiameter, double trackWidth,
            RegulatedMotor leftMotor, RegulatedMotor rightMotor, boolean reverse) {
        super(leftWheelDiameter, rightWheelDiameter, trackWidth, leftMotor, rightMotor, reverse);
        // TODO Auto-generated constructor stub
    }

}
