package localization;

import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.localization.OdometryPoseProvider;

/** Early prototype of a heading-corrected gyro-based pilot.*/
public class GyroAssistedPilot extends DifferentialPilot
{
    public GyroAssistedPilot(double leftWheelDiameter, double rightWheelDiameter, double trackWidth,
            RegulatedMotor leftMotor, RegulatedMotor rightMotor, boolean reverse) {
        super(leftWheelDiameter, rightWheelDiameter, trackWidth, leftMotor, rightMotor, reverse);
        // TODO Auto-generated constructor stub
    }
}
