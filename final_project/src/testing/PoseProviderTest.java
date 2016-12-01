package testing;

import common.RobotUtils;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;
import localization.DirectionKalmanPoseProvider;
import localization.GyrodometryPoseProvider;

public class PoseProviderTest
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

        // Get the motors
        NXTRegulatedMotor left = RobotUtils.LEFT_MOTOR;
        NXTRegulatedMotor right = RobotUtils.RIGHT_MOTOR;

        // Get the gyro
        EV3GyroSensor gyro = new EV3GyroSensor(RobotUtils.GYRO_PORT);

        // Create the pilot based on the Robot's parameters
        DifferentialPilot pilot = new DifferentialPilot(RobotUtils.wheel_diameter, RobotUtils.get_track_width(), left,
                right);

        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(pilot, gyro);

        // Create the navigator used to perform the operations described
        Navigator nav = new Navigator(pilot, gyro_pose);

        // ---- DEFINE PATH

        nav.addWaypoint(50, 0);
        nav.addWaypoint(50, 50);
        nav.addWaypoint(0, 50);
        nav.addWaypoint(0, 0);

        while (!lejos.hardware.Button.ENTER.isDown())
        {
            Delay.msDelay(20);
        }

        boolean success = true;

        nav.followPath();
        while (nav.isMoving() && success)
        {
            success = success && !lejos.hardware.Button.ESCAPE.isDown();
        }

        if (!success)
        {
            nav.stop();
        }
    }
}
