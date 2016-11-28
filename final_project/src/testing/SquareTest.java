package testing;

import common.GyrodometryPoseProvider;
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

public class SquareTest
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

        // Get the motors
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;

        // Get the gyro
        EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S1);

        // Create the pilot based on the Robot's parameters
        DifferentialPilot pilot = new DifferentialPilot(RobotUtils.wheel_diameter, RobotUtils.wheel_track_width, left,
                right);

        // Pass in the pilot as a MoveProvider, and the Gyro
        GyrodometryPoseProvider gyro_pose = new GyrodometryPoseProvider(pilot, gyro);

        // Create the navigator used to perform the operations described
        Navigator nav = new Navigator(pilot, gyro_pose);

        // ---- DEFINE PATH

        // Define the square
        Pose start = new Pose(0, 0, 0);
        Pose xa = new Pose(100, 0, 90);
        Pose xb = new Pose(100, 100, 180);
        Pose xc = new Pose(0, 100, 270);
        Pose[] states = { start, xa, xb, xc, start };

        // Add waypoints to the path
        for (Pose pose : states)
        {
            nav.addWaypoint(new Waypoint(pose));
        }

        // ---- WAIT
        Button.ENTER.waitForPress();

        // ---- GO
        nav.followPath();

        // Wait for the robot to be finished, or us to exit early
        while (nav.getPath() != null && Button.ENTER.isDown())
        {
            Delay.msDelay(50);
        }
    }
}
