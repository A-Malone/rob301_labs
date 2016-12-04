package testing;

import common.Robot;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
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

        // Create the robot with default parameters
        Robot robot = new Robot();

        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(robot.pilot, robot.gyro.getAngleMode(), true);
        robot.setPoseProvider(gyro_pose);
        
        robot.pilot.setAcceleration((int)(robot.pilot.getTravelSpeed()*2));

        // ---- DEFINE PATH

        int n = 2;
        for(int i = 0; i < n; i++)
        {
            robot.navigator.addWaypoint(25, 0);
            robot.navigator.addWaypoint(25, 25);
            robot.navigator.addWaypoint(0, 25);
            robot.navigator.addWaypoint(0, 0);
        }

        System.out.println("Press ENTER to start");
        Button.ENTER.waitForPress();
        
        LCD.clear();

        boolean success = true;

        robot.navigator.followPath();
        while (robot.navigator.isMoving() && success)
        {
            success = success && !lejos.hardware.Button.ESCAPE.isDown();            
        }

        if (!success)
        {
            robot.navigator.stop();
        }
        
        Button.ESCAPE.waitForPress();
    }
}
