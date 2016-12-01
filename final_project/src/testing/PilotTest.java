package testing;

import common.Robot;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import localization.DirectionKalmanPoseProvider;

public class PilotTest
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

        // Create the robot with default parameters
        Robot robot = new Robot();

        while (!lejos.hardware.Button.ENTER.isDown())
        {
            Delay.msDelay(20);
        }

        boolean success = true;
        for (int i = 0; i < 4; i++)
        {
            if (success)
            {
                robot.pilot.travel(50, true);
            }

            while (robot.pilot.isMoving() && success)
            {
                success = success && !lejos.hardware.Button.ESCAPE.isDown();
            }

            if (success)
            {
                robot.pilot.rotate(180);
            }

            while (robot.pilot.isMoving() && success)
            {
                success = success && !lejos.hardware.Button.ESCAPE.isDown();
            }
        }
    }
}
