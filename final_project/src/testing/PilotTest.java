package testing;

import common.RobotUtils;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;

public class PilotTest
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

        // Get the motors
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;

        // Create the pilot based on the Robot's parameters
        DifferentialPilot pilot = new DifferentialPilot(RobotUtils.wheel_diameter, RobotUtils.get_track_width(), left,
                right);

        pilot.setTravelSpeed(15);
        pilot.setRotateSpeed(180 / 4);

        while (!lejos.hardware.Button.ENTER.isDown())
        {
            Delay.msDelay(20);
        }

        boolean success = true;
        for (int i = 0; i < 4; i++)
        {
            if (success)
            {
                pilot.travel(50, true);
            }
            while (pilot.isMoving() && success)
            {
                success = success && !lejos.hardware.Button.ESCAPE.isDown();
            }

            if (success)
            {
                pilot.rotate(180);
            }
            while (pilot.isMoving() && success)
            {
                success = success && !lejos.hardware.Button.ESCAPE.isDown();
            }
        }
    }
}
