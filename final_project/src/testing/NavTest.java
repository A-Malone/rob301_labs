package testing;

import common.RobotUtils;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.utility.Delay;

public class NavTest
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

        // Get the motors
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;

        // Create the pilot based on the Robot's parameters
        DifferentialPilot pilot = new DifferentialPilot(RobotUtils.wheel_diameter, RobotUtils.wheel_track_corrected,
                left, right);

        pilot.setAcceleration(30);
        pilot.setTravelSpeed(15);
        pilot.setRotateSpeed(180 / 6);

        // Create the navigator used to perform the operations described
        Navigator nav = new Navigator(pilot);

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
