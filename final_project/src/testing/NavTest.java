package testing;

import common.Robot;
import lejos.utility.Delay;

public class NavTest
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

        Robot robot = new Robot();

        robot.navigator.addWaypoint(50, 0);
        robot.navigator.addWaypoint(50, 50);
        robot.navigator.addWaypoint(0, 50);
        robot.navigator.addWaypoint(0, 0);

        while (!lejos.hardware.Button.ENTER.isDown())
        {
            Delay.msDelay(20);
        }

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
    }
}
