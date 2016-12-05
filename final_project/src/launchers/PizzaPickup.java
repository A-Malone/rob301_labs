package launchers;

import common.BoardUtils.PizzaPedestal;
import common.Robot;
import lejos.hardware.Button;
import lejos.robotics.navigation.Pose;
import localization.DirectionKalmanPoseProvider;
import tasks.PizzaPickupTask;

/** Entrypoint for the pizza pick-up task */
public class PizzaPickup
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT
        // Create the robot with default parameters
        Robot robot = new Robot();

        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(robot.pilot, robot.gyro, true);
        robot.setPoseProvider(gyro_pose);

        robot.pose_provider.setPose(new Pose(0, 0, -90));

        // Get the pizza pedestal we'll be moving to
        PizzaPedestal target = PizzaPedestal.LEFT;

        System.out.println("Press ENTER to start");
        Button.ENTER.waitForPress();

        boolean success = PizzaPickupTask.run_task(robot, target);

        if (success)
        {
            System.out.println("Press ENTER to end");
            Button.ENTER.waitForPress();
        }
    }
}
