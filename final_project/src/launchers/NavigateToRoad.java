package launchers;

import common.BoardUtils.Road;
import common.Robot;
import lejos.hardware.Button;
import lejos.robotics.navigation.Pose;
import localization.DirectionKalmanPoseProvider;
import tasks.ObstacleAvoidanceTask;

/** Entrypoint for the navigate to road task */
public class NavigateToRoad
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT
        // Create the robot with default parameters
        Robot robot = new Robot();

        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(robot.pilot, robot.gyro, true);
        robot.setPoseProvider(gyro_pose);
        robot.pose_provider.setPose(new Pose(0, -5, -90));
        
        // Choose road starting position
        Road road = Road.BLUE_ROAD;

        System.out.println("Press ENTER to start");
        Button.ENTER.waitForPress();
        
        boolean success = ObstacleAvoidanceTask.navigate_to_pose_task(robot, new Pose(0,100,0));
        
        if (success)
        {
            System.out.println("Press ENTER to end");
            Button.ENTER.waitForPress();
        }
    }
}
