package launchers;

import common.BoardUtils.House;
import common.BoardUtils.Road;
import common.Robot;
import lejos.hardware.Button;
import localization.DirectionKalmanPoseProvider;
import tasks.PizzaDropOffTask;

/** Entrypoint for the pizza drop-off task */
public class PizzaDropOff
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

     // Create the robot with default parameters
        Robot robot = new Robot();
        
        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(robot.pilot, robot.gyro, true);
        robot.setPoseProvider(gyro_pose);
        
        // Choose road
        Road road = Road.BLUE_ROAD;
        gyro_pose.setPose(road.start);
        
        // Choose House
        House house = new House(road, false, 1);
        
        System.out.println("Press ENTER to start");
        Button.ENTER.waitForPress();

        boolean success = PizzaDropOffTask.run_task(robot, house);
        
        if (success)
        {
            System.out.println("Press ENTER to end");
            Button.ENTER.waitForPress();
        }
    }
}
