package launchers;

import common.Robot;
import common.BoardUtils.PizzaPedestal;
import lejos.hardware.Button;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.utility.Delay;
import localization.DirectionKalmanPoseProvider;
import tasks.PizzaPickupTask;

public class PizzaPickup
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT        
        // Create the robot with default parameters
        Robot robot = new Robot();
        
        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(robot.pilot, robot.gyro);
        robot.setPoseProvider(gyro_pose);
        
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
