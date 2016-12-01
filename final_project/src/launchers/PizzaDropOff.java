package launchers;

import common.BoardUtils.House;
import common.BoardUtils.Road;
import common.Robot;
import lejos.hardware.Button;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import localization.DirectionKalmanPoseProvider;
import tasks.PizzaDropOffTask;

public class PizzaDropOff
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

     // Create the robot with default parameters
        Robot robot = new Robot();
        
        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(robot.pilot, robot.gyro);
        robot.setPoseProvider(gyro_pose);
        
        // Choose road
        Road road = Road.BLUE_ROAD;
        gyro_pose.setPose(road.start);
        
        // Choose House
        House house = new House(road, true, 1);
        
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
