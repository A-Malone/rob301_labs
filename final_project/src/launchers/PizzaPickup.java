package launchers;

import common.RobotUtils;
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

        // Get the motors
        NXTRegulatedMotor left = RobotUtils.LEFT_MOTOR;
        NXTRegulatedMotor right = RobotUtils.RIGHT_MOTOR;
        NXTRegulatedMotor claw = RobotUtils.CLAW_MOTOR;

        // Get the gyro
        EV3GyroSensor gyro = new EV3GyroSensor(RobotUtils.GYRO_PORT);
        
        // Get the ultrasound
        EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(RobotUtils.ULTRASOUND_PORT);

        // Create the pilot based on the Robot's parameters
        DifferentialPilot pilot = new DifferentialPilot(RobotUtils.wheel_diameter, RobotUtils.get_track_width(), left,
                right);

        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(pilot, gyro);

        // Create the navigator used to perform the operations described
        Navigator nav = new Navigator(pilot, gyro_pose);
        
        // Get the pizza pedestal we'll be moving to
        PizzaPedestal target = PizzaPedestal.LEFT;
        
        System.out.println("Press ENTER to start");
        Button.ENTER.waitForPress();

        boolean success = PizzaPickupTask.run_task(nav, pilot, ultra.getDistanceMode(), claw, target);
        
        if (success)
        {
            System.out.println("Press ENTER to end");
            Button.ENTER.waitForPress();
        }
    }
}
