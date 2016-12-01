package launchers;

import common.BoardUtils.Road;
import common.RobotUtils;
import lejos.hardware.Button;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import localization.DirectionKalmanPoseProvider;
import tasks.ObstacleAvoidanceTask;

public class NavigateToRoad
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

        // Get the motors
        NXTRegulatedMotor left = RobotUtils.LEFT_MOTOR;
        NXTRegulatedMotor right = RobotUtils.RIGHT_MOTOR;        

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
        
        // Choose road starting position
        Road road = Road.BLUE_ROAD;

        System.out.println("Press ENTER to start");
        Button.ENTER.waitForPress();
        
        boolean success = ObstacleAvoidanceTask.navigate_to_pose_task(nav, pilot, ultra, road.start);
        
        if (success)
        {
            System.out.println("Press ENTER to end");
            Button.ENTER.waitForPress();
        }
    }
}
