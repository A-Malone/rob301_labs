package launchers;

import common.BoardUtils.House;
import common.BoardUtils.Road;
import common.RobotUtils;
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

        // Motors
        NXTRegulatedMotor left = RobotUtils.LEFT_MOTOR;
        NXTRegulatedMotor right = RobotUtils.RIGHT_MOTOR;
        NXTRegulatedMotor ultra_motor = RobotUtils.ULTRA_MOTOR;
        NXTRegulatedMotor claw = RobotUtils.CLAW_MOTOR;

        // Sensors
        EV3GyroSensor gyro = new EV3GyroSensor(RobotUtils.GYRO_PORT);
        EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(RobotUtils.ULTRASOUND_PORT);
        EV3ColorSensor color = new EV3ColorSensor(RobotUtils.COLOR_PORT);

        // Create the pilot based on the Robot's parameters
        DifferentialPilot pilot = new DifferentialPilot(RobotUtils.wheel_diameter, RobotUtils.get_track_width(), left,
                right);

        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(pilot, gyro);

        // Create the navigator used to perform the operations described
        Navigator nav = new Navigator(pilot, gyro_pose);
        
        // Choose road
        Road road = Road.BLUE_ROAD;
        gyro_pose.setPose(road.start);
        
        // Choose House
        House house = new House(road, true, 1);

        PizzaDropOffTask.run_task(nav, pilot, ultra, color, ultra_motor, claw, house);
    }
}
