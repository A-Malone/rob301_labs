package common;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Navigator;
import lejos.utility.PilotProps;

public class Robot
{
    static final float CM_TO_IN = 0.393701f;

    // ---- ROBOT PHYSICAL PROPERTIES
    public static float wheel_width = 2.8f;
    public static float wheel_track_outer = 14.7f;
    public static float wheel_track_width = (wheel_track_outer - wheel_width);
    public static float wheel_track_corrected = 12.83f;
    public static float wheel_track_radius = wheel_track_width / 2;

    public static float wheel_diameter = 5.6f;

    // ---- ROBOT WIRING PROPERTIES

    // Motors
    public static final NXTRegulatedMotor LEFT_MOTOR = Motor.A;
    public static final NXTRegulatedMotor RIGHT_MOTOR = Motor.B;
    public static final NXTRegulatedMotor CLAW_MOTOR = Motor.D;
    public static final NXTRegulatedMotor ULTRA_MOTOR = Motor.C;

    // Sensors
    public static final Port ULTRASOUND_PORT = SensorPort.S1;
    public static final Port COLOR_PORT = SensorPort.S2;
    public static final Port GYRO_PORT = SensorPort.S4;

    // NON-STATIC VARIABLES
    public DifferentialPilot pilot;
    public Navigator navigator;
    public PoseProvider pose_provider;

    public EV3GyroSensor gyro;
    public EV3UltrasonicSensor ultra;
    public EV3ColorSensor color;

    /** Constructor using default pose provider */
    public Robot()
    {
        // Get all sensors
        gyro = new EV3GyroSensor(Robot.GYRO_PORT);
        ultra = new EV3UltrasonicSensor(Robot.ULTRASOUND_PORT);
        color = new EV3ColorSensor(Robot.COLOR_PORT);
        
        // Set the rotation speed of the claw
        CLAW_MOTOR.setSpeed(20);

        // Create the pilot based on the Robot's parameters
        pilot = new DifferentialPilot(Robot.wheel_diameter, Robot.get_track_width(), LEFT_MOTOR, RIGHT_MOTOR, true);

        pilot.setTravelSpeed(15);
        pilot.setRotateSpeed(180 / 3);

        // Create the navigator used to perform the operations described
        navigator = new Navigator(pilot);

        pose_provider = navigator.getPoseProvider();
    }

    public void setPoseProvider(PoseProvider ppv)
    {
        pose_provider = ppv;
        navigator.setPoseProvider(ppv);
    }

    public static float get_track_width()
    {
        //return 12.83f;
        PilotProps pp = new PilotProps();
        return Float.parseFloat(pp.getProperty(PilotProps.KEY_TRACKWIDTH, String.valueOf(wheel_track_width)));
    }

    // ---- ROBOT UTILITY FUNCTIONS
    public boolean open_claw()
    {
        boolean success = true;

        CLAW_MOTOR.rotate(90, true);
        while (success)
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
                success = false;
                CLAW_MOTOR.stop();
            }
        }

        return success;
    }

    public boolean close_claw()
    {
        boolean success = true;

        CLAW_MOTOR.rotate(-90, true);
        while (success)
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
                success = false;
                CLAW_MOTOR.stop();
            }
        }

        return success;
    }
}
