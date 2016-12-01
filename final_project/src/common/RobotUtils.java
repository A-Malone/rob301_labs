package common;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.utility.PilotProps;

public class RobotUtils
{
    static final float CM_TO_IN = 0.393701f;

    // ---- ROBOT PHYSICAL PROPERTIES
    public static float wheel_width = 2.8f;
    public static float wheel_track_outer = 14.7f;
    public static float wheel_track_width = (wheel_track_outer - wheel_width);
    public static float wheel_track_corrected = (wheel_track_outer - 0.6f * wheel_width);
    public static float wheel_track_radius = wheel_track_width / 2;

    public static float wheel_diameter = 5.6f;

    public static float get_track_width()
    {
        PilotProps pp = new PilotProps();
        return Float.parseFloat(pp.getProperty(PilotProps.KEY_TRACKWIDTH, String.valueOf(wheel_track_width)));
    }
    
    // ---- ROBOT WIRING PROPERTIES
    
    //Motors
    public static final NXTRegulatedMotor LEFT_MOTOR = Motor.A;
    public static final NXTRegulatedMotor RIGHT_MOTOR = Motor.B;
    public static final NXTRegulatedMotor CLAW_MOTOR = Motor.C;
    public static final NXTRegulatedMotor ULTRA_MOTOR = Motor.D;
    
    // Sensors
    public static final Port ULTRASOUND_PORT = SensorPort.S4;
    public static final Port COLOR_PORT = SensorPort.S2;
    public static final Port GYRO_PORT = SensorPort.S1;
    
    // ---- ROBOT UTILITY FUNCTIONS
    public static boolean open_claw(NXTRegulatedMotor claw)
    {
        boolean success = true;
        
        claw.rotate(90,true);
        while (success)
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
               success = false;
               claw.stop();
            }
        }
        
        return success;
    }
    
    public static boolean close_claw(NXTRegulatedMotor claw)
    {
        boolean success = true;
        
        claw.rotate(-90, true);
        while (success)
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
               success = false;
               claw.stop();
            }
        }
        
        return success;
    }
}
