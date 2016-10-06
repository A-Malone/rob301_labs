package rob;

import lejos.hardware.motor.NXTRegulatedMotor;

public class PositionUtils 
{
    private static int get_position_angle(int position)
    {
        return 180 - 45*position;
    }
    
    public static void goto_position(
            NXTRegulatedMotor att_motor,
            NXTRegulatedMotor az_motor, 
            int position,
            float z)
    {
        if (position < 1 || position > 5)
        {
            return;
        }
        
        ClawUtils.synchronized_movement(
                att_motor,
                az_motor,
                PositionUtils.get_position_angle(position), 
                ClawUtils.get_azimuth(z)
                );
    }
}
