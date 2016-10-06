package rob;

import lejos.hardware.motor.NXTRegulatedMotor;

public class ClawUtils {
    public static float l1 = 7;
    public static float l2 = 16;
    public static float l3 = 19;
    public static float l4 = 11;
    public static float az_gear_ratio = 5;
    public static float at_gear_ratio = 2.5f;

    public static int get_azimuth(float z)
    {        
        System.out.println(Math.asin((z + l4 - l2)/l3));
        return (int)(az_gear_ratio*Math.asin((z + l4 - l2)/l3)*180/Math.PI/2);
    }
    
    public static int get_attitude(int psi)
    {
        return (int)(at_gear_ratio*psi);
    }

    public static int min_azimuth = get_azimuth(0);

    public static void synchronized_movement(
            NXTRegulatedMotor att_motor,
            NXTRegulatedMotor az_motor, 
            int attitude,
            int azimuth
            )
    {
        att_motor.synchronizeWith(new NXTRegulatedMotor[]{az_motor});

        att_motor.startSynchronization();
        att_motor.rotateTo(get_attitude(attitude), true);
        az_motor.rotateTo((int)(azimuth*az_gear_ratio), true);
        att_motor.endSynchronization();

        att_motor.waitComplete();
        az_motor.waitComplete();
    }
}
