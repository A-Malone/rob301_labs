package common;

import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.robotics.RegulatedMotor;

public class MotorUtils
{
    public static void setSpeeds(NXTRegulatedMotor left, NXTRegulatedMotor right, float sleft, float sright)
    {
        left.synchronizeWith(new RegulatedMotor[] {right});
        left.startSynchronization();
        right.flt();
        left.flt();
        left.endSynchronization();
        
        left.setSpeed(sleft);
        right.setSpeed(sright);
        right.forward();
        left.forward();
    }
}
