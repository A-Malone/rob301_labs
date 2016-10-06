package launchers;

import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.robotics.RegulatedMotor;

public class DriveStraight
{
    public static void main(String[] args) throws Exception
    {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;
        
        left.synchronizeWith(new RegulatedMotor[] {right});
        left.startSynchronization();
        left.rotate(720);
        right.rotate(720);
        left.endSynchronization();
    }
}
