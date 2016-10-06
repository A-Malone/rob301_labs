package launchers;

import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.robotics.RegulatedMotor;

public class DriveStraight
{
    static final float BASE_SPEED = 200;
    
    public static void main(String[] args) throws Exception
    {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;
        
//        left.synchronizeWith(new RegulatedMotor[] {right});
//        left.startSynchronization();
//        left.rotateTo(720);
//        right.rotateTo(720);
//        left.endSynchronization();
        
        left.setSpeed(BASE_SPEED);
        right.setSpeed(BASE_SPEED);
        left.forward();
        right.forward();
        
        Thread.sleep(5000);
    }
}
