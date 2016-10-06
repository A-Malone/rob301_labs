package rob;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.utility.Delay;
import rob.ClawUtils;

public class PathPlanning {

    public static void main(String[] args) throws Exception {		
        NXTRegulatedMotor az_motor = Motor.B;
        NXTRegulatedMotor att_motor = Motor.C;
        
        att_motor.setSpeed(100);
        az_motor.setSpeed(90);
        
        //att_motor.rotateTo(270*3);
        
        
        ClawUtils.synchronized_movement(
                att_motor,
                az_motor,
                180,
                -ClawUtils.get_azimuth(10)
                );
        Delay.msDelay(200);
        
        ClawUtils.synchronized_movement(
                att_motor,
                az_motor,
                0,
                0
                );
        Delay.msDelay(200);
                
    }
}
