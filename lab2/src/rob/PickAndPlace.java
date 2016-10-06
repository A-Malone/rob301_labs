package rob;

import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.utility.Delay;

public class PickAndPlace {

    public static void grab(
            NXTRegulatedMotor az_motor,
            NXTRegulatedMotor att_motor,
            NXTRegulatedMotor claw_motor,
            int position)
    {    
        PositionUtils.goto_position(att_motor, az_motor, position, 3);
        claw_motor.rotateTo(90);
        PositionUtils.goto_position(att_motor, az_motor, position, 0);
        claw_motor.rotateTo(45);
        PositionUtils.goto_position(att_motor, az_motor, position, 3);
    }
    
    public static void drop(
            NXTRegulatedMotor az_motor,
            NXTRegulatedMotor att_motor,
            NXTRegulatedMotor claw_motor,
            int position)
    {    
        PositionUtils.goto_position(att_motor, az_motor, position, 3);        
        PositionUtils.goto_position(att_motor, az_motor, position, 0);
        claw_motor.rotateTo(90);
        PositionUtils.goto_position(att_motor, az_motor, position, 3);
    }

    public static void main(String[] args) throws Exception {      
        NXTRegulatedMotor az_motor = Motor.A;
        NXTRegulatedMotor att_motor = Motor.B;
        NXTRegulatedMotor claw_motor = Motor.C;

        // 4 - 1
        grab(az_motor, att_motor, claw_motor, 4);
        Delay.msDelay(100);
        
        drop(az_motor, att_motor, claw_motor, 1);
        Delay.msDelay(100);
        
        // 2 - 4
        grab(az_motor, att_motor, claw_motor, 2);
        Delay.msDelay(100);
        
        drop(az_motor, att_motor, claw_motor, 4);
        Delay.msDelay(100);
        
        // 1 -5
        grab(az_motor, att_motor, claw_motor, 1);
        Delay.msDelay(100);
        
        drop(az_motor, att_motor, claw_motor, 5);
        Delay.msDelay(100);
    }
}
