package rob;

import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.utility.Delay;

public class PickAndStack {

    public static void grab(
            NXTRegulatedMotor az_motor,
            NXTRegulatedMotor att_motor,
            NXTRegulatedMotor claw_motor,
            int position,
            float z)
    {    
        PositionUtils.goto_position(att_motor, az_motor, position, 5);
        claw_motor.rotateTo(90);
        PositionUtils.goto_position(att_motor, az_motor, position, z);
        claw_motor.rotateTo(45);
        PositionUtils.goto_position(att_motor, az_motor, position, 5);
    }
    
    public static void drop(
            NXTRegulatedMotor az_motor,
            NXTRegulatedMotor att_motor,
            NXTRegulatedMotor claw_motor,
            int position,
            float z)
    {    
        PositionUtils.goto_position(att_motor, az_motor, position, 5);        
        PositionUtils.goto_position(att_motor, az_motor, position, z);
        claw_motor.rotateTo(90);
        PositionUtils.goto_position(att_motor, az_motor, position, 5);
    }

    public static void main(String[] args) throws Exception {      
        NXTRegulatedMotor az_motor = Motor.A;
        NXTRegulatedMotor att_motor = Motor.B;
        NXTRegulatedMotor claw_motor = Motor.C;

        // 1 - 3
        grab(az_motor, att_motor, claw_motor, 1, 0);
        Delay.msDelay(100);
        
        drop(az_motor, att_motor, claw_motor, 3, 0);
        Delay.msDelay(100);
        
        // 2 - 3
        grab(az_motor, att_motor, claw_motor, 2, 3);
        Delay.msDelay(100);
        
        drop(az_motor, att_motor, claw_motor, 3, 3);
        Delay.msDelay(100);
        
        // GOTO 5
        PositionUtils.goto_position(att_motor, az_motor, 5, 0); 
        
        // 3 - 4
        grab(az_motor, att_motor, claw_motor, 3, 3);
        Delay.msDelay(100);
        
        drop(az_motor, att_motor, claw_motor, 4, 0);
        Delay.msDelay(100);
        
        // 3 - 4
        grab(az_motor, att_motor, claw_motor, 3, 0);
        Delay.msDelay(100);
        
        drop(az_motor, att_motor, claw_motor, 4, 3);
        Delay.msDelay(100);
        
        // GOTO 5
        PositionUtils.goto_position(att_motor, az_motor, 5, 0); 
    }
}
