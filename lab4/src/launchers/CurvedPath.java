package launchers;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.utility.Delay;

import common.PathUtils;

public class CurvedPath {

    public static void main(String[] args)  throws Exception {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;
        
        left.setSpeed(250);
        right.setSpeed(250);
        
        while(!Button.UP.isDown())
        {
            Thread.sleep(20);
        }
        
        int x = 0;
        int y = 0;
        int theta = 0;
        
        // Straight line
        int dist = 179;
        PathUtils.move_straight(
                left, right,
                dist);
        Delay.msDelay(100);
        
        // Wide turn
        PathUtils.turn_with_radius(
                left, right,
                (float)Math.toRadians(135), 29.289f);
    }

}
