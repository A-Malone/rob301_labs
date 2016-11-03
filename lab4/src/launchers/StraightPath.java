package launchers;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.utility.Delay;

import common.PathUtils;

public class StraightPath {

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
        
        // Pivot
        int theta_goal = PathUtils.get_angle_to(0, 0, 200, 50) - theta;
        System.out.println(theta_goal);
        
        PathUtils.pivot(
                left, right, 
                theta_goal);
        theta = theta_goal;
               
        Delay.msDelay(500);
        
        // Straight line
        int dist = PathUtils.get_dist_to(0, 0, 200, 50);
        System.out.println(dist);
        PathUtils.move_straight(
                left, right,
                dist);
        Delay.msDelay(500);
        
        // Pivot
        PathUtils.pivot(
                left, right, 
                135 - theta);
    }

}
