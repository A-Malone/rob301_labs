package launchers;

import java.util.ArrayList;

import common.PathUtils;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.utility.Delay;

public class FourPoint {

    public static void main(String[] args) throws Exception {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;

        while (!Button.UP.isDown()) {
            Thread.sleep(20);
        }

        int[] start = { 0, 0, 0 };
        int[] xa = { 100, 0, 90 };
        int[] xb = { 100, 100, 180 };
        int[] xc = { 0, 100, 270 };

        int[][] states = { start, xa, xb, xc, start };

        int[] curr = states[0];

        for (int i = 1; i < states.length; i++) {
            int[] goal = states[i];

            // Pivot
            int theta_path = PathUtils.get_angle_to(curr[0], curr[1], goal[0], goal[1]) - curr[2];
            if (theta_path == 360 || theta_path == -360)
            {
                theta_path = 0;
            }
            
            System.out.println(theta_path);
            left.setSpeed(100);
            right.setSpeed(100);
            PathUtils.pivot(left, right, theta_path);
            Delay.msDelay(200);

            // Straight line
            int dist = PathUtils.get_dist_to(curr[0], curr[1], goal[0], goal[1]);           
            
            left.setSpeed(350);
            right.setSpeed(350);
            PathUtils.move_straight(left, right, dist);
            Delay.msDelay(200);

            // Pivot
            int theta_goal = goal[2] - (theta_path + curr[2]);
            if (theta_goal == 360 || theta_goal == -360)
            {
                theta_goal = 0;
            }
            left.setSpeed(100);
            right.setSpeed(100);
            PathUtils.pivot(left, right, theta_goal);

            curr = goal;
            Delay.msDelay(200);
        }
    }

}
