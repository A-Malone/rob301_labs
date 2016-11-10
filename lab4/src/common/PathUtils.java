package common;

import lejos.hardware.motor.NXTRegulatedMotor;

public class PathUtils {
	
	static float wheel_track_inner = 9.0f;
	static float wheel_track_outer = 14.5f;
	static float wheel_track_avg = (wheel_track_outer + wheel_track_inner)/2.0f;
	static float wheel_track_radius = wheel_track_avg / 2;
	
	static float wheel_radius = 5.5f/2;
	
	static float min_turn_radius = 25f;
	
	public static int get_angle_to(int x1, int y1, int x2, int y2)
	{
	    return (int)Math.round(Math.toDegrees(Math.atan2(y2-y1, x2-x1)));
	}
	
	public static int get_dist_to(int x1, int y1, int x2, int y2)
    {
        return (int)Math.round(Math.sqrt(Math.pow(y2-y1,2) + Math.pow(x2-x1, 2)));
    }
	
	public static float get_distance_travelled(NXTRegulatedMotor motor)
	{
	    return (float)Math.toRadians(motor.getTachoCount())*wheel_radius;
	}
	
	public static int get_smallest_equivalent_angle(int theta)
	{
	    return (int)Math.round(Math.toDegrees(
	            Math.atan2(Math.sin(Math.toRadians(theta)), Math.cos(Math.toRadians(theta)))
	            ));
	}
	
	public static void pivot(
	        NXTRegulatedMotor left,
	        NXTRegulatedMotor right, 
	        int theta
	        )
	{
		int theta_r = (int)Math.round(theta * wheel_track_radius / wheel_radius);
		synchronized_movement(left, right, -theta_r, theta_r);
	}
	
	public static void turn_with_radius(
            NXTRegulatedMotor left,
            NXTRegulatedMotor right, 
            float theta,
            float radius
            )
    {
	    radius = radius / 2;
	    theta = theta*2;
	    
	    float r_right = 0.0f;
	    float r_left = 0.0f;
	    if (theta < 0)
	    {
	        r_right = radius + wheel_track_radius / 2.0f;
	        r_left = radius - wheel_track_radius / 2.0f;
	    }
	    else
	    {
	        r_right = radius + wheel_track_radius / 2.0f;
            r_left = radius - wheel_track_radius / 2.0f;
	    }
	    
	    int v_right_0 = right.getSpeed();
	    int v_left_0 = left.getSpeed();
	    
	    // Calculate average speed
	    float omega = (float)(v_right_0) / radius;
	    
	    float v_right = omega * r_right;
	    float v_left = omega * r_left;
	    
	    right.setSpeed((int)(v_right));
	    left.setSpeed((int)(v_left));
        
        float dist_right = r_right*theta;
        float dist_left = r_left*theta;
        
        int theta_right = (int)Math.toDegrees(Math.round(dist_right / wheel_radius));
        int theta_left = (int)Math.toDegrees(Math.round(dist_left / wheel_radius));
       
        synchronized_movement(left, right, theta_left, theta_right);
    
        right.setSpeed(v_right_0);
        left.setSpeed(v_left_0);
    }
	
	public static void move_straight(
	        NXTRegulatedMotor left,
	        NXTRegulatedMotor right, 
	        int distance
	        )
	{
		int theta_r = (int)Math.toDegrees(Math.round((float)distance / wheel_radius));
		synchronized_movement(left, right, theta_r, theta_r);
	}
	
	public static void synchronized_movement(
	        NXTRegulatedMotor m1,
	        NXTRegulatedMotor m2,
	        int theta1,
	        int theta2
	        )
    {
        m1.synchronizeWith(new NXTRegulatedMotor[]{m2});

        m1.startSynchronization();
        m1.rotate(theta1);
        m2.rotate(theta2);
        m1.endSynchronization();

        m1.waitComplete();
        m2.waitComplete();
    }
}
