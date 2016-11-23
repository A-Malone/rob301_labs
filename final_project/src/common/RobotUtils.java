package common;

public class RobotUtils
{
    static final float CM_TO_IN = 0.393701f;
    
    // Robot properties, in cm
    public static float wheel_track_inner = 9.0f;
    public static float wheel_track_outer = 14.5f;
    public static float wheel_track_avg = (wheel_track_outer + wheel_track_inner)/2.0f;
    public static float wheel_track_radius = wheel_track_avg / 2;
    
    public static float wheel_diameter = 5.5f;
}
