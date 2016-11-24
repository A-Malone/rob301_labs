package common;

public class RobotUtils
{
    static final float CM_TO_IN = 0.393701f;
    
    // Robot properties, in cm
    
    public static float wheel_width = 2.8f;
    public static float wheel_track_outer = 14.7f;
    public static float wheel_track_width = (wheel_track_outer - wheel_width);
    public static float wheel_track_corrected = (wheel_track_outer - 0.6f*wheel_width);
    public static float wheel_track_radius = wheel_track_width / 2;
    
    public static float wheel_diameter = 5.6f;
}
