package common;

public class SensorUtils
{
    public static float MIN_VALUE = 0.05f;
    public static float MAX_VALUE = 0.33f;
    public static float MID_VALUE = (MAX_VALUE + MIN_VALUE)/2.0f;
    
    public static float get_error(float val)
    {
        return val - MID_VALUE;
    }
}
