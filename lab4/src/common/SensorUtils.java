package common;

public class SensorUtils
{  
    public static float MAX_ERROR = 30.0f;
    public static float MID_VALUE = 20.0f;
    
    public static float get_error(float val)
    {        
        return Math.max(-MAX_ERROR, Math.min(MAX_ERROR, val - MID_VALUE));
    }
}
