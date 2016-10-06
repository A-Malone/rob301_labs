package launchers;

import controllers.PIDController;
import controllers.SensorUtils;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class LagunaRacer
{
    static final float BASE_SPEED = 50;
    static final float D_SCALE = 1.0f;
    
    public static void main(String[] args) throws Exception
    {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;
        EV3ColorSensor color = new EV3ColorSensor(SensorPort.S1);
        
        float[] sensor_reading;
        int sampleSize = color.sampleSize();
        sensor_reading = new float[sampleSize];
        
        PIDController controller = new PIDController(1.0f, 1.0f, 1.0f);
        
        left.setSpeed(BASE_SPEED);
        right.setSpeed(BASE_SPEED);
        left.forward();
        right.forward();
        
        long t_last = System.nanoTime();
        
        while(true)
        {
            color.getRedMode().fetchSample(sensor_reading, 0);
            float val = sensor_reading[0];
            float error = SensorUtils.get_error(val);
            
            long t_curr = System.nanoTime();
            float correction = controller.step((t_curr - t_last)/(Math.pow(10, 9)), error);
            t_last = t_curr;
            
            float average_speed = (float)(BASE_SPEED / Math.exp(controller.get_derivative_error() / D_SCALE));
            left.setSpeed(average_speed * (1 - correction));
            right.setSpeed(average_speed * (1 + correction));
        }
    }
}