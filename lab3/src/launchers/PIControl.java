package launchers;

import controllers.PIDController;
import controllers.SensorUtils;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;


public class PIControl
{
    static final float BASE_SPEED = 50;
    
    public static void main(String[] args) throws Exception
    {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;        
        EV3ColorSensor color = new EV3ColorSensor(SensorPort.S1);
        
        float[] sensor_reading;
        int sampleSize = color.sampleSize();
        sensor_reading = new float[sampleSize];
        
        PIDController controller = new PIDController(1.0f, 1.0f, 0.0f);
        
        long t_last = System.nanoTime();
        
        while(true)
        {
            color.getRedMode().fetchSample(sensor_reading, 0);
            float val = sensor_reading[0];
            float error = SensorUtils.get_error(val);
            
            long t_curr = System.nanoTime();
            float correction = controller.step((t_curr - t_last)/(Math.pow(10, 9)), error);
            t_last = t_curr;
            
            left.setSpeed(BASE_SPEED * (1 - correction));
            right.setSpeed(BASE_SPEED * (1 + correction));
        }
    }
}

