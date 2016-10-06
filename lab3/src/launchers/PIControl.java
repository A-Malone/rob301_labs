package launchers;

import common.MotorUtils;
import common.PIDController;
import common.SensorUtils;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;


public class PIControl
{
    static final float BASE_SPEED = 250;
    
    public static void main(String[] args) throws Exception
    {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;        
        EV3ColorSensor color = new EV3ColorSensor(SensorPort.S1);
        
        float[] sensor_reading;
        int sampleSize = color.sampleSize();
        sensor_reading = new float[sampleSize];
        
        PIDController controller = new PIDController(3.0f, 3.0f, 0.0f);
        
        left.setSpeed(BASE_SPEED);
        right.setSpeed(BASE_SPEED);
        left.forward();
        right.forward();
        
        long t_last = System.nanoTime();
        
        while(!Button.UP.isDown())
        {
            Thread.sleep(20);
        }
        
        while(!Button.ENTER.isDown())
        {
            if (Button.LEFT.isDown())
            {
                controller.zero_integral();
            }
            color.getRedMode().fetchSample(sensor_reading, 0);
            float val = sensor_reading[0];
            float error = SensorUtils.get_error(val);
            
            long t_curr = System.nanoTime();
            float correction = controller.step((t_curr - t_last)/(Math.pow(10, 9)), error);
            t_last = t_curr;
            
            float sleft = BASE_SPEED * (1.0f - correction);
            float sright = BASE_SPEED * (1.0f + correction);
            MotorUtils.setSpeeds(left, right, sleft, sright);
        }
        color.close();
    }
}

