package launchers;

import common.MotorUtils;
import common.PIDController;
import common.SensorUtils;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;


public class WallFollowing
{
    static final float BASE_SPEED = 100;
    
    public static void main(String[] args) throws Exception
    {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;
        EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S2);
        
        float[] sensor_reading;
        int sampleSize = sonic.sampleSize();
        sensor_reading = new float[sampleSize];
        
        PIDController controller = new PIDController(0.05f, 0.0f, 0.0f);
        
        left.setSpeed(BASE_SPEED);
        right.setSpeed(BASE_SPEED);
        left.forward();
        right.forward();
        
        while(!Button.UP.isDown())
        {
            Thread.sleep(20);
        }
        
        long t_last = System.nanoTime();
        
        while(!Button.ENTER.isDown())
        {
            sensor_reading = new float[sampleSize];
            sonic.fetchSample(sensor_reading, 0);
            float error = SensorUtils.get_error(sensor_reading[0]*100);
            
            String dist = String.format("Dist: %2.2f", sensor_reading[0]*100);
            
            // Print the current state of the control variables
            LCD.drawString(dist, 0, 0);
            
            long t_curr = System.nanoTime();
            float correction = controller.step((t_curr - t_last)/(Math.pow(10, 9)), error);
            t_last = t_curr;
            
            float sleft = BASE_SPEED * (1.0f - correction);
            float sright = BASE_SPEED * (1.0f + correction);
            MotorUtils.setSpeeds(left, right, sleft, sright);
        }
        sonic.close();
    }
}
