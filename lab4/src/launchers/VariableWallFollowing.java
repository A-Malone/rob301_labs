package launchers;

import common.MotorUtils;
import common.PIDController;
import common.PathUtils;
import common.SensorUtils;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;


public class VariableWallFollowing
{
    static final float BASE_SPEED = 150;
    
    public static void main(String[] args) throws Exception
    {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;
        EV3UltrasonicSensor sonic = new EV3UltrasonicSensor(SensorPort.S2);
        
        float[] sensor_reading;
        int sampleSize = sonic.sampleSize();
        sensor_reading = new float[sampleSize];
        
        PIDController controller = new PIDController(0.075f, 0.0f, 0.0f);
        
        left.setSpeed(BASE_SPEED);
        right.setSpeed(BASE_SPEED);        
        
        while(!Button.UP.isDown())
        {
            Thread.sleep(20);
        }
        
        left.forward();
        right.forward();
        
        long t_last = System.nanoTime();
        
        while(!Button.ESCAPE.isDown())
        {
            float travelled = (PathUtils.get_distance_travelled(left) + PathUtils.get_distance_travelled(right))/2;            
            if (travelled > 300.0)
            {
                left.stop();
                right.stop();
            }
            else if (travelled > 250.0)
            {
                controller.set_proportional(0.075f);
            }
            else if (travelled > 200.0)
            {
                controller.set_proportional(0.025f);
                SensorUtils.MID_VALUE = 20.0f;
            }
            else if(travelled > 100.0)
            {
                SensorUtils.MID_VALUE = 30.0f;
            }
            
            sensor_reading = new float[sampleSize];
            sonic.fetchSample(sensor_reading, 0);
            float error = SensorUtils.get_error(sensor_reading[0]*100);
            
            String dist_str = String.format("Dist: %2.2f", sensor_reading[0]*100);
            String target_str = String.format("Target: %2.2f", SensorUtils.MID_VALUE);
            String error_str = String.format("Error: %2.2f", error);
            
            // Print the current state of the control variables
            LCD.drawString(dist_str, 0, 0);
            LCD.drawString(target_str, 0, 1);
            LCD.drawString(error_str, 0, 2);
            
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
