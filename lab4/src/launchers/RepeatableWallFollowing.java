package launchers;

import java.util.ArrayList;

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
import lejos.utility.Delay;


public class RepeatableWallFollowing
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
        
        PIDController controller = new PIDController(0.025f, 0.0f, 0.0f);
        
        left.setSpeed(BASE_SPEED);
        right.setSpeed(BASE_SPEED);        
        
        while(!Button.UP.isDown())
        {
            Thread.sleep(20);
        }
        
        left.forward();
        right.forward();
        
        long t_start = System.nanoTime();  
        long t_last = t_start;
        
        ArrayList<double[]> inputs = new ArrayList<double[]>();
        inputs.add(new double[]{0,0});        
        
        int start_dleft = left.getTachoCount();
        int start_dright = right.getTachoCount();
        
        while(!Button.ESCAPE.isDown())
        {
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
            
            inputs.add(new double[]{t_curr-t_start, correction});
            
            float sleft = BASE_SPEED * (1.0f - correction);
            float sright = BASE_SPEED * (1.0f + correction);
            MotorUtils.setSpeeds(left, right, sleft, sright);           
            
        }       
        
        left.stop();
        right.stop();
        
        while(!Button.UP.isDown())
        {
            Thread.sleep(20);
        }
        
        t_last = System.nanoTime();
        t_start = t_last;
        int i = 0;
        
        start_dleft = left.getTachoCount();
        start_dright = right.getTachoCount();
        
        left.setSpeed(BASE_SPEED);
        right.setSpeed(BASE_SPEED);
        
        left.forward();
        right.forward();
        
        while(!Button.ESCAPE.isDown() && i < inputs.size()-1)
        {   
            double[] past = inputs.get(i);
            
            float sleft = BASE_SPEED * (float)(1.0 - past[1]);
            float sright = BASE_SPEED * (float)(1.0 + past[1]);
            
            String l_str = String.format("L - V%2.2f", sleft);
            String r_str = String.format("R - V%2.2f", sright);
            
            // Print the current state of the control variables
            LCD.drawString(l_str, 0, 0);
            LCD.drawString(r_str, 0, 1);
            
            MotorUtils.setSpeeds(left, right, sleft, sright);
            
            i++;
            long t_curr = System.nanoTime() - t_start;            
            Delay.nsDelay((long)(inputs.get(i)[0] - t_curr));
        }
        
        sonic.close();
    }
}
