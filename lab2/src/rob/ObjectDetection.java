package rob;

import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.utility.Delay;

public class ObjectDetection {

    public static boolean check_position(
            NXTRegulatedMotor az_motor,
            NXTRegulatedMotor att_motor,
            EV3TouchSensor touch,
            int position)
    {
        PositionUtils.goto_position(att_motor, az_motor, position, 5);
        int sampleSize = touch.sampleSize();
        float[] touchsample = new float[sampleSize];        
        
        while(true)
        {
            touch.fetchSample(touchsample, 0);
            if (touchsample[0] > 0.1)
            {
                break;
            }
            Delay.msDelay(10);
        }
        
        return true;
    }

    public static void main(String[] args) throws Exception {      
        NXTRegulatedMotor az_motor = Motor.A;
        NXTRegulatedMotor att_motor = Motor.B;
        EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S1);

        for (int i = 1; i <= 5; i++)
        {
            check_position(az_motor, att_motor, touch, i);
        }
    }
}
