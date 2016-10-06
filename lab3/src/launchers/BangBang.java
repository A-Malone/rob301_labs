package launchers;

import controllers.PIDController;
import controllers.SensorUtils;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class BangBang
{
    static final float BASE_SPEED = 50;
    static final float CORRECTION = 0.5f;

    public static void main(String[] args) throws Exception
    {
        NXTRegulatedMotor left = Motor.A;
        NXTRegulatedMotor right = Motor.B;
        EV3ColorSensor color = new EV3ColorSensor(SensorPort.S1);

        float[] sensor_reading;
        int sampleSize = color.sampleSize();
        sensor_reading = new float[sampleSize];
        
        left.setSpeed(BASE_SPEED);
        right.setSpeed(BASE_SPEED);
        left.forward();
        right.forward();

        while (true)
        {
            color.getRedMode().fetchSample(sensor_reading, 0);
            float val = sensor_reading[0];
            float error = SensorUtils.get_error(val);

            float correction = error > 0 ? -CORRECTION : CORRECTION;

            left.setSpeed(BASE_SPEED * (1 - correction));
            right.setSpeed(BASE_SPEED * (1 + correction));
        }
    }
}
