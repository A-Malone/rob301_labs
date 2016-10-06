package launchers;

import common.MotorUtils;
import common.SensorUtils;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;

public class BangBang
{
    static final float BASE_SPEED = 100;
    static final float CORRECTION = 1.0f;

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

        while (!Button.ENTER.isDown())
        {
            color.getRedMode().fetchSample(sensor_reading, 0);
            float val = sensor_reading[0];
            float error = SensorUtils.get_error(val);

            float correction = error > 0.0f ? -CORRECTION : CORRECTION;

            MotorUtils.setSpeeds(left, right, BASE_SPEED * (1.0f - correction), BASE_SPEED * (1.0f + correction));
        }
        color.close();
    }
}
