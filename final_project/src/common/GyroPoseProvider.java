package common;

import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.Pose;

/**
 * Pose Provider using the EV3GyroSensor
 */
public class GyroPoseProvider extends OdometryPoseProvider
{

    private EV3GyroSensor gyro;
    private SampleProvider gyro_angle;

    public GyroPoseProvider(MoveProvider mp, EV3GyroSensor g)
    {
        super(mp);
        gyro = g;
        
        // Setup gyro for readings
        gyro_angle = gyro.getAngleMode();
        
        // Do a quick reading to work out the kinks
        get_gyro_angle();
    }
    
    private float get_gyro_angle()
    {
        float[] sample = new float[gyro_angle.sampleSize()];
        gyro_angle.fetchSample(sample, 0);
        return sample[0];
    }

    public Pose getPose()
    {
        Pose temp = super.getPose();
        temp.setHeading(get_gyro_angle());
        return temp;
    }
}
