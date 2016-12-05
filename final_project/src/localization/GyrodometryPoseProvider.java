package localization;

import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Move;
import lejos.robotics.navigation.MoveListener;
import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.Pose;

/**
 * Pose Provider using the EV3GyroSensor and the concept of gyrodometry, which
 * corrects sudden changes in the angle of the robot using the gyroscope. This
 * is supplanted by the DifferentialKalmanPoseProvider.
 */
public class GyrodometryPoseProvider implements PoseProvider, MoveListener, SampleProvider
{
    private static final float gyro_threshold = 2f;

    private EV3GyroSensor gyro;
    private SampleProvider gyro_angle;

    private float x = 0, y = 0, heading = 0;
    private float gyro_angle0, odo_angle0, distance0;
    MoveProvider mp;
    boolean current = true;

    public GyrodometryPoseProvider(MoveProvider mp, EV3GyroSensor g)
    {
        mp.addMoveListener(this);

        gyro = g;

        // Setup gyro for readings
        gyro_angle = gyro.getAngleMode();

        // Do a quick reading to work out the kinks
        get_gyro_angle();

        this.mp = mp;
    }

    private float get_gyro_angle()
    {
        float[] sample = new float[gyro_angle.sampleSize()];
        gyro_angle.fetchSample(sample, 0);
        return sample[0];
    }

    public synchronized Pose getPose()
    {
        if (!current)
        {
            updatePose(mp.getMovement());
        }
        return new Pose(x, y, heading);
    }

    public synchronized void moveStarted(Move move, MoveProvider mp)
    {
        gyro_angle0 = get_gyro_angle();
        odo_angle0 = 0;
        distance0 = 0;
        current = false;
        this.mp = mp;
    }

    public synchronized void setPose(Pose aPose)
    {
        setPosition(aPose.getLocation());
        setHeading(aPose.getHeading());
    }

    public void moveStopped(Move move, MoveProvider mp)
    {
        updatePose(move);
    }

    private synchronized void updatePose(Move event)
    {
        float odo_angle_delta = event.getAngleTurned() - odo_angle0;

        float gyro_angle = get_gyro_angle();
        float gyro_delta = gyro_angle - gyro_angle0;

        float angle_delta = (gyro_delta > gyro_threshold) ? gyro_delta : odo_angle_delta;

        float distance = event.getDistanceTraveled() - distance0;
        double dx = 0, dy = 0;
        double headingRad = (Math.toRadians(heading));

        if (event.getMoveType() == Move.MoveType.TRAVEL || Math.abs(angle_delta) < 0.2f)
        {
            dx = (distance) * (float) Math.cos(headingRad);
            dy = (distance) * (float) Math.sin(headingRad);
        }
        else if (event.getMoveType() == Move.MoveType.ARC)
        {
            double turnRad = Math.toRadians(angle_delta);
            double radius = distance / turnRad;
            dy = radius * (Math.cos(headingRad) - Math.cos(headingRad + turnRad));
            dx = radius * (Math.sin(headingRad + turnRad) - Math.sin(headingRad));
        }

        x += dx;
        y += dy;
        heading = normalize(heading + angle_delta); // keep angle between -180
                                                    // and 180
        odo_angle0 = event.getAngleTurned();
        distance0 = event.getDistanceTraveled();
        current = !event.isMoving();
    }

    private float normalize(float angle)
    {
        float a = angle;
        while (a > 180)
            a -= 360;
        while (a < -180)
            a += 360;
        return a;
    }

    private void setPosition(Point p)
    {
        x = p.x;
        y = p.y;
        current = true;
    }

    private void setHeading(float heading)
    {
        this.heading = heading;
        current = true;
    }

    @Override
    public int sampleSize()
    {
        return 3;
    }

    @Override
    public void fetchSample(float[] sample, int offset)
    {
        if (!current)
        {
            updatePose(mp.getMovement());
        }
        sample[offset + 0] = x;
        sample[offset + 1] = y;
        sample[offset + 2] = heading;
    }
}
