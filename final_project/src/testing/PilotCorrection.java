package testing;

import common.Robot;
import lejos.hardware.Button;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import lejos.utility.PilotProps;

/**
 * Performs golden section search to find the value for the track width which
 * provides the most error-free rotation according to the gyro.
 */
public class PilotCorrection
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT

        // Get the gyro
        EV3GyroSensor gyro = new EV3GyroSensor(Robot.GYRO_PORT);
        SampleProvider direction = gyro.getAngleMode();

        System.out.println("Press ENTER to start");
        Button.ENTER.waitForPress();

        // Search parameters
        float turn_angle = 720f;

        // Define the search range
        float search_lower = Robot.wheel_track_width / 2f;
        float search_upper = Robot.wheel_track_width * 1.5f;
        float search_current = Robot.wheel_track_width;

        boolean success = true;

        for (int i = 0; i < 10; ++i)
        {
            // Create the pilot based on the current estimate of the parameters
            DifferentialPilot pilot = new DifferentialPilot(Robot.wheel_diameter, search_current, Robot.LEFT_MOTOR, Robot.RIGHT_MOTOR);
            pilot.setRotateSpeed(180 / 4);

            // Starting gyro reading
            float[] sample = new float[direction.sampleSize() + 1];
            direction.fetchSample(sample, 0);

            pilot.rotate(turn_angle, true);

            while (pilot.isMoving() && success)
            {
                success = success && !lejos.hardware.Button.ESCAPE.isDown();
            }

            if (!success)
            {
                break;
            }

            // Binary search for the best value
            direction.fetchSample(sample, 1);
            float error = sample[1] - sample[0] - turn_angle;

            if (error < 0)
            {
                search_upper = search_current;
                search_current = (search_upper - search_lower) / 2;
            }
            else if (error > 0)
            {
                search_lower = search_current;
                search_current = (search_upper - search_lower) / 2;
            }
            else
            {
                break;
            }
        }
        
        // If we found the value, then store the parameter to be that value
        if (success)
        {
            PilotProps p = new PilotProps();
            p.setProperty(PilotProps.KEY_TRACKWIDTH, String.valueOf(search_current));
            p.storePersistentValues();
            
            System.out.println("Tw: " + search_current);
        }
        else
        {
            System.out.println("User exit");
        }
        
        // Wait to exit
        while (!lejos.hardware.Button.ENTER.isDown())
        {
            Delay.msDelay(20);
        }
    }
}
