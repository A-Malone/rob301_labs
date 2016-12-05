package common;

import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.RotateMoveController;
import lejos.utility.Delay;

/**
 * Failed attempt to perform a radial range scan in order to identify obstacles
 * and possible paths
 */
public class RangeFinderScan
{
    public static final float MAX_SENSOR_RANGE = 9f;

    public int scan_bandwidth;
    public float[][] range_spectrum;
    public float[] normalized_spectrum;
    public Pose origin;

    private int scan_band_halfwidth;

    public RangeFinderScan(Pose origin, int scan_bandwidth)
    {
        // Create a vector to store the radial distances and number of samples
        // detected during the pivot
        range_spectrum = new float[scan_bandwidth + 1][2];

        this.scan_bandwidth = scan_bandwidth;
        this.scan_band_halfwidth = scan_bandwidth / 2;
        this.origin = origin;
    }

    public boolean heading_in_scan_range(float heading)
    {
        return Math.abs(heading - origin.getHeading()) < scan_band_halfwidth;
    }

    public void add_reading(float heading, float range)
    {
        int index = relative_heading_to_index(heading);
        if (index != -1)
        {
            range_spectrum[index][0] += range;
            range_spectrum[index][1] += 1;
        }
    }

    private void normalize()
    {
        normalized_spectrum = new float[range_spectrum.length];

        // Normalize to the number of readings per bucket
        for (int i = 0; i < range_spectrum.length; i++)
        {
            float value = (range_spectrum[i][0] == Float.POSITIVE_INFINITY) ? MAX_SENSOR_RANGE : range_spectrum[i][0];
            float count = range_spectrum[i][1];
            normalized_spectrum[i] = (count != 0f) ? value / count : MAX_SENSOR_RANGE;
        }
    }

    private void smooth(int radius)
    {
        normalized_spectrum = new float[range_spectrum.length];

        float win_total = 0;
        float win_count = 0;

        // Build window
        for (int i = 0; i <= radius && i < range_spectrum.length; i++)
        {
            // Check if we should even add this value
            if (!(range_spectrum[i][1] == 0 || Float.isInfinite(range_spectrum[i][0])
                    || Float.isNaN(range_spectrum[i][0])))
            {
                win_total += range_spectrum[i][0];
                win_count += range_spectrum[i][1];
            }

        }

        for (int i = 0; i < range_spectrum.length; i++)
        {
            if (i >= radius)
            {
                // Check if we should even add this value
                if (!(range_spectrum[i - radius][1] == 0 || Float.isInfinite(range_spectrum[i - radius][0])
                        || Float.isNaN(range_spectrum[i - radius][0])))
                {
                    win_total -= range_spectrum[i - radius][0];
                    win_count -= range_spectrum[i - radius][1];
                }
            }

            if (i < range_spectrum.length - radius)
            {
                // Check if we should even add this value
                if (!(range_spectrum[i + radius][1] == 0 || Float.isInfinite(range_spectrum[i + radius][0])
                        || Float.isNaN(range_spectrum[i + radius][0])))
                {
                    win_total += range_spectrum[i + radius][0];
                    win_count += range_spectrum[i + radius][1];
                }
            }

            while (!(Button.ENTER.isDown() || Button.ESCAPE.isDown()))
            {
                Delay.msDelay(50);
            }
            if (Button.ESCAPE.isDown())
            {
                break;
            }
            System.out.println(i + " " + win_total + " " + win_count);

            normalized_spectrum[i] = (win_count == 0) ? MAX_SENSOR_RANGE : win_total / win_count;
        }
    }

    public float index_to_relative_heading(int index)
    {
        return (float) (index - scan_band_halfwidth);
    }

    public int relative_heading_to_index(float heading)
    {
        int index = (int) Math.round(heading) + scan_band_halfwidth;
        if (index < 0 || index >= range_spectrum.length)
        {
            return -1;
        }
        return index;
    }

    // ---- STATIC FUNCTIONS

    // Utility function
    private static boolean rotate_and_poll(RangeFinderScan scan_results, RotateMoveController pilot, PoseProvider ppv,
            SampleProvider range_finder, int angle)
    {
        boolean success = true;

        float[] sensor_reading = new float[range_finder.sampleSize()];

        pilot.rotate(angle, true);

        // Poll for ultrasound values
        while (pilot.isMoving())
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
                success = false;
                pilot.stop();
                break;
            }

            // Poll for results
            float heading = ppv.getPose().getHeading();

            if (scan_results.heading_in_scan_range(heading))
            {
                range_finder.fetchSample(sensor_reading, 0);
                scan_results.add_reading(heading, sensor_reading[0]);
            }
        }

        return success;
    }

    // Static constructor
    public static RangeFinderScan scan(Robot robot, int scan_bandwidth)
    {
        boolean success = true;

        SampleProvider range_finder = new MeanFilter(robot.ultra.getDistanceMode(), 3);

        PoseProvider ppv = robot.pose_provider;
        Pose start_pose = ppv.getPose();

        RangeFinderScan scan_results = new RangeFinderScan(start_pose, scan_bandwidth);

        int scan_band_halfwidth = scan_bandwidth / 2;

        // Rotate left by scan_band_halfwidth
        success = success && rotate_and_poll(scan_results, robot.pilot, ppv, range_finder, scan_band_halfwidth);

        // Rotate right by 2*scan_band
        success = success && rotate_and_poll(scan_results, robot.pilot, ppv, range_finder, -scan_bandwidth);

        // Rotate left by scan_band
        success = success && rotate_and_poll(scan_results, robot.pilot, ppv, range_finder, scan_band_halfwidth);

        // Return the normalized results or null depending on success
        if (success)
        {
            scan_results.normalize();
            // scan_results.smooth(2);
            return scan_results;
        }
        else
        {
            return null;
        }
    }
}
