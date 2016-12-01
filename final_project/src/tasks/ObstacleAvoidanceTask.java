package tasks;

import common.RangeFinderScan;
import lejos.hardware.Button;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.RotateMoveController;
import lejos.robotics.navigation.Waypoint;

public class ObstacleAvoidanceTask
{
    // ------------------------------------------------------------
    // ---- OBBSTACLE NAVIGATION TASK
    // ------------------------------------------------------------

    private static final float CRITICAL_OBSTACLE_DISTANCE = 10;
    private static final float MOVEMENT_STEP_SIZE = 50;
    private static final int MOVEMENT_SCAN_BAND_WIDTH = 180;

    public static boolean navigate_to_pose_task(Navigator nav, RotateMoveController pilot, EV3UltrasonicSensor ultra,
            Pose destination)
    {
        // STEP 0: INIT
        boolean success = true;

        PoseProvider ppv = nav.getPoseProvider();

        // Use this for ultrasound scans
        SampleProvider range_finder = ultra.getDistanceMode();

        // Use this for getting distance to obstacles when moving
        SampleProvider average_range = new MeanFilter(range_finder, 5);
        float[] average_reading = new float[average_range.sampleSize()];

        nav.addWaypoint(new Waypoint(destination));
        nav.followPath();

        while (success && !nav.pathCompleted())
        {
            // Step 1.1: Try moving directly to pose if we're not doing anything
            // else
            if (!pilot.isMoving() && !nav.isMoving())
            {
                nav.followPath();
            }

            // Step 1.2: If we're about to run into an obstacle activate
            // avoidance
            while (nav.isMoving() || pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    if (nav.isMoving())
                    {
                        nav.stop();
                    }
                    if (pilot.isMoving())
                    {
                        pilot.stop();
                    }

                    break;
                }

                average_range.fetchSample(average_reading, 0);

                // If we're about to run into an obstacle, stop and begin
                // avoidance routine
                if (average_reading[0] < CRITICAL_OBSTACLE_DISTANCE)
                {
                    if (nav.isMoving())
                    {
                        nav.stop();
                    }
                    if (pilot.isMoving())
                    {
                        pilot.stop();
                    }
                    break;
                }
            }

            // Step 1.3: We've encountered an obstacle, run scan
            if (success && !nav.pathCompleted())
            {
                // Perform scan of the surroundings
                RangeFinderScan scan_results = RangeFinderScan.scan(pilot, ppv, range_finder, MOVEMENT_SCAN_BAND_WIDTH);
                success = success && (scan_results != null);

                if (success)
                {
                    Pose current = ppv.getPose();
                    float desired_relative_heading = current.relativeBearing(destination.getLocation());

                    // Linear scan to find the best angle to travel
                    float min_effort = Float.MAX_VALUE;
                    int best_index = -1;

                    for (int i = 0; i < scan_results.range_spectrum.length; i++)
                    {
                        float effort_score = 3 - scan_results.range_spectrum[i][0]
                                + Math.abs(desired_relative_heading - scan_results.index_to_relative_heading(i)) / 180;
                        if (effort_score < min_effort)
                        {
                            min_effort = effort_score;
                            best_index = i;
                        }
                    }

                    if (best_index != -1)
                    {
                        float chosen_direction = scan_results.index_to_relative_heading(best_index);

                        // Add another waypoint to the nav
                        // Point next_waypoint =
                        // current.pointAt(MOVEMENT_STEP_SIZE,
                        // chosen_direction);
                        // nav.addWaypoint(new Waypoint(next_waypoint));

                        // Start moving in the direction of the best path
                        pilot.rotate(chosen_direction);
                        pilot.travel(MOVEMENT_STEP_SIZE, true);
                    }
                    else
                    {
                        success = false;
                    }
                }
            }
        }

        return success;
    }
}
