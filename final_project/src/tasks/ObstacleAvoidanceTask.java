package tasks;

import common.RangeFinderScan;
import common.Robot;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;

public class ObstacleAvoidanceTask
{
    // ------------------------------------------------------------
    // ---- OBBSTACLE robot.navigatorIGATION TASK
    // ------------------------------------------------------------

    private static final float CRITICAL_OBSTACLE_DISTANCE = 10;
    private static final float MOVEMENT_STEP_SIZE = 50;
    private static final int MOVEMENT_SCAN_BAND_WIDTH = 180;

    public static boolean navigate_to_pose_task(Robot robot,Pose destination)
    {
        // STEP 0: INIT
        boolean success = true;

        PoseProvider ppv = robot.navigator.getPoseProvider();

        // Use this for getting distance to obstacles when moving
        SampleProvider average_range = new MeanFilter(robot.ultra.getDistanceMode(), 5);
        float[] average_reading = new float[average_range.sampleSize()];

        robot.navigator.addWaypoint(new Waypoint(destination));
        robot.navigator.followPath();

        while (success && !robot.navigator.pathCompleted())
        {
            // Step 1.1: Try moving directly to pose if we're not doing anything
            // else
            if (!robot.pilot.isMoving() && !robot.navigator.isMoving())
            {
                robot.navigator.followPath();
            }

            // Step 1.2: If we're about to run into an obstacle activate
            // avoidance
            while (robot.navigator.isMoving() || robot.pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    if (robot.navigator.isMoving())
                    {
                        robot.navigator.stop();
                    }
                    if (robot.pilot.isMoving())
                    {
                        robot.pilot.stop();
                    }

                    break;
                }

                average_range.fetchSample(average_reading, 0);

                // If we're about to run into an obstacle, stop and begin
                // avoidance routine
                if (average_reading[0] < CRITICAL_OBSTACLE_DISTANCE)
                {
                    if (robot.navigator.isMoving())
                    {
                        robot.navigator.stop();
                    }
                    if (robot.pilot.isMoving())
                    {
                        robot.pilot.stop();
                    }
                    break;
                }
            }

            // Step 1.3: We've encountered an obstacle, run scan
            if (success && !robot.navigator.pathCompleted())
            {
                // Perform scan of the surroundings
                RangeFinderScan scan_results = RangeFinderScan.scan(robot, MOVEMENT_SCAN_BAND_WIDTH);
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

                        // Add another waypoint to the robot.navigator
                        // Point next_waypoint =
                        // current.pointAt(MOVEMENT_STEP_SIZE,
                        // chosen_direction);
                        // robot.navigator.addWaypoint(new Waypoint(next_waypoint));

                        // Start moving in the direction of the best path
                        robot.pilot.rotate(chosen_direction);
                        robot.pilot.travel(MOVEMENT_STEP_SIZE, true);
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
