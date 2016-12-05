package tasks;

import common.RangeFinderScan;
import common.Robot;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
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

    private static final float CRITICAL_OBSTACLE_DISTANCE = 20;
    private static final float MOVEMENT_STEP_SIZE = 40;
    private static final int MOVEMENT_SCAN_BAND_WIDTH = 180;

    public static boolean navigate_to_pose_task(Robot robot, Pose destination)
    {
        // STEP 0: INIT
        boolean success = true;

        PoseProvider ppv = robot.navigator.getPoseProvider();

        // Use this for getting distance to obstacles when moving
        SampleProvider range_finder = robot.ultra.getDistanceMode();
        float[] sensor_reading = new float[range_finder.sampleSize()];
        
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
                // Start scanning left
                boolean found_path = false;
                robot.pilot.rotate(MOVEMENT_SCAN_BAND_WIDTH/2, true);

                while (robot.pilot.isMoving())
                {
                    // Break early condition
                    if (Button.ESCAPE.isDown())
                    {
                        success = false;
                        robot.pilot.stop();
                        break;
                    }

                    range_finder.fetchSample(sensor_reading, 0);
                    float range = sensor_reading[0] * 100;

                    if (range > CRITICAL_OBSTACLE_DISTANCE*3)
                    {
                        robot.pilot.stop();
                        robot.pilot.rotate(5);
                        found_path = true;
                        break;
                    }
                }
                
                // Scan right
                if (success && !found_path)
                {
                    robot.pilot.rotate(-MOVEMENT_SCAN_BAND_WIDTH, true);
    
                    while (robot.pilot.isMoving())
                    {
                        // Break early condition
                        if (Button.ESCAPE.isDown())
                        {
                            success = false;
                            robot.pilot.stop();
                            break;
                        }
    
                        range_finder.fetchSample(sensor_reading, 0);
                        float range = sensor_reading[0] * 100;
    
                        if (range > CRITICAL_OBSTACLE_DISTANCE*2)
                        {
                            robot.pilot.stop();
                            robot.pilot.rotate(-5);
                            found_path = true;
                            break;
                        }
                    }
                }
                
                // If we can't find a path quit
                success = success && found_path;
                
                // If we found a path start moving in that direction
                if(success && found_path)
                {
                    robot.pilot.travel(MOVEMENT_STEP_SIZE, true);
                }
            }
        }

        return success;
    }
}
