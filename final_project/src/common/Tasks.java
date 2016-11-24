package common;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.geometry.Point;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.RotateMoveController;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;

public class Tasks
{
    // ------------------------------------------------------------
    // ---- PIZZA PICKUP TASK
    // ------------------------------------------------------------

    // The initial width of the band to scan for the pizza
    private static int PIZZA_SCAN_BAND_WIDTH = 15;

    // The initial estimate of the pizza's distance
    private static float PIZZA_DISTANCE_ESTIMATE = 100;

    // Ideal distance to pizza for pickup
    private static float PIZZA_PICKUP_DISTANCE = 10;

    // Back-up distance to pickup pizza
    private static float PIZZA_BACKUP_DISTANCE = 5;

    // STEP 1: Zero in on pizza
    public static boolean move_to_pizza_task(Navigator nav, RotateMoveController pilot, EV3UltrasonicSensor ultra)
    {
        // STEP 1.0: INIT
        boolean success = true;

        PoseProvider ppv = nav.getPoseProvider();
        SampleProvider range_finder = ultra.getDistanceMode();

        // Resulting parameters
        float pizza_relative_heading = 0;
        float pizza_dist = 0;

        // Search parameters
        int search_width = PIZZA_SCAN_BAND_WIDTH;
        float last_estimated_distance = PIZZA_DISTANCE_ESTIMATE;

        while (success)
        {
            // STEP 1.1: Identify heading to pizza
            RangeFinderScan scan_results = RangeFinderScan.scan(pilot, ppv, range_finder, search_width);
            success = success && (scan_results != null);

            if (success)
            {
                // Linear scan to find measurements best matching pizza
                float min_error = Float.MAX_VALUE;

                float best_distance = 0;
                int best_index = -1;

                for (int i = 0; i < scan_results.range_spectrum.length; i++)
                {
                    if (Math.abs(scan_results.range_spectrum[i][0] - last_estimated_distance) < min_error)
                    {
                        best_distance = scan_results.range_spectrum[i][0];
                        best_index = i;
                    }
                }

                // Check if we found the pizza
                if (best_index != -1)
                {   
                    pizza_relative_heading = scan_results.index_to_relative_heading(best_index);
                    pizza_dist = best_distance;
                    System.out.println("Pizza found " + pizza_relative_heading + ", " + ppv.getPose().getHeading());
                }
                else
                {
                    System.out.println("Pizza not found, exiting");
                    success = false;
                }
            }

            // STEP 1.2: Rotate towards pizza
            if (success)
            {
                pilot.rotate(-pizza_relative_heading);
            }

            // STEP 1.2: Move towards pizza, checking to make sure we don't lose
            // alignment, or get too close.
            pilot.travel(pizza_dist * 100 - PIZZA_PICKUP_DISTANCE, true);
            SampleProvider average_range = new MeanFilter(range_finder, 3);

            while (pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    pilot.stop();
                    break;
                }

                float est_goal_distance = last_estimated_distance - pilot.getMovement().getDistanceTraveled();

                float[] sensor_reading = new float[average_range.sampleSize()];
                average_range.fetchSample(sensor_reading, 0);

                // If we're off by a factor of 2, re-locate pizza
                if (Math.abs(sensor_reading[0] - est_goal_distance) > est_goal_distance / 2)
                {
                    // We've lost lock-on on the pizza, re-acquire, and update
                    // our estimate of where it is
                    pilot.stop();
                    last_estimated_distance = est_goal_distance;
                    break;
                }
            }
        }
        return success;
    }

    /**
     * Moves to the pizza from a starting location roughly oriented towards the
     * pizza, and picks it up using the claw arm.
     */
    public static boolean pizza_pickup_macro_task(Navigator nav, RotateMoveController pilot, EV3UltrasonicSensor ultra,
            Motor claw)
    {
        // STEP 0: Init
        boolean success = true;

        success = success && move_to_pizza_task(nav, pilot, ultra);

        // STEP 2: Turn around to position claw
        pilot.rotate(180);

        // STEP 3: Back up to pizza
        pilot.travel(-PIZZA_BACKUP_DISTANCE);

        // STEP 4: Close claw
        // TODO: Figure out what needs to be done to close the claw

        return success;
    }

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
            // Step 1.1: Try moving directly to pose if we're not doing anything else
            if(!pilot.isMoving() && !nav.isMoving())
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
                        float effort_score = 3 - scan_results.range_spectrum[i][0] + Math.abs(desired_relative_heading - scan_results.index_to_relative_heading(i)) / 180;
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
                        //Point next_waypoint = current.pointAt(MOVEMENT_STEP_SIZE, chosen_direction);
                        //nav.addWaypoint(new Waypoint(next_waypoint));
                        
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
