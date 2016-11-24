package common;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.RotateMoveController;
import lejos.utility.Delay;

public class Tasks
{
    // ------------------------------------------------------------
    // ---- PIZZA PICKUP TASK
    // ------------------------------------------------------------

    // The initial width of the band to scan for the pizza
    private static int SCAN_BAND_WIDTH = 10;

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
        int search_width = SCAN_BAND_WIDTH;
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
                    pizza_dist = min_error;
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
                pilot.rotate(pizza_relative_heading);
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
    
    public static boolean navigate_to_pose_task(Navigator nav, RotateMoveController pilot, EV3UltrasonicSensor ultra)
    {
        // STEP 0: INIT
        boolean success = true;

        PoseProvider ppv = nav.getPoseProvider();
        SampleProvider range_finder = ultra.getDistanceMode();

                
        
        return success;
    }
    
}
