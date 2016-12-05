package tasks;

import common.BoardUtils.House;
import common.BoardUtils.Road;
import common.Robot;
import lejos.hardware.Button;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.RotateMoveController;
import lejos.robotics.navigation.Waypoint;

/**
 * Class that performs the pizza pickup task. The run_task method is the main
 * entrypoint and causes the robot to drive in a straight line and count houses.
 * PID control was tested for line following, but was determined to add more
 * complexity, and use of the interial navigation system was selected instead
 */
public class PizzaDropOffTask
{

    public static float HOUSE_DETECTION_EPSILON = 15f;
    public static int HOUSE_MIN_SPACING = 5;

    /**
     * Moves to the drop-off location and drops the pizza
     */
    public static boolean run_task(Robot robot, House house)
    {
        // STEP 0: Init
        boolean success = true;

        PoseProvider ppv = robot.navigator.getPoseProvider();
        SampleProvider range_finder = robot.ultra.getDistanceMode();

        SampleProvider average_range = new MeanFilter(range_finder, 3);
        float[] average_reading = new float[average_range.sampleSize()];

        // STEP 1: Turn the ultrasound to the correct side
        if (house.left)
        {
            robot.ULTRA_MOTOR.rotate(90);
        }
        else
        {
            robot.ULTRA_MOTOR.rotate(-90);
        }

        // STEP 2: Drive straight for the several centimeters to avoid counting
        // obstacles at the start of the road
        robot.pilot.travel(Road.start_offset, true);

        while (robot.pilot.isMoving())
        {
            // Break early condition
            if (Button.ESCAPE.isDown())
            {
                success = false;
                robot.pilot.stop();

                break;
            }
        }

        // STEP 3: Drive straight and count houses as you go
        if (success)
        {
            robot.pilot.travel(Road.start_offset, true);

            Pose last_house_pose = null;
            int house_count = 0;

            while (robot.pilot.isMoving())
            {
                // Break early condition
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                    robot.pilot.stop();

                    break;
                }

                Pose current = ppv.getPose();

                // If we detect a house, add it to our counter
                range_finder.fetchSample(average_reading, 0);
                if (!(Float.isInfinite(average_reading[0]) || Float.isNaN(average_reading[0])))
                {
                    if (Math.abs(average_reading[0] * 100 - House.DIST_TO_ROAD) < HOUSE_DETECTION_EPSILON
                            && (last_house_pose == null
                                    || current.distanceTo(last_house_pose.getLocation()) > HOUSE_MIN_SPACING))
                    {
                        System.out.println(average_reading[0]);
                        house_count++;
                        last_house_pose = current;

                        // If this house is the desired house, break from the
                        // loop
                        if (house_count == house.address)
                        {
                            break;
                        }
                    }
                }
            }

            // STEP 4: Turn and drop the pizza
            if (success)
            {
                if (house.left)
                {
                    robot.pilot.rotate(90);
                }
                else
                {
                    robot.pilot.rotate(-90);
                }

                success = success && robot.open_claw();
                success = success && robot.close_claw();
            }

            // STEP 5: Return to the start of the road
            if (success)
            {
                robot.navigator.goTo(new Waypoint(house.road.start.getLocation()));

                while (robot.navigator.isMoving())
                {
                    // Break early condition
                    if (Button.ESCAPE.isDown())
                    {
                        success = false;
                        robot.navigator.stop();
                        break;
                    }
                }
            }

            // STEP 6: Return the ultrasound to the starting position
            if (house.left)
            {
                robot.ULTRA_MOTOR.rotate(-90);
            }
            else
            {
                robot.ULTRA_MOTOR.rotate(90);
            }
        }

        return success;
    }

    public static float COLOR_MIN_VALUE = 0.05f;
    public static float COLOR_MAX_VALUE = 0.33f;
    public static float COLOR_MID_VALUE = (COLOR_MAX_VALUE + COLOR_MIN_VALUE) / 2.0f;

    private static float get_error(float val)
    {
        return val - COLOR_MID_VALUE;
    }
}
