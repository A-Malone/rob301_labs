package launchers;

import common.BoardUtils.House;
import common.BoardUtils.PizzaPedestal;
import common.BoardUtils.Road;
import common.Robot;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;
import lejos.utility.TextMenu;
import localization.DirectionKalmanPoseProvider;
import tasks.ObstacleAvoidanceTask;
import tasks.PizzaDropOffTask;
import tasks.PizzaPickupTask;

/** Main entrypoint for the final product */
public class FullRoutine
{
    public static void main(String[] args) throws Exception
    {
        // ---- INIT
        // Create the robot with default parameters
        Robot robot = new Robot();

        // Pass in the pilot as a MoveProvider, and the Gyro
        DirectionKalmanPoseProvider gyro_pose = new DirectionKalmanPoseProvider(robot.pilot, robot.gyro, true);
        robot.setPoseProvider(gyro_pose);
        
        gyro_pose.setPose(new Pose(0,0,-90));

        // ----- USER MENU
        // ------------------------------------------------------------
        boolean success = true;

        // ----- ROAD SELECTION
        Road[] roads = Road.values();
        String[] road_names = new String[roads.length];

        for (int i = 0; i < roads.length; i++)
        {
            road_names[i] = roads[i].name();
        }
        TextMenu road_select = new TextMenu(road_names, 1, "ROAD SELECTION");
        int selected_road_index = road_select.select();
        success = success && (selected_road_index >= 0);

        Road road = null;
        if (success)
        {
            road = roads[selected_road_index];
        }
        LCD.clear();

        // ----- HOUSE SIDE SELECTION        
        boolean house_left_side = true;
        if (success)
        {
            LCD.drawString("Which side? (L/R)", 0, 0);
            while (success)
            {
                if (Button.ESCAPE.isDown())
                {
                    success = false;
                }
                else if (Button.LEFT.isDown())
                {
                    house_left_side = true;
                    break;
                }
                else if (Button.RIGHT.isDown())
                {
                    house_left_side = false;
                    break;
                }
                Delay.msDelay(50);
            }
        }
        LCD.clear();

        // ----- HOUSE NUMBER SELECTION        
        int house_address = -1;
        if (success)
        {
            TextMenu address_select = new TextMenu(new String[] { "1", "2", "3" }, 1, "ADDRESS SELECTION");
            int selected_address_index = address_select.select();
            success = success && (selected_address_index >= 0);
            house_address = selected_address_index + 1;
        }
        LCD.clear();

        // ----- DEFINE OBJECTIVES
        // ------------------------------------------------------------

        
            
        // Get the pizza pedestal we'll be moving to
        PizzaPedestal target = PizzaPedestal.LEFT;

        House house = null;
        if (success)
        {
            house = new House(road, house_left_side, house_address);
        }

        // ----- PERFORM THE TASKS
        // ------------------------------------------------------------

        if (success)
        {   
            Delay.msDelay(250);
            System.out.println("Press ENTER to start");
            Button.ENTER.waitForPress();
            
            System.out.println(road.start.getX() + " " + road.start.getY());
            System.out.println(house.address + " " + house.left);
        }

        success = success && PizzaPickupTask.run_task(robot, target);
        success = success && ObstacleAvoidanceTask.navigate_to_pose_task(robot, road.start);
        success = success && PizzaDropOffTask.run_task(robot, house);
        success = success && ObstacleAvoidanceTask.navigate_to_pose_task(robot, new Pose(0, 0, 0));

        if (success)
        {
            System.out.println("Press ENTER to end");
            Button.ENTER.waitForPress();
        }
    }
}
