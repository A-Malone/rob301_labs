package common;

import lejos.robotics.geometry.Point;
import lejos.robotics.navigation.Pose;

/** Contains all of the measured information about the board itself */
public class BoardUtils
{
    /** The roads for drop-off */
    public enum Road
    {
        RED_ROAD(new Pose(-50, 200, -25), 5), BLUE_ROAD(new Pose(0, 200, 0), 2), GREEN_ROAD(new Pose(50, 200, 25), 3);

        /** The length of a road */
        public static final int length = 150;
        
        /** The length of offset before houses start */
        public static final int start_offset = 20;
        
        public Pose start;
        public int color;

        private Road(Pose start, int color)
        {
            this.start = start;
            this.color = color;
        }
    }
    
    /** The pizza pedestal locations */
    public enum PizzaPedestal
    {
        LEFT(new Point(-100, 10)), RIGHT(new Point(100, 10));

        public Point location;

        private PizzaPedestal(Point location)
        {
            this.location = location;
        }
    }
    
    /** Represents a house on the board */
    public static class House
    {
        public final static int DIST_TO_ROAD = 10;
        
        public Road road;
        public boolean left;
        public int address;
        
        public House(Road road, boolean left, int address)
        {
            this.road = road;
            this.left = left;
            this.address = address;
        }
    }
}
