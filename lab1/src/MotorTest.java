import lejos.hardware.motor.*;
public class MotorTest {
	public static void main(String[] args) throws Exception {
		Motor.A.setSpeed(500);
		Motor.A.rotate(180);
	}
}
