import lejos.hardware.Button;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class ColorRedTest {
	public static void main(String[] args) {
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S4);
		LCD.clear();
		while (!Button.ENTER.isDown()) {
			int sampleSize = color.sampleSize();
			float[] redsample = new float[sampleSize];
			color.getRedMode().fetchSample(redsample, 0);
			LCD.clear();
			System.out.println(redsample[0]);
		}
		color.close();
	}
}