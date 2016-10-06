import lejos.hardware.Button;
public class HelloROB301 {
	public static void main(String[] args) throws Exception {
		System.out.println("Hello ROB301!");
		Button.waitForAnyPress();
	}
}