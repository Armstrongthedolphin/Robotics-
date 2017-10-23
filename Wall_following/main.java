/****
 * Author: Yan Ren, Victor Murta
 */
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;

//import lejos.robotics.RegulatedMotor;

	
public class Main {
	
	final static double RADIUS= .0275; //RADIUS of the tires in meters
	final static double PI = 3.141592653589793;
	final static float SONAR_OFFSET = .024f; //how far the sonar is from front of robut
	static double displacement = 0.0;
	
	
	public static void main(String[] args) {
		
		EV3MediumRegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A);
		EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);
		EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S2);
		EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);

		mA.synchronizeWith(new EV3MediumRegulatedMotor[] {mB});

		SensorMode touch = touchSensor.getTouchMode();
		SensorMode sonic = (SensorMode) ultraSensor.getDistanceMode();
		
		//Odometer test
		System.out.println("Moving forward into the great big world, Odometer style!");
		float distanceToGo = 1.50f;
		double numRotations = ( distanceToGo / (RADIUS * 2 * PI));
		int angle = (int) (360.0 * numRotations);
		mA.setSpeed(180);
		mB.setSpeed(180);
		mA.startSynchronization();
		mA.rotate(angle, false);
		mB.rotate(angle, false);
		mA.endSynchronization();
		
		displacement = distanceToGo;
		Sound.beep();
		System.out.println("\n\n\n\n\n\nWaiting to proceed to next step");
		Button.ENTER.waitForPressAndRelease();
		
		
		
		//SONAR TEST
		System.out.println("\n\n\n\n\n\n\nSonar test!");
		float distanceToWall = .45f + SONAR_OFFSET;
		float[] sonarSample = new float[sonic.sampleSize()];
		sonic.fetchSample(sonarSample, 0);
		
//		System.out.println("Initial Distance to wall: " + (sonarSample[0] + SONAR_OFFSET));
		if(sonarSample[0] > distanceToWall) {
			mA.startSynchronization();
			mA.forward();
			mB.forward();
			mA.endSynchronization();
		}
		//.02 is the fudge factor, "it just works"
		while(sonarSample[0] > (distanceToWall + .024)){
			sonic.fetchSample(sonarSample, 0);
		}
		mA.startSynchronization();
		mB.stop();
		mA.stop();
		mA.endSynchronization();
		
		Sound.beep();
		System.out.println("\n\n\n\n\n\nWaiting to proceed to next step");
		Button.ENTER.waitForPressAndRelease();
		
		
		
		//bump test
		//move forward until hits the wall
		mA.setSpeed(180);
		mB.setSpeed(180);
		mA.startSynchronization();
		mB.forward();
		mA.forward();
		mA.endSynchronization();
		
		//stop when you hit a wall
		float[] touchSample = new float[touch.sampleSize()];
		while(touchSample[0] == 0){
			touch.fetchSample(touchSample, 0);
		}
		mA.startSynchronization();
		
		mB.stop();
		mA.stop();
		mA.endSynchronization();

		//move back till 45cm away from wall
		mA.startSynchronization();
		mA.rotate(-720, false);
		mB.rotate(-720, false);
		mA.endSynchronization();
		mA.startSynchronization();
		mB.backward();
		mA.backward();
		mA.endSynchronization();
		sonic.fetchSample(sonarSample, 0);
		while(sonarSample[0] < distanceToWall){
			sonic.fetchSample(sonarSample, 0);
		}
		mA.startSynchronization();
		mB.stop();
		mA.stop();
		mA.endSynchronization();
		
		mA.close();
		mB.close();
	}
	
	

}
