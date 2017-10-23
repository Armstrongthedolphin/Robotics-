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
import java.lang.Math;

	
public class main {
	
	final static double RADIUS= .0275; //RADIUS of the tires in meters
	final static double PI = 3.141592653589793;
//	final static float SONAR_OFFSET = .024f; //how far the sonar is from front of robut
	final static int STRAIGHT = 0;
	final static int LEFT = 1;
	final static int RIGHT = 2;
	final static double AXLE_LENGTH = .17; //distance between two wheels, need to change
	final static double SONAR_ANGLE = PI / 4.0;
//	static double displacement = 0.0;
	static double orientation = PI / 2.0; //angle from right, measured counterclockwise
	
	
	public static void main(String[] args) {
		
		EV3MediumRegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A);
		EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);
		//EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S2);
		EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);

		mA.synchronizeWith(new EV3MediumRegulatedMotor[] {mB});

		//SensorMode touch = touchSensor.getTouchMode();
		SensorMode sonic = (SensorMode) ultraSensor.getDistanceMode();
		
		//Moving towards the wall until  30cm from the wall
		System.out.println("Moving towards the wall!");
		float distanceToWall = .15f;
		float[] sonarSample = new float[sonic.sampleSize()];
		sonic.fetchSample(sonarSample, 0);
		
		//try while?
	
		
//		System.out.println("Initial Distance to wall: " + (sonarSample[0] + SONAR_OFFSET));
		if(getFrontDistance(sonarSample[0]) > distanceToWall) {
			mA.startSynchronization();
			mA.forward();
			mB.forward();
			mA.endSynchronization();
		}
		while(sonarSample[0] > (distanceToWall)){
			sonic.fetchSample(sonarSample, 0);
		}
		mA.startSynchronization();
		mB.stop();
		mA.stop();
		mA.endSynchronization();
		
		Sound.beep();
		System.out.println("\n\n\n\n\n\nWaiting to proceed to next step");
		Button.ENTER.waitForPressAndRelease();
		
		
		
		//turn right 
		mA.setSpeed(180);
		mB.setSpeed(180);
		mA.startSynchronization();
		mB.forward();
		mA.forward();
		mA.endSynchronization();
		
		//wall following (Bang Bang)
		float leftbound = .1f;
		float rightbound = .2f; 
		float midline = .15f;
		float initspeed = 180f;
		int state = STRAIGHT; 
		mA.startSynchronization();
		mB.forward();//left wheel
		mA.forward();//right wheel
		mA.endSynchronization();
		sonic.fetchSample(sonarSample, 0);
		while(sonarSample[0] <  .30){
			//less than 0.10cm, turn right
			if(sonarSample[0] < leftbound){
				mA.setSpeed(initspeed+10);
				state = RIGHT;
				
				
			}else if(sonarSample[0] > rightbound){//larger than 0.20cm, turn left
				mB.setSpeed(initspeed+10);
				state = LEFT;
			}else{//between 0.1cm and 0.2cm, go straight
				if (state != STRAIGHT){
					mA.setSpeed(initspeed);
					mB.setSpeed(initspeed);
					state = STRAIGHT;
				}
				
			
			}
		}
		
		//turn to face forward
		
		//move 0.75m 
		float distanceToGo = 0.75f;
		double numRotations = ( distanceToGo / (RADIUS * 2 * PI));
		int angle = (int) (360.0 * numRotations);
		mA.setSpeed(180);
		mB.setSpeed(180);
		mA.startSynchronization();
		mA.rotate(angle, false);
		mB.rotate(angle, false);
		mA.endSynchronization();
		
	}
	
	//
	private static float getFrontDistance(float sonarDistance) {
		return (float) Math.cos(sonarDistance) * sonarDistance;
	}
	
	

}
