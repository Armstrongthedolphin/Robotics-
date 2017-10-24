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
	static double mOrientation = PI / 2.0; //angle from right, measured counterclockwise
	
	
	public static void main(String[] args) {
		
		EV3MediumRegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A); //left motor
		EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B); //right motor
		EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S2);
		EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);

		mA.synchronizeWith(new EV3MediumRegulatedMotor[] {mB});

		SensorMode touch = touchSensor.getTouchMode();
		SensorMode sonic = (SensorMode) ultraSensor.getDistanceMode();
		
		float[] sonarSample = new float[sonic.sampleSize()];
		float[] touchSample = new float[touch.sampleSize()];
		
		
		//Go until bump into wall
		System.out.println("Moving towards the wall!");
		while(touchSample[0] == 0){
			touch.fetchSample(touchSample, 0);
		}
		mA.startSynchronization();
		mB.stop();
		mA.stop();
		mA.endSynchronization();
		
		float distanceToGo = .15f;
		double numRotations = ( distanceToGo / (RADIUS * 2 * PI));
		int backAngle = (int) (-360.0 * numRotations);
		mA.startSynchronization();
		mA.rotate(backAngle, false);
		mB.rotate(backAngle, false);
		mA.endSynchronization();	
		Sound.beep();
		
		
		
		
		//turn right 
		rotateAngle((float) (-PI/4.0), mA, mB);
		Sound.beep();
		
		
		
		//wall following (Bang Bang)
		float leftbound = .1f;
		float rightbound = .2f; 
		float midline = .15f;
		float initspeed = 180f;
		boolean forever = true;
		int state = STRAIGHT; 
		float previousSonarMeasure = 0;

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
				
				
			} else if(sonarSample[0] > rightbound){//larger than 0.20m, turn left
				mB.setSpeed(initspeed+10);
				state = LEFT;
			} else{//between 0.1m and 0.2m, go straight
				if (state != STRAIGHT){
					mA.setSpeed(initspeed);
					mB.setSpeed(initspeed);
					state = STRAIGHT;
				}
			}
			previousSonarMeasure = sonarSample[0];
			sonic.fetchSample(sonarSample, 0);
		}
		
		//turn to face forward
		
		
		//move 0.75m 
		distanceToGo = 0.75f;
		numRotations = ( distanceToGo / (RADIUS * 2 * PI));
		int forwardAngle = (int) (360.0 * numRotations);
		mA.setSpeed(180);
		mB.setSpeed(180);
		mA.startSynchronization();
		mA.rotate(forwardAngle, false);
		mB.rotate(forwardAngle, false);
		mA.endSynchronization();
		
	}
	
	//
	private static float getFrontDistance(float sonarDistance) {
		return (float) Math.cos(sonarDistance) * sonarDistance;
	}
	
	private static void rotateAngle(float angle, EV3MediumRegulatedMotor left, EV3MediumRegulatedMotor right) {
		long initTime = System.nanoTime();
		long timeToRotate;
		float desiredAngularVelocity;
		double wheelRotationSpeed;
		if (angle < 0) {
			wheelRotationSpeed = right.getRotationSpeed();
			desiredAngularVelocity = (float) (( wheelRotationSpeed * RADIUS) / AXLE_LENGTH) ;
			timeToRotate = (long) ( -angle / desiredAngularVelocity) * 1000000000; 
			left.stop();
			while(System.nanoTime() < timeToRotate) {
			}
			left.forward();
			
		} else {
			wheelRotationSpeed = left.getRotationSpeed();
			desiredAngularVelocity = (float) (( wheelRotationSpeed * RADIUS) / AXLE_LENGTH) ;
			timeToRotate = (long) (angle / desiredAngularVelocity) * 1000000000; 
			right.stop();
			while(System.nanoTime() < timeToRotate) {
			}
			right.forward();
		}
		mOrientation += angle;
	}
	
	//takes in two sonar readings and the distance traveled between those two readings
	//outputs the angle of attack to object detected by sonar in radians
	//positive values mean going towards object
	private static float calculateAngle(float sonar0, float sonar1, 
											float distanceTravelled) {
		float angle = (float) Math.acos((sonar1 - sonar0)/distanceTravelled);
		return angle;
	}
	
	

}
