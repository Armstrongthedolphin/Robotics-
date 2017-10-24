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
import static java.lang.Math.*;



	
public class main {
	
	final static double RADIUS= .0275; //RADIUS of the tires in meters
	final static double PI = 3.141592653589793;
	final static float SONAR_OFFSET = .024f; //how far the sonar is from front of robut
	final static int STRAIGHT = 0;
	final static int LEFT = 1;
	final static int RIGHT = 2;
	final static double AXLE_LENGTH = .17;
	static double displacement = 0.0;
	static double mOrientation = PI/ 4.0;//????why init to this
	
	
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
		float distanceToWall = .30f + SONAR_OFFSET;
		float[] sonarSample = new float[sonic.sampleSize()];
		sonic.fetchSample(sonarSample, 0);
		
		//try while?
	
		
//		System.out.println("Initial Distance to wall: " + (sonarSample[0] + SONAR_OFFSET));
		if(sonarSample[0] > distanceToWall) {
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
		rotateAngle((float) (-PI/4.0), mA, mB);
		
		
		
		//wall following (Bang Bang)
		float leftbound = .1f;
		float rightbound = .2f; 
		float setDistance = .05f;
		float initspeed = 180f;
		float error = 0f;
		float newerror = 0f;
		float errordiff = 0f;
		float initbufferspeed = 20f;
		float bufferspeed = 0f;
		float setbuffer = 0.02f;
		float terminatediff = 0f;
		float distanceTraveled = 0f;
		float adjustAngle = 0f;
		boolean forever = true;
		int state = STRAIGHT; 
		mA.startSynchronization();
		mB.forward();//left wheel
		mA.forward();//right wheel
		mA.endSynchronization();
		sonic.fetchSample(sonarSample, 0);
		error = sonarSample[0] - setDistance;
		while(forever){
			newerror = sonarSample[0] - setDistance;
			errordiff = newerror - error; // if positive, error increase
			//according to the error difference, adjust the angle with one wheel set to speed 0
			if ( abs(errordiff) > terminatediff ){//end of the wall, break loop
				
				break;
			} else if(abs(errordiff) > setbuffer){
				//adjust angle
				//calculate distance traveled
				adjustAngle = calculateAngle(error, newerror,distanceTraveled );
				rotateAngle(adjustAngle, mA, mB);
				
			}
//			if(errordiff > 0){
//				bufferspeed = error;
//				
//				mA.setSpeed(initspeed+10);
//				state = RIGHT;
//				
//				
//			}else if(sonarSample[0] > rightbound){//larger than 0.20cm, turn left
//				mB.setSpeed(initspeed+10);
//				state = LEFT;
//			}else{//between 0.1cm and 0.2cm, go straight
//				if (state != STRAIGHT){
//					mA.setSpeed(initspeed);
//					mB.setSpeed(initspeed);
//					state = STRAIGHT;
//				}
//			}
			error = newerror;
			mA.setSpeed(initspeed);
			mB.setSpeed(initspeed);
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			sonic.fetchSample(sonarSample, 0);
			
		}
		
		//turn to face forward
		    //
		
		
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
	private static float calculateAngle(float sonar0, float sonar1, float distanceTravelled) {
		float angle = (float) Math.acos((sonar1 - sonar0)/distanceTravelled);
		return angle;
	}
	

}
