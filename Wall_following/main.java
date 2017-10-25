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
	final static float SONAR_OFFSET = .022f; //how far the sonar is from front of robut
	final static double AXLE_LENGTH = .122;
	// static double mDisplacement = 0.0;
	static double mOrientation = 0.0;
	static EV3MediumRegulatedMotor left;
	static EV3MediumRegulatedMotor right;
	
	public static void main(String[] args) {
		
		left= new EV3MediumRegulatedMotor(MotorPort.A);
		right = new EV3MediumRegulatedMotor(MotorPort.D);
		left.synchronizeWith(new EV3MediumRegulatedMotor[] {right});
		EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S3);
		EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		SensorMode touch = touchSensor.getTouchMode();
		SensorMode sonic = (SensorMode) ultraSensor.getDistanceMode();
		float[] touchSample = new float[touch.sampleSize()];
		float[] sonicSample = new float[sonic.sampleSize()];

		System.out.println("Moving forward");
		left.startSynchronization();
		right.forward();
		left.forward();
		left.endSynchronization();
		
		//stop when you hit a wall
		while(touchSample[0] == 0){
			touch.fetchSample(touchSample, 0);
		}
		left.startSynchronization();
		right.stop();
		left.stop();
		left.endSynchronization();

		//back up 15cm
		Sound.beep();
		System.out.println("Moving Backwards");
		move(-.15f);
		Button.ENTER.waitForPressAndRelease();
//		double numRotations = ( .15 / (RADIUS * 2 * PI));
//		int angle = (int) (-360.0 * numRotations);
//		left.startSynchronization();
//		left.rotate(angle, false);
//		right.rotate(angle, false);
//		left.endSynchronization();
//		Button.ENTER.waitForPressAndRelease();
		
		//turn right 
		System.out.println("turn right");
		rotateAngle((float) (PI/2.0));
		Sound.beep();
		
		

		
		
		
		
		
		//wall following (Bang Bang)
		float setDistance = .10f;
		float initspeed = 180f;
		float error = 0f;
		float newerror = 0f;
		float errordiff = 0f;
		float setbuffer = 0.05f;
		float terminatediff = 0.4f;
		float distanceTraveled = 0f;
		float adjustAngle = 0f;
		float infinity = .30f;
		long travelTime = 100000000; //in nanoseconds
		double orientationTolerance = PI/6.0;
		long timestamp;
		boolean forever = true;
		left.startSynchronization();
		right.forward();//left wheel
		left.forward();//right wheel
		left.endSynchronization();
		sonic.fetchSample(sonicSample, 0);
		error = sonicSample[0] - setDistance;
		
		touch.fetchSample(touchSample, 0);
		while(forever){
			
			newerror = sonicSample[0] - setDistance;			
			errordiff = newerror - error; // if positive, error increase
			System.out.print("E " + newerror + " " + errordiff+ " ");


			
			if(mOrientation < orientationTolerance) {
				break;
			}
			//according to the error difference, adjust the angle with one wheel set to speed 0
			if ( abs(errordiff) > terminatediff || newerror > infinity ){//end of the wall, break loop
				
				break;
			}else {
				if(newerror< -1*setbuffer || newerror> setbuffer){//if drifting left from the offset turn right
					adjustAngle = calculateAngle(error, newerror,distanceTraveled );
					rotateAngle(adjustAngle);	
				}

			}
//			} else if(abs(errordiff) > setbuffer){
//				//adjust angle
//				//calculate distance traveled
//				distanceTraveled = (float) ( initspeed * travelTime*0.001 * (PI / 360) * RADIUS); //m
//				System.out.print("D " + distanceTraveled + " " );//15.118915
//				adjustAngle = calculateAngle(error, newerror,distanceTraveled );
//				rotateAngle(adjustAngle, left, right);
//				
//			}

			error = newerror;
			left.setSpeed(initspeed);
			right.setSpeed(initspeed);
			
			timestamp = System.nanoTime() + travelTime;
			while(System.nanoTime() < timestamp) {
				touch.fetchSample(touchSample, 0);
				if(touchSample[0] != 0){
					System.out.println("Collision detected");
					move( -.15f);
					rotateAngle( (float) (PI/6.0));
					move( .10f);
					
				}
			}

			sonic.fetchSample(sonicSample, 0);
			touch.fetchSample(touchSample, 0);
		}
		
		//test
		left.startSynchronization();
		right.stop();
		left.stop();
		left.endSynchronization();
		Sound.beep();
	
		
		
		
		
		
		
		System.out.println("Returning to original heading");
		//turn to face forward
		rotateAngle((float) -mOrientation);
		Sound.beepSequenceUp();
		
		//move 0.75m 
		move(.75f);
		Button.ENTER.waitForPressAndRelease();
	}
	
	
	private static void move(float distanceToGo) {
		move(distanceToGo, 180, 180);
	}
	
	private static void move(float distanceToGo, int leftSpeed, int rightSpeed) {
		left.setSpeed(leftSpeed);
		right.setSpeed(rightSpeed);
		double numRotations = ( distanceToGo / (RADIUS * 2.0 * PI));
		int angle = (int) (360.0 * numRotations);
		System.out.println("moving wheels " + angle + " degrees ");
		left.startSynchronization();
		left.rotate(angle, true);
		right.rotate(angle, false);
		left.endSynchronization();
		while(left.isMoving()) {
		}
	}


	private static void rotateAngle(float angle) {
		assert(right.getRotationSpeed() == 0 || left.getRotationSpeed() == 0);
		long initTime = System.nanoTime();
		long timeToRotate;
		float desiredAngularVelocity;
		int wheelRotationSpeedDegrees,RightwheelRotationSpeedDegrees,LeftwheelRotationSpeedDegrees;
		float wheelRotationSpeedRadians;
		
		System.out.print((int)(angle * 180.0f/PI) + "degrees" );
		RightwheelRotationSpeedDegrees = right.getRotationSpeed();
		LeftwheelRotationSpeedDegrees = right.getRotationSpeed();
		
		if (angle < 0) {	//turning left
			
			wheelRotationSpeedDegrees = right.getRotationSpeed();
			
			if (!right.isMoving()) { //sammy is stationary
				System.out.println("stationary left turn");
				wheelRotationSpeedDegrees = 180;
				right.setSpeed(wheelRotationSpeedDegrees);
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( -angle / desiredAngularVelocity * 1000000000.0) + System.nanoTime(); 
				right.forward();
				while(System.nanoTime() < timeToRotate) {
					right.forward();
				}
				right.stop();
				
			} else { //sammie was originally moving
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( -angle / desiredAngularVelocity * 1000000000.0) + System.nanoTime(); 
				left.stop();
				while(System.nanoTime() < timeToRotate) {
				}
				left.forward();
			}
			
			
		} else {	//turning right
			wheelRotationSpeedDegrees = left.getRotationSpeed();
			
			if (!left.isMoving()) { //sammy is stationary 
				System.out.println("stationary right turn");
				wheelRotationSpeedDegrees = 180;
				left.setSpeed(wheelRotationSpeedDegrees);
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( angle / desiredAngularVelocity * 1000000000.0)  + System.nanoTime(); 
				System.out.print("T " + timeToRotate + "  ");
				left.forward();
				while(System.nanoTime() < timeToRotate) {
					left.forward();
				}
				left.stop();
				
			} else {
				wheelRotationSpeedRadians = (float) (wheelRotationSpeedDegrees  * PI / 180.0);
				desiredAngularVelocity = (float) (( wheelRotationSpeedRadians * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) (angle / desiredAngularVelocity * 1000000000.0)  + System.nanoTime(); 
				right.stop();
				while(System.nanoTime() < timeToRotate) {
				}
				right.forward();
			}
		}
		left.setSpeed(wheelRotationSpeedDegrees);
		right.setSpeed(wheelRotationSpeedDegrees);
		mOrientation += angle;
		if (mOrientation > (2.0 * PI)) {
			mOrientation -= PI;
		}
	}
	
	//takes in two sonar readings and the distance traveled between those two readings
	//outputs the angle of attack to object detected by sonar in radians
	//positive values mean going towards object
	private static float calculateAngle(float sonar0, float sonar1, float distanceTravelled) {
		float angle;
		float maxAngle = (float) (10*(PI/180));
		float sonarscaler = 1f;
		if(sonar1>0){//turn left
			angle= maxAngle *-1 ;
		}else{//turn right
			angle= maxAngle;
		}
		//angle= (float) Math.atan((sonar0 - sonar1)/distanceTravelled);
		return angle;
	}
	

}
