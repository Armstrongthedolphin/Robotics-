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
	final static double AXLE_LENGTH = .124;
	static double displacement = 0.0;
	static double mOrientation = 0.0;
	
	
	public static void main(String[] args) {
		
		EV3MediumRegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A);
		EV3MediumRegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);
		mA.synchronizeWith(new EV3MediumRegulatedMotor[] {mB});
		
		EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S2);
		EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		SensorMode touch = touchSensor.getTouchMode();
		SensorMode sonic = (SensorMode) ultraSensor.getDistanceMode();
		float[] touchSample = new float[touch.sampleSize()];
		float[] sonicSample = new float[sonic.sampleSize()];
		mA.startSynchronization();
		mB.forward();
		mA.forward();
		mA.endSynchronization();
		
		//stop when you hit a wall
		while(touchSample[0] == 0){
			touch.fetchSample(touchSample, 0);
		}
		mA.startSynchronization();
		mB.stop();
		mA.stop();
		mA.endSynchronization();
		
		//back up 15cm
		float distanceToGo = .15f;
		double numRotations = ( distanceToGo / (RADIUS * 2 * PI));
		int backAngle = (int) (-360.0 * numRotations);
		mA.setSpeed(180);
		mB.setSpeed(180);
		mA.startSynchronization();
		mA.rotate(backAngle, false);
		mB.rotate(backAngle, false);
		mA.endSynchronization();
	
		Sound.beep();
		
		//turn right 
		rotateAngle((float) (-PI/4.0), mA, mB);
		
		
		
		
		
		
		
		//wall following (Bang Bang)
		float setDistance = .05f;
		float initspeed = 180f;
		float error = 0f;
		float newerror = 0f;
		float errordiff = 0f;
		float setbuffer = 0.02f;
		float terminatediff = 0f;
		float distanceTraveled = 0f;
		float adjustAngle = 0f;
		float infinity = .40f;
		boolean forever = true;
		mA.startSynchronization();
		mB.forward();//left wheel
		mA.forward();//right wheel
		mA.endSynchronization();
		sonic.fetchSample(sonicSample, 0);
		error = sonicSample[0] - setDistance;
		while(forever){
			newerror = sonicSample[0] - setDistance;
			errordiff = newerror - error; // if positive, error increase
			//according to the error difference, adjust the angle with one wheel set to speed 0
			if ( abs(errordiff) > terminatediff || newerror > infinity ){//end of the wall, break loop
				
				break;
			} else if(abs(errordiff) > setbuffer){
				//adjust angle
				//calculate distance traveled
				adjustAngle = calculateAngle(error, newerror,distanceTraveled );
				rotateAngle(adjustAngle, mA, mB);
				
			}

			error = newerror;
			mA.setSpeed(initspeed);
			mB.setSpeed(initspeed);
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			sonic.fetchSample(sonicSample, 0);
			
		}
		
		//turn to face forward
		rotateAngle((float) -mOrientation, mA, mB);
		
		
		//move 0.75m 
		distanceToGo = 0.75f;
		goforward(distanceToGo, mA, mB);
		
	}
	
	
	private static void goforward(float distanceToGo, EV3MediumRegulatedMotor left, EV3MediumRegulatedMotor right) {
		float distance = distanceToGo;
		double numRotations = ( distance / (RADIUS * 2 * PI));
		int angle = (int) (360.0 * numRotations);
		left.setSpeed(180);
		right.setSpeed(180);
		left.startSynchronization();
		left.rotate(angle, false);
		right.rotate(angle, false);
		left.endSynchronization();
		
	}
	
	private static void goforward(float distanceToGo, int leftSpeed, int rightSpeed, EV3MediumRegulatedMotor left, EV3MediumRegulatedMotor right) {
		float distance = distanceToGo;
		double numRotations = ( distance / (RADIUS * 2 * PI));
		int angle = (int) (360.0 * numRotations);
		left.setSpeed(leftSpeed);
		right.setSpeed(rightSpeed);
		left.startSynchronization();
		left.rotate(angle, false);
		right.rotate(angle, false);
		left.endSynchronization();
		
	}


	private static void rotateAngle(float angle, EV3MediumRegulatedMotor left, EV3MediumRegulatedMotor right) {
		assert(right.getRotationSpeed() == 0 || left.getRotationSpeed() == 0);
		long initTime = System.nanoTime();
		long timeToRotate;
		float desiredAngularVelocity;
		double wheelRotationSpeed;
		
		if (angle < 0) {	//turning left
			
			wheelRotationSpeed = right.getRotationSpeed();
			
			if (wheelRotationSpeed == 0) { //sammy is stationary
				right.setSpeed(180);
				wheelRotationSpeed = 180;
				desiredAngularVelocity = (float) (( wheelRotationSpeed * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( -angle / desiredAngularVelocity) * 1000000000; 
				while(System.nanoTime() < timeToRotate) {
				}
				right.stop();
				
			} else { //sammie was originally moving
				desiredAngularVelocity = (float) (( wheelRotationSpeed * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( -angle / desiredAngularVelocity) * 1000000000; 
				left.stop();
				while(System.nanoTime() < timeToRotate) {
				}
				left.forward();
			}
			
			
		} else {	//turning right
			wheelRotationSpeed = left.getRotationSpeed();
			
			if (wheelRotationSpeed == 0) { //sammy is stationary
				left.setSpeed(180);
				wheelRotationSpeed = 180;
				desiredAngularVelocity = (float) (( wheelRotationSpeed * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) ( -angle / desiredAngularVelocity) * 1000000000; 
				while(System.nanoTime() < timeToRotate) {
				}
				left.stop();
				
			} else {
				desiredAngularVelocity = (float) (( wheelRotationSpeed * RADIUS) / AXLE_LENGTH) ;
				timeToRotate = (long) (angle / desiredAngularVelocity) * 1000000000; 
				right.stop();
				while(System.nanoTime() < timeToRotate) {
				}
				right.forward();
			}
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
