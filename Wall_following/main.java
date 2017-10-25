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
	// static double mDisplacement = 0.0;
	static double mOrientation = 0.0;
	EV3MediumRegulatedMotor left;
	EV3MediumRegulatedMotor right;
	
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
		float distanceToGo = .15f;
		double numRotations = ( distanceToGo / (RADIUS * 2 * PI));
		int backAngle = (int) (-360.0 * numRotations);
		System.out.println(backAngle);
		left.setSpeed(180);
		right.setSpeed(180);
		left.startSynchronization();
		left.rotate(backAngle, false);
		right.rotate(backAngle, false);
		left.endSynchro	nization();
		Sound.beep();
		
		//turn right 
		rotateAngle((float) (PI/2.0));
		Sound.beep();

		
		
		
		
		
		
		
		//wall following (Bang Bang)
		float setDistance = .05f;
		float initspeed = 90f;
		float error = 0f;
		float newerror = 0f;
		float errordiff = 0f;
		float setbuffer = 0.02f;
		float terminatediff = 0.4f;
		float distanceTraveled = 0f;
		float adjustAngle = 0f;
		float infinity = .40f;
		float travelTime = 100f;
		boolean forever = true;
		left.startSynchronization();
		right.forward();//left wheel
		left.forward();//right wheel
		left.endSynchronization();
		sonic.fetchSample(sonicSample, 0);
		error = sonicSample[0] - setDistance;
		while(forever){

			newerror = sonicSample[0] - setDistance;
			System.out.print("E " + (int)newerror*100 + "  ");

						//last resort collision
			if(touchSample[0] != 0){
				move()
			}
			errordiff = newerror - error; // if positive, error increase
			//according to the error difference, adjust the angle with one wheel set to speed 0
			if ( abs(errordiff) > terminatediff || newerror > infinity ){//end of the wall, break loop
				
				break;
			} else if(abs(errordiff) > setbuffer){
				//adjust angle
				//calculate distance traveled
				distanceTraveled = (float) ( initspeed * travelTime * (PI / 360) * RADIUS);
				adjustAngle = calculateAngle(error, newerror,distanceTraveled );
				rotateAngle(adjustAngle);
				
			}

			error = newerror;
			left.setSpeed(initspeed);
			right.setSpeed(initspeed);
			try {
				Thread.sleep((long)travelTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
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
		Button.ENTER.waitForPressAndRelease();
		
		
		
		
		
		
		System.out.println("Returning to original heading");
		//turn to face forward
		rotateAngle((float) -mOrientation);
		Sound.beepSequenceUp();
		
		//move 0.75m 
		distanceToGo = 0.75f;
		move(distanceToGo);
		
	}
	
	
	private static void move(float distanceToGo) {
		double numRotations = ( distanceToGo / (RADIUS * 2 * PI));
		int angle;
		if (distanceToGo < 0){
			angle = (int) (-360.0 * numRotations);
		} else {
			angle = (int) (360.0 * numRotations);
		}
		left.setSpeed(180);
		right.setSpeed(180);
		left.startSynchronization();
		left.rotate(angle, false);
		right.rotate(angle, false);
		left.endSynchronization();
		
	}
	
	private static void move(float distanceToGo, int leftSpeed, int rightSpeed) {
		assert(distanceToGo > 0 || (leftSpeed < 0 && rightSpeed < 0));
		left.setSpeed(leftSpeed);
		right.setSpeed(rightSpeed);
		move(distanceToGo);
	}


	private static void rotateAngle(float angle) {
		assert(right.getRotationSpeed() == 0 || left.getRotationSpeed() == 0);
		long initTime = System.nanoTime();
		long timeToRotate;
		float desiredAngularVelocity;
		int wheelRotationSpeedDegrees;
		float wheelRotationSpeedRadians;
		
		System.out.println("Turning " + (int)(angle * 180.0f/PI) + "degrees" );
		if (angle < 0) {	//turning left
			
			wheelRotationSpeedDegrees = right.getRotationSpeed();
			
			if (wheelRotationSpeedDegrees <=  5) { //sammy is stationary
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
			
			if (wheelRotationSpeedDegrees <=  5) { //sammy is stationary 
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
