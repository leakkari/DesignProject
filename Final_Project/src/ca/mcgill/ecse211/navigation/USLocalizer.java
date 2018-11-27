package ca.mcgill.ecse211.navigation;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.main.*;

/** This class serves to drive the robot to the 0 degrees axis
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */
public class USLocalizer implements Runnable {
	
	private enum State {RESET,FIRSTWALL,SECONDWALL,CORRECTION,DONE}
	private State state;
	
	// vehicle constants
	public int ROTATE_SPEED;
	private double deltaTheta;

	//Odometer
	private Odometer odometer;

	//Robot
	private RobotController robot;

	//Sensor
	private UltrasonicSensorController usSensor;
	
	
	private boolean isRunning;
	private int filterControl;
	private static final int FILTER_OUT = 30;
	private int prevDistance;
	
	//Constant
	private int OPEN_SPACE = 50;
	private int WALL = 25;
	private int ERROR = 5;
	private double angleA, angleB, turningAngle;

	/**
	 * Constructor to initialize variables
	 * 
	 * @param Odometer
	 * @param EV3LargeRegulatedMotor
	 * @param EV3LargeRegulatedMotor
	 * @param usSensor
	 */
	public USLocalizer(Odometer odometer, RobotController robot,UltrasonicSensorController usSensor) {
		this.odometer = odometer;
		this.robot = robot;
		this.usSensor =usSensor;
		this.ROTATE_SPEED = 150;
		isRunning = true;
		prevDistance = Integer.MAX_VALUE;
		state = State.RESET;
		robot.setSpeeds(175,175);
	}

	public void run() {
	
		while(isRunning) {
			
			int distance = usSensor.fetch();
			
			if (distance >= 255 && filterControl < FILTER_OUT && prevDistance < distance) {
				filterControl++;
				distance = prevDistance;
			} else if (distance >= 255) {
				// do nothing
			} else {
				filterControl = 0;
			}
			
			switch(state) {
			case RESET:{
				if (distance >= 45) {
					robot.stopMoving();
					//odometer.setTheta(0);
					state = State.FIRSTWALL;
				} 
				//If it is not too far from the wall, start moving counterclockwise until it is
				else {
					robot.rotate(true);
				}
				break;
			}
			case FIRSTWALL:{
				if ((distance <= WALL + ERROR) && (prevDistance > WALL + ERROR))  {
					robot.stopMoving();
					angleA = odometer.getXYT()[2];
					Sound.beep();
					//robot.turnBy(50, true); //turn out of wall
					robot.rotate(true);
					try {
						Thread.sleep(3000);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					state = State.SECONDWALL;
				} 
				else {
					robot.rotate(false);
				}
				break;	
			}
			case SECONDWALL:{
				if ((distance <= WALL + ERROR) && (prevDistance > WALL + ERROR))  {
					robot.stopMoving();
					angleB = odometer.getXYT()[2];
					Sound.beep();
					state = State.CORRECTION;
				} 
				//If no falling edge, keep moving counterclockwise
				else {
					robot.rotate(true);
				}
				break;
			}
			case CORRECTION:{
				
				if (angleA < angleB) {
					deltaTheta = 45 - (angleA + angleB) / 2;

				} else if (angleA > angleB) {
					deltaTheta = 225 - (angleA + angleB) / 2;
				}
				turningAngle = deltaTheta + odometer.getXYT()[2];
				robot.turnBy(turningAngle,false);
				odometer.setXYT(0.0, 0.0, 0.0);
				state = State.DONE;
				break;
			}
			case DONE:{ 
				isRunning = false;
				break;
			}
			default: break;
			
			}
			this.prevDistance = distance;
		}
	}
	/**
	 * A method to localize position using the falling edge
	 * 
	 */
	public void usLocalize() {

		double angleA, angleB, turningAngle;
		boolean firstWallDetected, secondWallDetected;
		
		robot.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);
		
//		if(usSensor.fetch() < WALL) {
//			while(usSensor.fetch() < 50) {
//				robot.rotate(true);
//			}
//		}
		// Rotate to open space
		while (usSensor.fetch() < OPEN_SPACE) {
			robot.rotate(false);
		}
		
		// Rotate to the first wall
		while (usSensor.fetch() > WALL) {
			robot.rotate(false);
		}

		robot.stopMoving();
		Sound.beep();
		angleA = odometer.getXYT()[2];
		
		//robot.turnBy(50, true);
		
		robot.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);
		
		// rotate out of the wall range
		while (usSensor.fetch() < OPEN_SPACE ) {
			robot.rotate(true);
		}
		
		// rotate to the second wall
		while (usSensor.fetch() > WALL) {
			robot.rotate(true);
		}
		
		robot.stopMoving();
		Sound.beep();
		angleB = odometer.getXYT()[2];

		robot.stopMoving();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		robot.turnBy(turningAngle,false);

		// set odometer to theta = 0
		odometer.setXYT(0.0, 0.0, 0.0);

	}



}
