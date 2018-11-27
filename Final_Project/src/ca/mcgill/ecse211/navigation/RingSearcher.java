package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.ColorSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.enumeration.SearchState;
import ca.mcgill.ecse211.main.Main;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This class implements the ring searcher. When this class is called, the state is set to IN_PROGRESS.While this state is set, the robot 
 * can do 3 different actions: DETECTING, APPROACHING, GRABBING: so once the robot detects a ring, it approaches it
 * to be able to detect its color and then grabs it.
 * @author Jeffrey Leung
 * @author leaakkari
 *
 */
public class RingSearcher implements Runnable {

	private enum State {APPROACHING,DETECTING,GRABING};
	private State state;
	//Robot
	private RobotController robot;

	//Sensors
	private ColorSensorController colorSensor;
	private UltrasonicSensorController usSensor;

	public static final double SENSOR_LENGTH = 3.3;

	//Odometer
	private Odometer odometer;

	//Search state
	public SearchState searchState;

	//Constants
	private long START_TIME = Main.START_TIME;

	//Odometry correction
	private OdometryCorrection odoCorr;

	//Tree Sides
	private int targetRing = 4;

	//	//Side motors
	//	private static final EV3LargeRegulatedMotor leftSideMotor = 
	//			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	//
	//	private static final EV3LargeRegulatedMotor rightSideMotor = 
	//			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	//usdistance
	//	int usDistance = usSensor.fetch();

	/**
	 *  Constructor for ring searcher
	 * @param colorSensor
	 * @param gyroSensor
	 * @param usSensor
	 * @param leftMotor
	 * @param rightMotor
	 */
	public RingSearcher(Odometer odometer,ColorSensorController colorSensor, UltrasonicSensorController usSensor, RobotController robot) {
		this.colorSensor = colorSensor;
		this.usSensor = usSensor;
		this.robot = robot;
		this.odometer = odometer;
		searchState = SearchState.IN_PROGRESS;
		state = State.APPROACHING;

	}
	/**
	 * Thread that controls the ring search
	 */
	@Override 
	public void run(){
		while(searchState == SearchState.IN_PROGRESS) {

			long timeElapsed = System.currentTimeMillis() - START_TIME;
			//Time out at 4 min
			if(timeElapsed >= 240000) {
				searchState = SearchState.TIME_OUT;
			}

			float[] rgb = colorSensor.fetch();

			switch(state) {
			case APPROACHING:{
				odoCorr.correct(odometer.getXYT()[2]);
				robot.setSpeeds(100, 100);
				robot.travelDist(8); //Distance to reach the ring

				state = State.DETECTING;
				break;
			}
			case DETECTING:{
				if(colorSensor.findMatch(rgb) != 4) {
					colorSensor.beep();
					state = State.GRABING;

				}
				break;
			}
			case GRABING:{
				robot.setSpeeds(80, 80);
				robot.travelDist(8); //Distane to grab the ring
				robot.travelDist(-13); //Distance to back off
				odoCorr.correct(odometer.getXYT()[2]);
				robot.travelDist(SENSOR_LENGTH);
				searchState = SearchState.RING_FOUND;
				break;
			}
			}
		}
	}


	/**
	 * Sets the OdometryCorrection object to be used by the robot controller.
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorr = odoCorrection;
	}
}

/*
 //	public void detectRing() {
//
//		int color = colorSensor.findMatch(colorSensor.fetch()) ;
//
//		while(color == 4) {
//			color = colorSensor.findMatch(colorSensor.fetch()) ;	
//		}
//		colorSensor.beep();
//		
//		searchState = SearchState.RING_FOUND;
//		
//	}
//	
	/**
 * This method serves to move forward and backward in order to detect the ring
 */
//	public void grabRing() {
//				
//		odoCorr.correct(odometer.getXYT()[2]);
//		
//		robot.setSpeeds(100, 100);
//
//		robot.travelDist(15);
//		
//		while(searchState == SearchState.IN_PROGRESS) {
////			long timeElapsed = System.currentTimeMillis() - START_TIME;
////			//Time out at 4 min
////			if(timeElapsed >= 240000) {
////				searchState = SearchState.TIME_OUT;
////			}
////			
////			if(colorSensor.findMatch(colorSensor.fetch()) != 4) {
////				colorSensor.beep();
////				searchState = SearchState.RING_FOUND;
////			}
//		}
//		robot.travelDist(10);
//		
//		robot.travelDist(-30);
//		
//		odoCorr.correct(odometer.getXYT()[2]);
//		
//	}*/




