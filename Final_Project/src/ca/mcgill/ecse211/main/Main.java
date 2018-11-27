
package ca.mcgill.ecse211.main;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.tester.*;
import java.text.DecimalFormat;
import ca.mcgill.ecse211.controller.*;
import ca.mcgill.ecse211.enumeration.SearchState;

/** This is the main class of the project. It is at the top level of the layered hierarchy.
 *  It serves to link all the classes together and make a sequence of methods classes from 
 *  the middle layer (navigation package) in order to execute the tasks of the game. 
 *  
 *  First, all the constants about the robot are set (WHEEL_RAD, TRACK, TILE_SIZE, SENSOR_LENGTH), all the sensor are initialized 
 *  and Odometer is declared, as well as the wifi class to be able to pass parameters to the robot.
 *  Then, threads start: the display thread is launched, followed by the odometer thread and then the odometer display thread.
 *  The robot localizes. Once it finishes localizing, the odometry correction is launched, so that when the robot is travelling to 
 *  the tree, it stays on the right path, so it corrects at every tile.
 *  The robot then travels through the tunnel to the ring set. Once the tree is detected, the ring search thread is started.
 *  The robot tries to detect as many rings as possible and to pick them up when found.
 *  Once it finds all the rings, the robot travels back through the tunnel to the starting point 
 *  and drops all the rings it collected.
 * 
 * @author Jeffrey Leung
 * @author Lea Akkary
 */


public class Main {

	//Constants
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 14.25; //14.35
	public static final double TILE_SIZE = 30.48;
	public static final double SENSOR_LENGTH = 3.3;
	public static int[] startingCorner;

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	//LCD Screen Object
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	//Sensor Object
	private static final EV3ColorSensor leftLight = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private static final SensorModes us = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	private static final EV3ColorSensor color = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	private static final EV3ColorSensor rightLight = new EV3ColorSensor(LocalEV3.get().getPort("S4"));


	//Odometer
	private static final Odometer odometer = Odometer.getOdometer(leftMotor,rightMotor);

	//Controllers
	private static ColorSensorController colorSensor = new ColorSensorController(color);
	private static UltrasonicSensorController usSensor = new UltrasonicSensorController(us,lcd);
	private static LightSensorController leftLS = new LightSensorController(leftLight,lcd);
	private static LightSensorController rightLS = new LightSensorController(rightLight,lcd);
	private static RobotController robot = new RobotController(odometer,leftMotor,rightMotor);
	private static OdometryCorrection odoCorr = new OdometryCorrection(odometer,robot,leftLS,rightLS);

	public static long START_TIME = System.currentTimeMillis();

	// WiFi class
	private static WiFi wifi = new WiFi();

	//Navigation
	private static USLocalizer usLocalizer = new USLocalizer(odometer,robot,usSensor);
	private static LightLocalizer lightLocalizer = new LightLocalizer(odometer,robot,leftLS, rightLS);
	private static RingSearcher ringSearcher = new RingSearcher(odometer,colorSensor, usSensor,robot);
	private static Navigation navigation = new Navigation(odometer,robot,ringSearcher,wifi);


	/**
	 * This is the central method in which it sequentically calls methods from the navigation package.
	 * It first localizes with the ultrasonic sensor, then localizes on the starting corner with the light 
	 * sensor. Then it navigates to and through the tunnel. Then it searches for the ring set and grabs a ring.
	 * Then it returns back to the starting corner by passing through the tunnel. 
	 * 
	 * @param args 
	 */
	public static void main(String[] args) throws OdometerExceptions {

		do {
			/*----------Test sensors----------*/	
			//	Tester test = new Tester(leftMotor, rightMotor,leftLight,color,us,lcd);
			//			//Gyro
			//			test.testGyro();
			//			//Color Sensor
			//			test.testCS();
			//			///Light Sensor
			//			test.testLS();
			//			//Ultrasonic sensor
			//			test.testUS();
			//			//Turns
			//			test.testTurn();
			/*--------------END---------------*/

			//Display
			Display odometryDisplay = new Display(lcd); 

			//Odometer thread
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			//Odometer display thread
			Thread odoDisplayThread = new Thread(odometryDisplay);			
			odoDisplayThread.start();

			//Set odometry correction
			robot.setOdoCorrection(odoCorr);
			navigation.setOdoCorrection(odoCorr);
			ringSearcher.setOdoCorrection(odoCorr);

			
			robot.turnMotor();
			
			Thread usThread = new Thread(usLocalizer);
			usThread.start();

			try {
				usThread.join();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			lightLocalizer.initialLocalize();

			//Initialize odometer
			odometer.initialize(wifi.getStartingCorner(wifi.getTeam()));

			//Navigation to tunnel entrance
			navigation.travelToTunnel(); 

			//Navigation through tunnel 
			navigation.travelThroughTunnel();

			//Navigation to ring set
			navigation.travelToRingSet();

			//Ring search and grab
			Thread ringSearch = new Thread(ringSearcher);
			ringSearch.start();


			try {
				ringSearch.join();
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}

			//Navigation to tunnel exit
			navigation.travelToTunnelExit();

			//Navigation through tunnel 
			navigation.travelThroughTunnel();

			//Navigation to starting point
			navigation.travelToStartingPoint();

			robot.unload();


			Sound.beep();
			Sound.twoBeeps();
			Sound.twoBeeps();

			System.exit(0);

		}while (Button.waitForAnyPress() != Button.ID_ESCAPE); 


		System.exit(0);

	}

}