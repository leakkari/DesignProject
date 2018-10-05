package ca.mcgill.ecse211.localization;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;

public class localization {

	// Motor Objects, and Robot related parameters

	//private static final EV3ColorSensor colorSensor =
	//new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 16.63;
	public static boolean edge=false;


	public static void main(String[] args) throws OdometerExceptions {
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		UltrasonicLocalizer us_localizer = new UltrasonicLocalizer(leftMotor,rightMotor,TRACK,WHEEL_RAD);
		LightLocalizer light_localizer= new LightLocalizer(leftMotor,rightMotor,TRACK,WHEEL_RAD);
		int buttonChoice;
		// Odometer related objects
		//ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance(); 
		Display odometryDisplay = new Display(lcd);
		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left |  Right >", 0, 0);
			lcd.drawString("       |         ", 0, 1);
			lcd.drawString("Rising |  Falling", 0, 2);
			lcd.drawString("edge   |  edge   ", 0, 3);
			lcd.drawString("       | 		 ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			us_localizer.run();
			buttonChoice = Button.waitForAnyPress();
			light_localizer.run();
			
		} else {
			edge=true;
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			us_localizer.run();
			buttonChoice = Button.waitForAnyPress();
			light_localizer.run();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

}
