package ca.mcgill.ecse211.localization;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.localization.*;
import lejos.hardware.Button;

/**
 * 
 * @author Babettesmith
 *
 */
public class localization {

  //Vaiables for motors and sensors 
    private static final Port colorPort = LocalEV3.get().getPort("S2");  
  
    private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

    private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

    private static final TextLCD lcd = LocalEV3.get().getTextLCD();

    public static final double WHEEL_RAD = 2.2;
    public static final double TRACK = 13.1;
    public static boolean edge=false;
	
	/**
	 * Main method, allows user to inteact with brick 
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {
	  
	    //Sets up colour sensor and array holding data
	    SensorModes colorSensor = new EV3ColorSensor(colorPort);
	    SampleProvider colorValue = colorSensor.getMode("RGB");
        float[] colorData = new float[colorValue.sampleSize()];
      
        //Creates odometer object
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		
		//Creates navigation object 
		Navigation navi = new Navigation(odometer);
		
		//Creates ultrasonic localizer object
		UltrasonicLocalizer us_localizer = new UltrasonicLocalizer(leftMotor,rightMotor,TRACK,WHEEL_RAD);
		
		//Creates light localizer object
		LightLocalizer light_localizer= new LightLocalizer(odometer, colorValue, colorData, navi);
		
		int buttonChoice;
		
		//Creates Display object 
		Display odometryDisplay = new Display(lcd);
		do {
			// clear the lcd display
			lcd.clear();

			//Prompts the user to select falling edge or rising edge 
			lcd.drawString("< Left |  Right >", 0, 0);
			lcd.drawString("       |         ", 0, 1);
			lcd.drawString("Rising |  Falling", 0, 2);
			lcd.drawString("edge   |  edge   ", 0, 3);
			lcd.drawString("       | 		 ", 0, 4);

			//Waits for the user to make a selection
			buttonChoice = Button.waitForAnyPress();
		} 
		
		while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		//Robot performs rising edge localization, waits for user to press a button then performs light localization
		if (buttonChoice == Button.ID_LEFT) {
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			us_localizer.run();
			buttonChoice = Button.waitForAnyPress();
			light_localizer.doLocalization();
			
		} else { //Robot performs falling edge localization, waits for user to press a button then performs light localization
		    edge = true;
		    Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			us_localizer.run();
			buttonChoice = Button.waitForAnyPress();
			light_localizer.doLocalization();
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

}
