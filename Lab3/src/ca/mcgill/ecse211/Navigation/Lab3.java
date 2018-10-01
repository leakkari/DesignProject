
package ca.mcgill.ecse211.Navigation;


import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {
	
	// Parameters: adjust these for desired performance

	  private static final int bandCenter = 20; // Offset from the wall (cm)
	  private static final int bandWidth = 2; // Width of dead band (cm)
	  private static final int motorLow = 100; // Speed of slower rotating wheel (deg/sec)
	  private static final int motorHigh = 200; // Speed of the faster rotating wheel (deg/seec)

	  private static int flag = 0;
	  
  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

 
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 12.9;

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    Navigation navigation = new Navigation(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    Display odometryDisplay = new Display(lcd); 
    
    
    do {
        // clear the display
        lcd.clear();

        // ask the user whether the motors should drive in a square or float
        lcd.drawString("< Left |  Right >", 0, 0);
        lcd.drawString("       |         ", 0, 1);
        lcd.drawString(" Float | Navigate", 0, 2);
        lcd.drawString("motors | 	     ", 0, 3);
        lcd.drawString("       | 		 ", 0, 4);

        buttonChoice = Button.waitForAnyPress();
      } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

      if (buttonChoice == Button.ID_LEFT) {

        leftMotor.forward();
        leftMotor.flt();
        rightMotor.forward();
        rightMotor.flt();

     // Display changes in position as wheels are (manually) moved
        
        Thread odoThread = new Thread(odometer);
        odoThread.start();
        Thread odoDisplayThread = new Thread(odometryDisplay);
        odoDisplayThread.start();

      }
      else {
          // clear the display
          lcd.clear();

          // ask the user whether the motors should drive in a square or float
          lcd.drawString("< Left  | Right >", 0, 0);
          lcd.drawString(" without | with   ", 0, 1);
          lcd.drawString("  obs - | obs-   ", 0, 2);
          lcd.drawString("  tacle | tacle  ", 0, 3);
          lcd.drawString("        |        ", 0, 4);
          
          buttonChoice = Button.waitForAnyPress();
          
       // Start odometer and display threads
          Thread odoThread = new Thread(odometer);
          odoThread.start();
          Thread odoDisplayThread = new Thread(odometryDisplay);
          odoDisplayThread.start();
          
          if(buttonChoice == Button.ID_LEFT){
        	 navigation.run();
          }
          
          if(buttonChoice == Button.ID_RIGHT){	 
	         obstacleAvoidance.run();
        	 
        
          }
        }

        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        System.exit(0);
      }
    }