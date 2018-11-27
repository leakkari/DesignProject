/*
 * This class serves to link all the classses together and display a menu when running the code
 * 
 * @author Yassine Douida
 * 
 * @author Jeffrey Leung
 */
package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.localization.*;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.colorClassification.ColorClassification;
import ca.mcgill.ecse211.lab5.*;

public class Lab5 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  private static final Port usPort = LocalEV3.get().getPort("S2");

  // Set vehicle constants
  public static final double WHEEL_RAD = 2.1;
  public static final double TRACK = 10.0; // 10.5

  private static final int FORWARD_SPEED = 250;
  private static final int ROTATE_SPEED = 150;
  private static final double TILE_SIZE = 30.48;

  // { LLx, LLy, UUx, UUy, TR, SC }
  public static int[] variable = new int[]{1, 1, 7, 7, 1, 0};


  public static int printMenu(String label, int content) {
    int buttonChoice;
    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString(label, 0, 0);
      lcd.drawString("" + content + "", 8, 2);
      lcd.drawString("<  Left |  Right >", 0, 4);
      lcd.drawString("Press Enter", 4, 7);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        && buttonChoice != Button.ID_ENTER && buttonChoice != Button.ID_ESCAPE);
    return buttonChoice;
  }

  public static int[] menu(int[] variable) {
    String tag[] = {"LLx", "LLy", "URx", "URy", "TR ", "SC "};
    String label[] = new String[6];
    for (int i = 0; i < label.length; i++) {
      label[i] = "<   Choose " + tag[i] + "   >";
    }
    int maxRange[] = {8, 8, 8, 8, 4, 3};
    int minRange[] = {0, 0, 0, 0, 1, 0};
    int labelContent[] = {0, 0, 0, 0, 1, 0};

    int i = 0;
    while (i < 6) {
      int buttonChoice;
      buttonChoice = printMenu(label[i], labelContent[i]);
      if (buttonChoice == Button.ID_LEFT) {
        if (labelContent[i] != minRange[i]) {
          labelContent[i]--;
        }
      }
      if (buttonChoice == Button.ID_RIGHT) {
        if (labelContent[i] != maxRange[i]) {
          labelContent[i]++;
        }
      }
      if (buttonChoice == Button.ID_ENTER) {
        variable[i] = labelContent[i];
        i++;
      }
      if (buttonChoice == Button.ID_ESCAPE) {
        System.exit(0);
      }
    }
    return variable;

  }

  public static void main(String[] args) throws OdometerExceptions {


    do {

      // Odometer objects
      Odometer odometer =
          Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD, variable[5]);
      Display odometryDisplay = new Display(lcd); // No need to change

      // OdometryCorrection odometryCorrection = new OdometryCorrection(variable[5]);

      @SuppressWarnings("resource") // Because we don't bother to close this resource
      // usSensor is the instance
      SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
      // usDistance provides samples from this instance
      SampleProvider usDistance = ultrasonicSensor.getMode("Distance");


      // Odometer thread
      Thread odoThread = new Thread(odometer);
      odoThread.start();

      // Odometer display thread
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      // Center to 0 axis with USLocalizer (true: Rising edge / false: Falling edge)
      // USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, true,
      // usDistance);
      // USLocalizer.localizeRisingEdge();

      // Localize robot to origin with LightLocalizer
      LightLocalizer lightLocalizer = new LightLocalizer(odometer, leftMotor, rightMotor);
      lightLocalizer.moveToOrigin();
      lightLocalizer.localize(0.0, 0.0);
      odometer.initialize();

      // NOTE: Origin is at (1,1)

      // Navigate to (LLx,LLy)
      //Navigation navigation = new Navigation(odometer, leftMotor, rightMotor);
     // navigation.travelTo(0, variable[1] * TILE_SIZE); // (0, LLy)
     // navigation.travelTo(variable[0] * TILE_SIZE, variable[1] * TILE_SIZE);    // (LLx, LLy)

      // Localizer
     // lightLocalizer.localize(variable[0] * TILE_SIZE, variable[1] * TILE_SIZE);

      // Searching
      ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance(odometer, leftMotor, rightMotor, usDistance);
    
      Thread snakeThread = new Thread(obstacleAvoidance);
      snakeThread.run();

      //      // Color class object
      //      ColorClassification colorClass =
      //          new ColorClassification(lcd, leftMotor, rightMotor, variable[4], usDistance);
      //
      //      // USNavigation Thread
      //      USNavigation usNavigation =
      //          new USNavigation(odometer, rightMotor, leftMotor, usDistance, colorClass, variable);
      //      Thread usThread = new Thread(usNavigation);
      //      usThread.start();

    } while (Button.waitForAnyPress() != Button.ID_ESCAPE);

    System.exit(0);
  }

}
