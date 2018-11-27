/**
 * * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private double Theta;

  private double displacementL;
  private double displacementR;
  private int oldLeftMotorTachoCount;
  private int oldRightMotorTachoCount;

  private double X;
  private double Y;

  private final double TRACK;
  private final double WHEEL_RAD;

  private double[] position;

  private int startingCorner;
  private int nbXLines;
  private int nbYLines;
  private static final double TILE_LENGTH = 30.48;


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD, int startingCorner) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    this.Theta = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

    this.startingCorner = startingCorner;
  }

  public void initialize() {
    switch (startingCorner) {
      case 0:
        nbXLines = 0;
        nbYLines = 0;
        odo.setXYT(0.0, 0.0, 0.0);
        break;
      case 1:
        nbXLines = 6;
        nbYLines = 0;
        odo.setXYT(nbXLines * TILE_LENGTH, 0.0, 270.0);
        break;
      case 2:
        nbXLines = 6;
        nbYLines = 6;
        odo.setXYT(nbXLines * TILE_LENGTH, nbYLines * TILE_LENGTH, 180.0);
        break;
      case 3:
        nbXLines = 0;
        nbYLines = 6;
        odo.setXYT(0.0, nbYLines * TILE_LENGTH, 90.0);
        break;
    }

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD,
      int startingCorner) throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD, startingCorner);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();

      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      double dX, dY, dTheta, dDisplace;

      // TODO Calculate new robot position based on tachometer counts

      displacementL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - oldLeftMotorTachoCount) / 180;
      displacementR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - oldRightMotorTachoCount) / 180;

      oldLeftMotorTachoCount = leftMotorTachoCount;
      oldRightMotorTachoCount = rightMotorTachoCount;
      dDisplace = 0.5 * (displacementL + displacementR);
      dTheta = (displacementL - displacementR) / TRACK;
      Theta += dTheta;

      dX = dDisplace * Math.sin(Theta);
      dY = dDisplace * Math.cos(Theta);


      // TODO Update odometer values with new calculated values
      odo.update(dX, dY, dTheta * 180 / Math.PI);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
