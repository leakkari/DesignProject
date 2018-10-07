/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.localization;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; 

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;
  private int lastTachoL;
  private int lastTachoR;
  private double X=0;
  private double Y=0;
  private double Theta = 0;
  private double distL;
  private double distR;
  private double deltaD; 
  private double deltaT;
  private double dX; 
  private double dY;

  //Regularity at which odometer updates 
  private static final long ODOMETER_PERIOD = 25; 

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

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
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
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
    		
    		//computes distance traveled by the left and right wheels periods
    		distL=3.1415926*WHEEL_RAD*(leftMotorTachoCount-lastTachoL)/180;  
    		distR=3.1415926*WHEEL_RAD*(rightMotorTachoCount-lastTachoR)/180; 
    		
    		//Updates left and right wheel tacho counts 
    		lastTachoL=leftMotorTachoCount; 
    		lastTachoR=rightMotorTachoCount;
    		
    		//Computes changes in distances and theta and updates them 
    		deltaD=0.5*(distL+distR);
    		deltaT=(distL-distR)/TRACK; 
    		Theta+=deltaT; 
    		dX=deltaD*Math.sin(Theta); 
    		dY=deltaD*Math.cos(Theta);
    		X=X+dX;
    		Y=Y+dY;		
    		
			//Updates the odometer with the new information 
    	  	odo.update(dX, dY, deltaT*180/Math.PI);
      


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
  
  /**
   * Getter method for motors 
   * @return
   */
  public EV3LargeRegulatedMotor [] getMotors() {
    return new EV3LargeRegulatedMotor[] {this.leftMotor, this.rightMotor};
  }
  
  /**
   * Getter method for left motor 
   * @return
   */
  public EV3LargeRegulatedMotor getLeftMotor() {
    return this.leftMotor;
  }
  
  /**
   * Getter mother for right motor 
   * @return
   */
  public EV3LargeRegulatedMotor getRightMotor() {
    return this.rightMotor;
  }

  /**
   * Getter method for angle 
   * @return
   */
  public double getAng() {
    synchronized (this) {
      return Theta;
    }
  }   

  /**
   * Setter method for robot position 
   * @param position
   * @param update
   */
  public void setPosition(double[] position, boolean[] update) {
    synchronized (this) {
      if (update[0])
          X = position[0];
      if (update[1])
          Y = position[1];
      if (update[2])
          Theta = position[2];
    }
  }

/**
 * Getter method for x coordinate
 * @return
 */
  public double getX() {
    synchronized (this) {
      return X;
    }
  }

/**
 * Getter method for y coordinate 
 * @return
 */
  public double getY() {
    synchronized (this) {
      return Y;
    }
  }   
}
