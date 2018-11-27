package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.localization.LightLocalizer;

public class SnakeDriver extends Thread {

  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  private LightLocalizer lightLocalizer;
  private SampleProvider usDistance;

  public static final double WHEEL_RAD = 2.1;
  public static final double TRACK = 9.8; // 10.5

  private static final int FORWARD_SPEED = 250;
  private static final int ROTATE_SPEED = 150;
  private static final double TILE_SIZE = 30.48;

  private int[] variable = Lab5.variable;
  private double ydist = (variable[3] - variable[1]) * TILE_SIZE;
  
  private float[] usData = new float[1];
  private float usDist;

  private double[][]  maps = new double[][]{
    {1*30.48,1*30.48}, 
    {1*30.48,6*30.48},
    {2*30,48,6*30.48},
    {2*30.48,1*30.48},
    {3*30.48,1*30.48},
    {3*30.48,6*30.48},
    {4*30.48,6*30.48},
    {5*30.48,1*30.48},
    {6*30.48,1*30.48},
    {6*30.48,6*30.48}};
  
  public SnakeDriver(Odometer odometer, EV3LargeRegulatedMotor leftMotor, 
      EV3LargeRegulatedMotor rightMotor, LightLocalizer lightLocalizer, SampleProvider usDistance) 
          throws OdometerExceptions {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.lightLocalizer = lightLocalizer;
    this.usDistance = usDistance;
  }


  public void run() {

    leftMotor.stop();
    rightMotor.stop();

    leftMotor.setAcceleration(300); // originally 3000
    rightMotor.setAcceleration(300); 

    usDistance.fetchSample(usData, 0);
    usDist = usData[0] * 100;
    
    // snake driving
    for (int i = variable[0]; i < variable[2]; i++) {
      moveUp(usDist);
      moveRight(usDist);
      moveDown(usDist);
      moveRight(usDist);

      lightLocalizer.localize((i+2) * TILE_SIZE, variable[1] * TILE_SIZE);

      i++;

    }

    // additional UP when even number of tiles on x-axis
    if ((variable[2]-variable[0]) % 2 == 0){
      // UP
      moveUp(usDist);
    } else {
      moveUp(usDist);
      moveRight(usDist);
    }

    //        navigation.travelTo(i * TILE_SIZE,    variable[3]-1 * TILE_SIZE);  // up
    //        navigation.travelTo(i+1 * TILE_SIZE,  variable[3]-1 * TILE_SIZE);  // right
    //        navigation.travelTo(i+1 * TILE_SIZE,  i * TILE_SIZE);    // down
    //        navigation.travelTo(i+2 * TILE_SIZE,  i * TILE_SIZE);
    //        lightLocalizer.localize(i+2 * TILE_SIZE, i * TILE_SIZE);
    //        i++;
  }

  boolean isNavigating() {
    if((leftMotor.isMoving() || rightMotor.isMoving()))
      return true;
    else 
      return false;
  }

  private void checkObstacles(float usDist) {

    if (usDist <= 15) {

      //checking robot position with respect to bounds to determine which way it will turn

      //check robot position, depending on position in the square, turn left
      if(odometer.getXYT()[0] < 2.4 * 30.48 
          && odometer.getXYT()[0] > 1.3 * 30.48 
          && odometer.getXYT()[1] < 2.5 * 30.48
          && odometer.getXYT()[1] > 1.6 * 30.48){

        //turn 90 degrees
        leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true); 
        rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);

        //travel 30.2 cm
        leftMotor.rotate(convertDistance(WHEEL_RAD, 30.2), true);
        rightMotor.rotate(convertDistance(WHEEL_RAD, 30.2), false);

        //turn 90 degrees
        leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
        rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);

        //travel 40 cm
        leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
        rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
      }

      // or turn right
      else {

        //turn 90 degrees
        leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);  
        rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);

        //travel 30.2 cm
        leftMotor.rotate(convertDistance(WHEEL_RAD, 30.2), true);
        rightMotor.rotate(convertDistance(WHEEL_RAD, 30.2), false);

        //turn 90 degrees
        leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
        rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);

        //travel 30.2 cm
        leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
        rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
      }

      // go back to waypoint we wanted to go to before robot had to avoid obstacle
      // i--;
    }
  }

  /** Helper methods for movement **/
  private void moveUp(float usDist){
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(convertDistance(WHEEL_RAD, ydist), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, ydist), true);

    while (isNavigating()){
      usDistance.fetchSample(usData, 0);
      usDist = usData[0] * 100;
      checkObstacles(usDist);
    }

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), true);
    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), false);
  }

  private void moveDown(float usDist){
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(convertDistance(WHEEL_RAD, ydist), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, ydist), true);

    while (isNavigating()){
      usDistance.fetchSample(usData, 0);
      usDist = usData[0] * 100;
      checkObstacles(usDist);
    }

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), true);
    rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), false);
  }

  private void moveRight(float usDist){
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(convertDistance(WHEEL_RAD, TILE_SIZE), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, TILE_SIZE), false);

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), true);
    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), false);

  }

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
