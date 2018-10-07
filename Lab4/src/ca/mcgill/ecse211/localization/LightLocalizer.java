package ca.mcgill.ecse211.localization;


import ca.mcgill.ecse211.localization.*;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
  
  private Odometer odometer;
  private SampleProvider colorSensor;
  private float[] colorDataArray;
  private double[] angles;
  private double[] odometerAngles;
  private int angleIndex;
  private double firstColour;
 
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  Navigation navigator; 
  public static int ROT_SPEED = 100;
  public static int FWD_SPEED = 150;
  public static int BANDWIDTH = 18;
  public static int ACCELERATION = 600;
  private static double colourPercentDiff = 20;  
  private static int lightSensorDistance = 8;
    
    
    
    //Constructor for LightLocalizer
    public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData, Navigation navi) {
        this.odometer = odo;
        this.colorSensor = colorSensor;
        this.colorDataArray = colorData;
        this.navigator = navi;
        EV3LargeRegulatedMotor[] motors = odo.getMotors();
        this.leftMotor = motors[0];     
        this.rightMotor = motors[1];
        this.leftMotor.setAcceleration(ACCELERATION);
        this.rightMotor.setAcceleration(ACCELERATION);
        //initialize arrays
        angles = new double[4];
        odometerAngles = new double[4];
        angleIndex = 0;
    }
    /**
     * Gets robot to do localization using light sensor 
     */
    public void doLocalization() {
      firstColour = getColorData();

      //Robot should be at y-axis position from the ultrasonic localizer so robot moves forward in that direction 
      leftMotor.setSpeed(FWD_SPEED);
      rightMotor.setSpeed(FWD_SPEED);
      leftMotor.forward();
      rightMotor.forward();
      
      //Robot keeps moving until it senses the first black line 
      while(100*Math.abs(getColorData() - firstColour)/firstColour < colourPercentDiff){
          try {
              Thread.sleep(100);
          } catch (InterruptedException e) {
              e.printStackTrace();
          }
      }
      
     
      leftMotor.stop(true);
      rightMotor.stop(true);
      
      //Robot has sensed a black line so backs up and rotates 90 degrees
      leftMotor.rotate(-convertDistance(localization.WHEEL_RAD, BANDWIDTH), true); 
      rightMotor.rotate(-convertDistance(localization.WHEEL_RAD, BANDWIDTH), false);
      
     
      leftMotor.setSpeed(ROT_SPEED);
      rightMotor.setSpeed(ROT_SPEED);
      leftMotor.rotate(convertAngle(localization.WHEEL_RAD, localization.TRACK, 90.0), true);
      rightMotor.rotate(-convertAngle(localization.WHEEL_RAD, localization.TRACK, 90.0), false);
      
      //Robot now searches for black line on x-axis
      leftMotor.setSpeed(ROT_SPEED);
      rightMotor.setSpeed(ROT_SPEED);
      leftMotor.forward();
      rightMotor.forward();
      
      while(100*Math.abs(getColorData() - firstColour)/firstColour < colourPercentDiff){
          try {
              Thread.sleep(100);
          } catch (InterruptedException e) {
              e.printStackTrace();
          }
      }
      
      
      leftMotor.stop(true);
      rightMotor.stop(true);
      
    //Robot has sensed a black line so backs up and rotates 90 degrees
      leftMotor.rotate(-convertDistance(localization.WHEEL_RAD, BANDWIDTH), true); 
      rightMotor.rotate(-convertDistance(localization.WHEEL_RAD, BANDWIDTH), false);
      
      
      /*The robot now searches for four consecutive black lines so it can locate a corner point. It does this by rotating 360 degrees
      recording the angle the robot is at every time it senses a black line*/
      leftMotor.setSpeed(ROT_SPEED);
      rightMotor.setSpeed(ROT_SPEED);
      leftMotor.backward();
      rightMotor.forward();

      while(angleIndex < 4){
          
          
          if(100*Math.abs(getColorData() - firstColour)/firstColour > colourPercentDiff){ 
              angles[angleIndex] = odometer.getAng(); 
              angleIndex+=1;
              Sound.beep();
              try {
                  Thread.sleep(200); //After testing, this sleep value was necessary so one line wasn't sensed multiple times 
              } catch (InterruptedException e) {
                  e.printStackTrace();
              }
          }
      }
      
      //4 lines have been sensed so the robot stops
      leftMotor.stop(true);
      rightMotor.stop(false);
      
      
      
  
      //0th element = first y line
      //1st element = first x line
      //2nd element = second y line
      //3rd element= second x line
      
      //Calculates changes in x and y
      double deltaY = angles[2] - angles[0];
      double deltaX = angles[3] - angles[1];
      
      //Computes (0,0) point and degree needed to travel at
      double xValue = lightSensorDistance*Math.cos(Math.PI*deltaX/(360));
      double yValue = lightSensorDistance*Math.cos(Math.PI*deltaY/(360));
      double thetaYMinus = angles[0] ;
      
      //double deltaTheta = +180 - deltaY/2 - thetaYMinus;
  
     //Turns to point at (0,0) point 
      //navigator.turnTo(90, true); 
      
      //leftMotor.stop(true);
      //rightMotor.stop(true);
      
      
     // leftMotor.rotate(convertAngle(localization.WHEEL_RAD, localization.TRACK, -35.0), true);
      //rightMotor.rotate(-convertAngle(localization.WHEEL_RAD, localization.TRACK, -35.0), false);
      
      //Updates position of robot through the odometer
     //odometer.setPosition(new double [] {-xValue, -yValue, 0}, new boolean [] {true, true, true});
      
      //Robot travels to (0,0) coordinate and turns to y-axis
      navigator.travelTo(xValue, yValue);
      
      leftMotor.rotate(convertAngle(localization.WHEEL_RAD, localization.TRACK, -45), true);
      rightMotor.rotate(-convertAngle(localization.WHEEL_RAD, localization.TRACK, -45), false);
      
      //odometer.setPosition(new double [] {0, 0, 0}, new boolean [] {true, true, true});
      //odometer.setPosition(new double [] {/*-xValue, -yValue*/180,180, 0}, new boolean [] {true, true, true});
      //navigator.turnTo(70 + 180 , true);
      
       
  }
    
    
    
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }
    
    
    private float getColorData() {
        colorSensor.fetchSample(colorDataArray, 0);
        float colorBrightnessLevel = (colorDataArray[0] + colorDataArray[1] + colorDataArray[2]);
        return colorBrightnessLevel;
    }


}