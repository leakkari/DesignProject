package ca.mcgill.ecse211.colorClassification;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.lab5.*;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class ColorClassification {

  //Light sensor
  private static final Port lightSampler = LocalEV3.get().getPort("S4");
  private EV3ColorSensor colosSamplerSensor = new EV3ColorSensor(lightSampler);
  private SampleProvider colorIDValue = colosSamplerSensor.getColorIDMode();
  private SampleProvider rgbValue = colosSamplerSensor.getMode("RGB");
  
  //Motors
  private EV3LargeRegulatedMotor leftMotor, rightMotor;

  // Ultrasonic test
  //private static final Port usPort = LocalEV3.get().getPort("S2");
 // private SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
  //private SampleProvider usDistance = ultrasonicSensor.getMode("Distance");
  //private float[] usData = new float[1];
  private boolean traveling = false;
  private int distance;
  
  private float[] rgbData = new float[3];
  private float[] colorData = new float[1];

  private TextLCD lcd;

  private int counter;
  
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;
  
  // mean and stdev values (from sampling)
  private float meanR, meanG, meanB;
  private float stdevR, stdevG, stdevB;

  private int targetRing;
  
  public ColorClassification(TextLCD lcd, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int targetRing) throws OdometerExceptions {
    this.lcd = lcd;
    this.leftMotor = leftMotor;
	this.rightMotor = rightMotor;
	this.targetRing = targetRing;
  }

  public ColorClassification(TextLCD lcd, long timeout) throws OdometerExceptions {
    this.lcd = lcd;
    this.timeout = timeout;
  }

  public void run() {

    int buttonChoice;
    boolean detecting = true;
    
    lcd.clear();

    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    
 
    do {
      updateStart = System.currentTimeMillis();

      // ultrasonic test
//      leftMotor.setSpeed(150);
//      rightMotor.setSpeed(150);
//      leftMotor.forward();
//      rightMotor.forward();
      
      // Retrieve color sensor sample
      colorIDValue.fetchSample(colorData, 0);
      float valueColorID = colorData[0];
      String colorName = "";

      switch((int) valueColorID){
        case Color.NONE: colorName  =   "NONE  "; break;
        case Color.BLACK: colorName =   "BLACK "; break;
        case Color.BLUE: colorName  =   "BLUE  "; break;
        case Color.GREEN: colorName =   "GREEN "; break;
        case Color.YELLOW: colorName =  "YELLOW"; break;
        case Color.RED: colorName   =   "RED   "; break;
        case Color.WHITE: colorName =   "WHITE "; break;
        case Color.BROWN: colorName =   "BROWN "; break;
      }

      // ultrasonic test
      //usDistance.fetchSample(usData, 0);
      //float usDist = usData[0] * 1000;

      // fetch rgb values to rgbData
      rgbValue.fetchSample(rgbData, 0);
      float valueR = rgbData[0] * 1000;
      float valueG = rgbData[1] * 1000;
      float valueB = rgbData[2] * 1000;
      
      // Print color information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      if (valueColorID != 0){
        lcd.drawString(" Object Detected\n", 0, 0);
      }
      
      lcd.drawString("Color: " + colorName, 0, 4);
      lcd.drawString("R: " + numberFormat.format(valueR) , 0, 5);
      lcd.drawString("G: " + numberFormat.format(valueG) , 0, 6);
      lcd.drawString("B: " + numberFormat.format(valueB) , 0, 7);
      //lcd.drawString("Distance: " + numberFormat.format(usDist) , 0, 8);
      
      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while (((updateEnd - tStart) <= timeout));
    
  
    	
  }

  /**
   * Returns the Euclidean distance, d, between the RGB values of a measured sample.
   */
  private float euclidDist(float r, float g, float b) {

    float[] norm = normalize(r,g,b);
    float sr = norm[0];
    float sg = norm[1];
    float sb = norm[2];
    
    return (float) Math.sqrt(Math.pow(sr - meanR, 2) 
        + Math.pow(sg - meanG, 2) + Math.pow(sb - meanB, 2));
    
  }
  
  /**
   * Returns the normalized RGB values in a float array. 
   * 
   * @param r   red value.
   * @param g   green value.
   * @param b   blue value.
   * @return    f[0] = norm(r), f[1] = norm(g), f[2] = norm(b)
   */
  private float[] normalize(float r, float g, float b) {
    
    float[] hat = new float[3];
    
    hat[0] = (float) (r / Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2) + Math.pow(b, 2))); 
    hat[1] = (float) (g / Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2) + Math.pow(b, 2))); 
    hat[2] = (float) (b / Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2) + Math.pow(b, 2))); 
    
    return hat;
    
  }
  
//  @Override
//  public void processUSData(int distance){
//    this.distance = distance;
//
//  }
//  
//  @Override
//  public int readUSDistance() {
//    return this.distance;
//  }

}
