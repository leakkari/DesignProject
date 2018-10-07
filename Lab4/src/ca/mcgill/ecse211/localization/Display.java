package ca.mcgill.ecse211.localization;

import java.text.DecimalFormat;

//import ca.mcgill.ecse211.navigation.UltrasonicController;
import lejos.hardware.lcd.TextLCD;

/**
 * 
 * @author Babettesmith
 *
 */

public class Display implements Runnable {

  private Odometer odometer;
  private TextLCD lcd;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;
  
  
  /**
   * Constructor for Display Object with one parameter 
   * @param lcd
   * @throws OdometerExceptions
   */
  public Display(TextLCD lcd) throws OdometerExceptions {
    odometer = Odometer.getOdometer();
    this.lcd = lcd;
  }

 /**
  * Constructor for Display Object with two parameters
  * @param lcd
  * @param timeout
  * @throws OdometerExceptions
  */
  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
    odometer = Odometer.getOdometer();
    this.timeout = timeout;
    this.lcd = lcd;
  }

  /**
   * run method 
   */
  public void run() {
    
    lcd.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odometer.getXYT();
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      //lcd.drawString("US Distance: " + cont.readUSDistance(), 0, 3); // print last US reading      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}
