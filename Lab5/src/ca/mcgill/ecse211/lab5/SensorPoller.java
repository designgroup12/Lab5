package ca.mcgill.ecse211.lab5;


import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class SensorPoller extends Thread {
	  private SampleProvider us;
	  private Navigation navigation;
	  private int maxDistance = 10;
	  private double realDistance;
	  static boolean obstacle = false;
	  private static int FILTER_OUT = 5;
	  private int filterControl = 0; 
	  private static EV3LargeRegulatedMotor leftMotor;
	  private static EV3LargeRegulatedMotor rightMotor;
	  
	  public static float[] usData;

	  public SensorPoller(SampleProvider us, float[] usData, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,Navigation navigation) {
	    this.us = us;
	    this.usData = usData;
	    this.leftMotor = leftMotor;
	    this.rightMotor = rightMotor;
	    this.navigation = navigation;
	  }

	  /*
	   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	   * [0,255] (non-Javadoc)
	   * 
	   * @see java.lang.Thread#run()
	   */
	  public void run() {
	    int distance;
	    while (true) {
	      us.fetchSample(usData, 0); // acquire data
	      distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
	      if (distance > 200) {
	    	  distance = 200;
	      }
	      
	      if ((distance < maxDistance)  && filterControl < FILTER_OUT) {
		      // bad value, do not set the distance var, however do increment the
		      // filter value
		      filterControl++;
		    } else if (distance < maxDistance) {
		      // We have repeated large values, so there must actually be nothing
		      // there: leave the distance alone
		      realDistance = distance;
		    } else {
		      // distance went below 255: reset filter and leave
		      // distance alone.
		      filterControl = 0;
		      realDistance = distance;
		    }
	      
	      if((realDistance < maxDistance)&&Navigation.isNavigating()) {
	    	  leftMotor.stop();
	    	  rightMotor.stop();
	    	  try {
				navigation.avoid(realDistance);
			} catch (OdometerExceptions e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    	 // Lab3.nextPoint--;
	    	  //obstacle = false;
	      }
	      
	      try {
	        Thread.sleep(50);
	      } catch (Exception e) {
	      } // Poor man's timed sampling
	      
	      
	      
	      
	    }
	  }

	}



