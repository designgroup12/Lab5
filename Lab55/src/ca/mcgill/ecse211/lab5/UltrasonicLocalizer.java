package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Lab5;
import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {
	
	
  private static int ROTATE_SPEED = 80;
	
	
  private static Odometer odo;
  private static EV3LargeRegulatedMotor leftMotor ;
  private static EV3LargeRegulatedMotor rightMotor;
	
  private static SampleProvider sampleProviderUS;
  static EV3UltrasonicSensor usSensor;
  static float[] usValues;
	
	
  int SC; 
  /**
   * contructor
   * @param sampleProviderUS 
   * @param usValues
   * @param leftMotor
   * @param rightMotor
   * @param type        rising or falling
   * @throws OdometerExceptions
   */
  public UltrasonicLocalizer(SampleProvider sampleProviderUS, float[] usValues, EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor,  int SC) throws OdometerExceptions{
		
    odo =  Odometer.getOdometer();
    this.sampleProviderUS = sampleProviderUS;
    this.usValues = usValues;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.SC = SC;
  }
	
	public  void doLocalization() {
		
		double alpha; //odo.getXYT()[2];
		double beta;
		double deltaTheta = 0;
		double orientation;
		double wallDistance = 20.0; //GONNA NEED TO CHANGE THIS ONE FOR ACCURACY 
	
		
		 for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			  motor.stop();
			  motor.setAcceleration(3000);
		  }

		  // Sleep for 2 seconds
		  try {
			  Thread.sleep(2000);
		  } catch (InterruptedException e) {
			  // There is nothing to be done here
		  }
		  
			  
			  leftMotor.setSpeed(ROTATE_SPEED);
		      rightMotor.setSpeed(ROTATE_SPEED);
		      
		      //rotate until the robots doesn't see a wall
		      while(getDistance() < wallDistance + 3.5) {
		    	  leftMotor.backward();
		    	  rightMotor.forward();
		      }
		      Sound.beep(); //to notify that the robot doesn't see a wall anymore
		      
		      //rotate until robot sees a wall
		      while(getDistance() > wallDistance) {
		    	  leftMotor.backward();
		    	  rightMotor.forward();
		      }
		      leftMotor.stop(true);
		      rightMotor.stop();
		      alpha = odo.getXYT()[2];
		      Sound.beep(); //robot sees a wall
		      
		     
		      
		      //change direction
		      //rotate until sees no wall
		      while(getDistance() < wallDistance + 3.5) {
		    	  leftMotor.forward();
		    	  rightMotor.backward();
		      }
		      
		      Sound.beep(); //doesn't see a wall anymore
		      
		      //rotate until it sees a wall
		      while(getDistance() > wallDistance) {
		    	  leftMotor.forward();
		    	  rightMotor.backward();
		      }
		      leftMotor.stop(true);
		      rightMotor.stop();
		      Sound.beep(); //sees a wall
		     
		      beta = odo.getXYT()[2];
		      
		      //stops the robot
		      leftMotor.stop(true);
		      rightMotor.stop();
		      
		      //calculate deltatheta
		      if(alpha < beta) {
		    	  deltaTheta = 45 - (alpha + beta)/2;
		      }
		      else if(alpha > beta) {
		    	  deltaTheta = 225 - (alpha + beta)/2;
		      }
		      
		      //orientation of the robot
		      orientation = deltaTheta + odo.getXYT()[2];
			  
		      //update the position of the robot
		      odo.setTheta(orientation);
		      
		      //turn to 0 degrees
		      if(orientation <= 180) {
		    	leftMotor.setSpeed(ROTATE_SPEED);
		  		rightMotor.setSpeed(ROTATE_SPEED);
		  		leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, orientation-1), true);
		  		rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, orientation-1), false);
		      }else {
		    	  orientation = 360 - orientation;
		    	  leftMotor.setSpeed(ROTATE_SPEED);
		    	  rightMotor.setSpeed(ROTATE_SPEED);
			  	  leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, orientation+2), true);
			  	  rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, orientation+2), false);
			      
		      }
		      odo.setTheta((0 + 360 -this.SC*90)%360);
		
		
	}
	
	private static float getDistance() {
		Lab5.usSensor.fetchSample(usValues, 0);
		
		float distance = 100*(average(usValues));
		
		//TODO ADD FILTER
		
		return distance;
	}
	private static float average(float[] usvalues) {
		float sum = 0;
		int i = 0;
		for (float values: usvalues) {
			sum += values;
			i++;
		}
		return sum/i;
	}
	
	private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	 
	private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	 }
}


