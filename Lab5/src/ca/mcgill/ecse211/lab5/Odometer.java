package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.OdometerData;
import ca.mcgill.ecse211.lab5.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  int lastTachoCountL; 
  int lastTachoCountR;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;//distance between 2 centers of wheel
  private final double WHEEL_RAD;
  
  private double[] position;


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param TRACK
   * @param WHEEL_RAD
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
	  	lastTachoCountL = leftMotor.getTachoCount();
	  	this.rightMotorTachoCount = 0;
	  	lastTachoCountR = rightMotor.getTachoCount();

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
    double distL,distR,dTheta,dDist,dX,dY,angle;
    while (true) {
      updateStart = System.currentTimeMillis();

      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      //Calculate new robot position based on tachometer counts
      distL = Math.PI*WHEEL_RAD*(leftMotorTachoCount - lastTachoCountL)/180;//calculate the distance of left wheel move 
      distR = Math.PI*WHEEL_RAD*(rightMotorTachoCount - lastTachoCountR)/180;//calculate the distance of right wheel move 
      lastTachoCountL = leftMotorTachoCount;// update the old value of TachoCount
      lastTachoCountR = rightMotorTachoCount;
      dDist = (distL + distR)/2;//calculate the distance of  center of rotation moves
      dTheta = (distL - distR)/TRACK;//calculate the delta theta value;
      
      if (dTheta < 0.0004 && dTheta > (-0.0004f)) {
    	  dTheta = 0;
      }
      
      position = odo.getXYT();
      angle = (position[2]*Math.PI/180 + dTheta);//get the old theta value in radian form and calculate the new theta value
      dX = dDist * Math.sin(angle);//calculate the delta X by trigonometric function
      dY = dDist * Math.cos(angle);//calculate the delta Y by trigonometric function
      
      //Update odometer values with new calculated values
      odo.update(dX,dY,dTheta*180/Math.PI);// update  the new value

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
