package ca.mcgill.ecse211.lab5;


import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;


public class LightLocalizer {
	
  private static int ROTATE_SPEED = 65; // turning speed of the car
  private static int FORWARD_SPEED = 90;// forward speed of the car
  public static float curColor = 0;// the reflection value received by the light sensor
  private int j = 0;  //the number of filtered value that light sensor received 
  private int size = 12;// the size of array of results for storing the reflection value
  private float[ ] results ;  // the array of storing the reflection value to make the comparison to check if meet the deadline
  static Odometer odo; // the odometer 
  
  int SC;
  
  // the motors of the car
  private static EV3LargeRegulatedMotor leftMotor ;
  private static EV3LargeRegulatedMotor rightMotor;
	
  //the light sensor and related object
  private  SensorMode sampleProviderLight;
  EV3ColorSensor lightSensor;
  float[] lightValues;
  private static final Port colorPort = LocalEV3.get().getPort("S3");
	
  private static double distanceLS = 12.0; //distance between the light sensor and the center of rotation: change in function of robot
  private static final double tileSize = 30.48;	
  /**
    * This is the class constructor
    * 
    * @param leftmotor and right motor
    * @throws OdometerExceptions 
  */
  public LightLocalizer( EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor,int SC) throws OdometerExceptions{
		
    odo =  Odometer.getOdometer();
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;	
    // set up the light sensor
    this.lightSensor = new EV3ColorSensor(colorPort);
    this.sampleProviderLight = lightSensor.getMode("Red");
    this.lightValues = new float[sampleProviderLight.sampleSize()];
    this.results = new float[size];
    for (int k = 0;k<size;k++) { //initialize the results array
      this.results[k] = 100;
    }
    this.SC = SC;
  }
  /**
    * the doLocalization method  is the method to find the x and y coordinate, to find them ,we need to use the trigonometric 
    * and find the angle between car and x , y axis
    *  first find a place which could read 4 lines on the grid,
    * then do the spinning the record four value of angle which two on y-axis and two on x axis,
    * after that the car use four value to calculate the delta theta on x and y,
    *  divided by two and use the cos() function to get the x and y co0rdinate
    */
  public void doLocalization() {//
    int nLines = 0;// number of line passed during the spining
    boolean seenLine = false; // the boolean value to avoid reading the black line twice 
    double angle = 0;
    double x1 = 0; // the first angle value of the horizontal line
    double x2 = 0; // the second angle value of the horizontal line
    double y1 = 0; // the first angle value of the vertical line
    double y2 = 0; // the second angle value of the vertical line
    double finalx; // the calculated x coordinate value of the position
    double finaly; // the calculated x coordinate value of the position
    double distance;// the distance between the position and the origin
    double Rx;// the radian value of delta thetax
    double Ry;//the radian  value of delta thetay
		 
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
    closeToOrigin();
		  
    //robot spins 360
    getData();
    while (nLines < 4) {  
      carSetSpeed(ROTATE_SPEED);
      carMoveForward();
      getData();
      if (meetBlacklineWithoutGet() && !seenLine) {
        angle = odo.getXYT()[2] + this.SC*90;
        Sound.beep();
        seenLine = true;
        nLines ++;
				  
        //makes sure angle is from the origin 
        if(angle > 360) {
          angle = angle % 360;
        }
       //check the angle is on x-axis or y-axis and avoid fetch same value
        if((angle - 0 < 50) || (360 - angle <50)) {
          x1 = angle;
        }
        else if(Math.abs(angle - 90) < 50) {
          y1 = angle;
        }
        else if (Math.abs(angle-180) < 50) {
          x2 = angle;
        }
        else if (Math.abs(angle-270) < 50) {
          y2 = angle;
        }
      initializeResults();			  		  
      }else {
        seenLine = false;
      }
			  
    }
		  
    //stop motors
    carStop();
    Rx = Math.toRadians((x2-y1)/2);
    Ry = Math.toRadians((y2-y1)/2);
    //set final x and y coordinate of robot
    finalx = Math.cos(Rx)*distanceLS;
    finaly = Math.cos(Ry)*distanceLS;
	if (this.SC == 0) {	  
      odo.setX(tileSize-finalx);
      odo.setY(tileSize-finaly);
	}else if (this.SC == 1) {
	  odo.setX(7*tileSize + finalx);
	  odo.setY(tileSize-finaly);
	}else if (this.SC == 2) {
	  odo.setX(7*tileSize + finalx);
	  odo.setY(7*tileSize + finaly);
	}else {
	  odo.setY(7*tileSize + finaly);
	  odo.setX(tileSize - finalx);
	}
    //travel to 0,0
   // gotoOrigin();
    //close the light sensor;
    lightSensor.close();
    //go to the 
		
  }
  /**
    * go to origin by use the method from Lab3 Navigation class.
    */
  private void gotoOrigin() {
    travelTo(0.0,0.0);
    carStop();
    carSetSpeed(ROTATE_SPEED/2);
    //back to 0 degree orientation
    leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK,odo.getXYT()[2]+8), true);
    rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK,odo.getXYT()[2]+8), false);
    odo.update(0, 0, 8);
  }
  /**
    * if we need to find a place which could get 4 line ,we need to close to the origin which is intersection of x,y axis, 
    * so we could close to the x-axis first, and turn right to find a place close to y axis
    */
  private void closeToOrigin() {
    carSetSpeed(FORWARD_SPEED);
		  
    //drive to location where light sensor can get 4 lines
		  
    while (!meetBlackline()) {
      carMoveForward();
    }
    Sound.beep();
    carStop();
    initializeResults();
    //sees line, back up by 15 cms
    leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -16), true);
    rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, -16), false);
		  
    carSetSpeed(ROTATE_SPEED);
    //turns 90 degrees to the right
    carTurnRight(90);
    carSetSpeed(FORWARD_SPEED);
    //drives forward until line
    getData();
    while (!meetBlackline()) {
      carMoveForward();
    }
    Sound.beep();
    carStop();
    initializeResults();
    carSetSpeed(FORWARD_SPEED);
    //sees line, back up
    carMove(-(distanceLS + 2));
    carSetSpeed(ROTATE_SPEED);
    //turns 90 degrees to the left
    carTurnLeft(90);
  }
	
  /**
   * get the data read from light sensor	
   */
  private  void getData() {
    this.sampleProviderLight.fetchSample(lightValues,0);
    curColor = 100*(average(lightValues));
    this.results[j % size] = curColor;
    j++;		
  }
  /**
   * the filter method use the average value to reduce the error of unusual value
   * @param usvalues   the values read by light sensor
   * @return
   */
  private static float average(float[] usvalues) {
    float sum = 0;
    int i = 0;
    for (float values: usvalues) {
      sum += values;
      i++;
    }
    return sum/i;
  }
  /**
   * store the filtered value in a array,and compare the first and last one, if difference is too large ,the car meet the line
   * @return  if the car meet the black line
   */
  private  boolean meetBlackline() {
    getData();
    System.out.println();
    return (this.results[size - 1] -this. results[0]) < -5.0;
  }
  private  boolean meetBlacklineWithoutGet() {	
    System.out.println();
    return (this.results[size -1] - this.results[0]) < -5.0;
  }
  /**
   * the method from last lab
   * @param x   x coordianate
   * @param y   y coordianate
   */
  public void travelTo(double x, double y) {
    double currx,deltax;
    double curry,deltay;
    double currTheta;
    currx = odo.getXYT()[0];
    curry = odo.getXYT()[1];
    deltax = x - currx;
    deltay = y - curry;
    // Calculate the angle to turn around
    currTheta = (odo.getXYT()[2]) * Math.PI / 180;
    double mTheta = Math.atan2(deltax, deltay) - currTheta;
    double hypot = Math.hypot(deltax, deltay);
    // Turn to the correct angle towards the endpoint
    turnTo(mTheta);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, hypot), true);
    rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, hypot), false);
		// stop vehicle
    leftMotor.stop(true);
    rightMotor.stop();
  }
  /**
   * the method from last lab
   * @param theta  orientation
   */
  public void turnTo(double theta) {
    // ensures minimum angle for turning
    if (theta > Math.PI) {
      theta -= 2 * Math.PI;
    } else if (theta < -Math.PI) {
      theta += 2 * Math.PI;
    }
    // set Speed
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // rotate motors at set speed

    // if angle is negative, turn to the left
    if (theta < 0) {
      leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -(theta * 180) / Math.PI), true);
      rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, -(theta * 180) / Math.PI), false);
    } else {
    // angle is positive, turn to the right
    leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, (theta * 180) / Math.PI), true);
    rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, (theta * 180) / Math.PI), false);
  }
	}

  private void carMoveForward() {
    leftMotor.forward();
    rightMotor.forward();
  }
  private void carMove(double distance) { 
    leftMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), true);
    rightMotor.rotate(convertDistance(Lab5.WHEEL_RAD, distance), false);
  }
  private void carTurnRight(float angles) {
    leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK,angles), true);
    rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angles), false);
  }
  private void carTurnLeft(float angles) {
    leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK,angles), true);
    rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angles), false);
  }
	
  private void carStop() {
    leftMotor.stop(true);
    rightMotor.stop();
  }
  private void carRotate() {
    leftMotor.forward();
    rightMotor.backward();
  }
  private void carSetSpeed( int speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
  }
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }
  private void initializeResults() {
    for (int k = 0;k<size;k++) {
      this.results[k] = 100;
    }
  }
 
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}

