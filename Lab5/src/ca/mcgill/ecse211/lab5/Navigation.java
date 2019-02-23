package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation extends Thread{
  //private Odometer odometer;
 
  //private Odometer odometer;
  private static final int FORWARD_SPEED = 180;
  public static final String[] colors = {"red","green","blue","yellow"};
  private int targetColour;
  private int LLx;
  private int URx;
  private int LLy;
  private int URy;
  public static final double tileSize = 30.48;
  private static final EV3MediumRegulatedMotor sensorMotor =
	      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final int ROTATE_SPEED = 120;
  private static final double TILE_SIZE = 30.48;
  public static final Odometer odometer = Lab5.odometer;
  private static EV3LargeRegulatedMotor leftMotor ;
  private static EV3LargeRegulatedMotor rightMotor;
  private ColorClassification colorDetector;
  static final double leftRadius = 2.11;
  static final double rightRadius = 2.11;
  static final double track = 13.6;
  private static final int safeDistance= 10;
  private boolean travelling;
  private boolean planning;
  private boolean found;
   static double jobx;
   static double joby;
   public boolean search;
  private static boolean avoid=false;
  private static boolean back = true;
  
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public Navigation(EV3LargeRegulatedMotor leftmotor,EV3LargeRegulatedMotor rightmotor,int targetColour,int URx,int LLx,int URy,int LLy,ColorClassification colorDetector) throws OdometerExceptions {
	this.LLx = LLx;
	this.LLy = LLy;
	this.URx = URx;
	this.URy = URy;
	this.targetColour = targetColour;
	this.found = false;
	this.colorDetector = colorDetector;
	this.search = false;
	this.travelling = false;
	leftMotor = leftmotor;
    rightMotor = rightmotor;
    jobx = 0;
    joby = 0;
    
  }

  /**
   * This method is meant to drive the robot in a square of size 2x2 Tiles. It is to run in parallel
   * with the odometer and Odometer correcton classes allow testing their functionality.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param width
   */
  public void travelTo(double x,double y ) throws OdometerExceptions{
	jobx = x;
	joby = y;
	double theta, dX, dY, distance;
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

    //TO DO SEE POSITION TO NEXT POSITION 
   // angle = odometer.getXYT()[2];
    
    dX = x - odometer.getXYT()[0];
    dY = y - odometer.getXYT()[1];
    distance = Math.sqrt(dX*dX + dY*dY);
    theta = Math.toDegrees(Math.atan(dX/dY));
    
    if(dX < 0 && dY < 0) {
    	theta += 180;
    }
    if(dX > 0 && dY < 0) {
    	theta += 180;
    }
    if(dX < 0 && dY > 0) {
    	theta += 360;
    }
    turnTo(theta);
    this.travelling = true;
   
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), true);
    if (!planning) {
    	double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	while(distance1 > 2) {
    		distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	}
    } else {
    	while(true) {
    		if (avoid) {
    			if (search) {
    				if ((odometer.getXYT()[0]-jobx <= 2)||(odometer.getXYT()[1]-jobx <= 1)) {
    					break;
    				}
    			}
    		}else {
    			double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	    	if(distance1 > 2) {
    	    		distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	    	}else {
    	    		break;
    	    	}
    		}
    	}
    }
    
    travelling = false;
   
  }
  
  public  void avoid(double distance) throws OdometerExceptions{
	int color = 4;
    leftMotor.stop(true);
	rightMotor.stop();
	if(planning) {
	  color = colorDetector.findColor();
	  if (color == targetColour) {
	    Sound.beep();
	    this.found = true;
	    return;
	  }else {
	    Sound.twoBeeps();
	  }
	}
	travelling = false;
	if (distance < safeDistance) {
	  if (predictPath() == 1){
	    RightAvoid(color);
	  }
	  else if (predictPath() == 0){
	    leftAvoid(color);
	  }
	}
	if (!planning) {
		travelTo(jobx,joby);   
	}
	   
  }
  
  

  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  public void turnTo (double theta) {
	  double angle,smallestAngle;
	  angle = odometer.getXYT()[2];
	  
	    if((theta - angle) > 180) {
	    	smallestAngle = theta - angle - 360;
	    }
	    else if((theta - angle) < -180) {
	    	smallestAngle = theta - angle + 360;
	    }
	    else {
	    	smallestAngle = theta - angle;
	    }
	        // turn 90 degrees clockwise
	        leftMotor.setSpeed(ROTATE_SPEED);
	        rightMotor.setSpeed(ROTATE_SPEED);

	        leftMotor.rotate(convertAngle(leftRadius, track, smallestAngle), true);
	        rightMotor.rotate(-convertAngle(rightRadius, track, smallestAngle), false);
  }
   boolean  isNavigating(){
	  return this.travelling;
  }
  public int predictPath() {

		double currx = odometer.getXYT()[0];
		double curry = odometer.getXYT()[1];
		double currTheta = odometer.getXYT()[2];
		
		if (currTheta > 350 || currTheta <= 10) {//going up
			if (currx < (LLx+0.5)*tileSize -1) {
				return 1;
				//wallFollowRight();            // 1 represents right dodge and 0 represents left dodge
			} 
			else {
				return 0;
				//wallFollowLeft();
			}
		} 
		else if(currTheta >= 80 && currTheta < 100){//going right
			if (curry < (LLy+0.5)*tileSize) {
				return 0;
				//wallFollowLeft();
			} 
			else {
				return 1;
				//wallFollowRight();
			}
		}
		else if(currTheta > 170 && currTheta < 190){//going down
			if (currx < (URx-0.5)*tileSize) {
				return 0;
				//wallFollowLeft();
			} 
			else {
				return 1;
				//wallFollowRight();
			}
		}
		else if(currTheta > 260 && currTheta < 280){//going left
			if (curry <= (LLy+0.5)*tileSize ) {
				return 1;
				//wallFollowRight();
			} 
			else {
				return 0;
				//wallFollowLeft();
			}
		}
		else { 
			//wallFollowRight();
			return 1;
			}
	}
  public void RightAvoid(int color) {
	  avoid = true;
	  back =false;
	  leftMotor.setSpeed(ROTATE_SPEED/2);
	  rightMotor.setSpeed(ROTATE_SPEED/2);
	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
      rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
      leftMotor.setSpeed(FORWARD_SPEED/2);
      rightMotor.setSpeed(FORWARD_SPEED/2);
      leftMotor.rotate(convertDistance(leftRadius, 15), true);
      rightMotor.rotate(convertDistance(rightRadius, 15), false);
      leftMotor.setSpeed(ROTATE_SPEED/2);
	  rightMotor.setSpeed(ROTATE_SPEED/2);
	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
      rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
      leftMotor.setSpeed(FORWARD_SPEED/2);
      rightMotor.setSpeed(FORWARD_SPEED/2);
      leftMotor.rotate(convertDistance(leftRadius, 15), true);
      rightMotor.rotate(convertDistance(rightRadius, 15), false);
      if (color == -1) {
    	  leftMotor.setSpeed(ROTATE_SPEED/2);
    	  rightMotor.setSpeed(ROTATE_SPEED/2);
    	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
          rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
          color = colorDetector.findColor();
    	  if (color == targetColour) {
    	    Sound.beep();
    	    this.found = true;
    	    return;
    	  }else {
    	    Sound.twoBeeps();
    	  }
          if (search == true && ((odometer.getXYT()[0]-jobx <= 2)||(odometer.getXYT()[0]-jobx <= 2))) {
        	  leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              back = true;
          }
      }else {
    	  if (search == true && ((odometer.getXYT()[0]-jobx <= 2)||(odometer.getXYT()[0]-jobx <= 2))) {
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              back = true;
    	  }
      }
      avoid = false;
      
	  
	}
	
	
	public void leftAvoid(int color) {
		back  = false;
		avoid = true;
		  leftMotor.setSpeed(ROTATE_SPEED/2);
		  rightMotor.setSpeed(ROTATE_SPEED/2);
		  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	      rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	      leftMotor.setSpeed(FORWARD_SPEED/2);
	      rightMotor.setSpeed(FORWARD_SPEED/2);
	      leftMotor.rotate(convertDistance(leftRadius, 15), true);
	      rightMotor.rotate(convertDistance(rightRadius, 15), false);
	      leftMotor.setSpeed(ROTATE_SPEED/2);
		  rightMotor.setSpeed(ROTATE_SPEED/2);
		  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	      rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	      leftMotor.setSpeed(FORWARD_SPEED/2);
	      rightMotor.setSpeed(FORWARD_SPEED/2);
	      leftMotor.rotate(convertDistance(leftRadius, 15), true);
	      rightMotor.rotate(convertDistance(rightRadius, 15), false);
	      if (color == -1) {
	    	  leftMotor.setSpeed(ROTATE_SPEED/2);
	    	  rightMotor.setSpeed(ROTATE_SPEED/2);
	    	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	          rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	          color = colorDetector.findColor();
	    	  if (color == targetColour) {
	    	    Sound.beep();
	    	    this.found = true;
	    	    return;
	    	  }else {
	    	    Sound.twoBeeps();
	    	  }
	          if (search == true && ((odometer.getXYT()[0]-jobx <= 2)||(odometer.getXYT()[0]-jobx <= 2))) {
	        	  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              back = true;
	          }
	      }else {
	    	  if (search == true && ((odometer.getXYT()[0]-jobx <= 2)||(odometer.getXYT()[0]-jobx <= 2))) {
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              back = true;
	    	  }
		avoid = false;
	}
	}


  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  public void verticalPlanning(int Rx,int Ry) throws OdometerExceptions {
	  boolean firstSide = true;
	  this.planning = true;
	  for(int i = 1;i <= Ry;i++) {
		    if(firstSide) {
		      travelTo(this.LLx*tileSize,(this.LLy+1)*tileSize);
		    }else {
		    	travelTo(this.URx*tileSize,(this.LLy+1)*tileSize);
		    }
		  if(firstSide) {
			  turnTo(90);
		  }else {
			  turnTo(270);
		  }
		  if (SensorPoller.usData[0]<(Rx*tileSize)) {
			  back = true;
			  this.search = true;
			  if(firstSide) {
				  travelTo(this.URx*tileSize,(this.LLy+1)*tileSize);
			  }else {
				  travelTo(this.LLx*tileSize,(this.LLy+1)*tileSize);
			  }
			 firstSide = !firstSide;
		  }
		  if (!back) {
			  if (firstSide) {
				  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
			  }else {
				  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
			  }
		  }
		 this.search = false; 
		  if (this.found) {
			  break;
		  }
	  }
	  planning = false;
	  travelTo(URx*tileSize,URy*tileSize);
  }
  public void horizontalPlanning(int Rx,int Ry) throws OdometerExceptions {
	  boolean firstSide = true;
	  this.planning = true;
	  for(int i = 1;i <= Rx;i++) {
		    if(firstSide) {
		      travelTo((this.LLx+1)*tileSize,this.LLy*tileSize);
		    }else {
		    	travelTo((this.LLx+1)*tileSize,this.URy*tileSize);
		    }
		  if(firstSide) {
			  turnTo(0);
		  }else {
			  turnTo(180);
		  }
		  if (SensorPoller.usData[0]<(Rx*tileSize)) {
			  back = true;
			  this.search = true;
			  if(firstSide) {
				  travelTo((this.LLx+1)*tileSize,this.URy*tileSize);
			  }else {
				  travelTo((this.LLx+1)*tileSize,this.LLy*tileSize);
			  }
			  firstSide = !firstSide;
		  }
		  if (!back) {
			  if (firstSide) {
				  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
			  }else {
				  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
			  }
		  }
		  this.search = false;
		  if (this.found) {
			  break;
		  }
	  }
	  planning = false;
	  travelTo(URx*tileSize,URy*tileSize);
  }
  public void run() {
	  try {
		travelTo(LLx * tileSize,LLy*tileSize);
	} catch (OdometerExceptions e1) {
		// TODO Auto-generated catch block
		e1.printStackTrace();
	}
	  int Rx = this.URx - this.LLx;
	  int Ry = this.URy - this.LLy;
	  if (Rx <= Ry) {
		  try {
		  verticalPlanning(Rx,Ry);
		  }catch (OdometerExceptions e) {
				
				e.printStackTrace();
			}
	  }
	  else {
		  try {
		  horizontalPlanning(Rx,Ry);
		  }catch (OdometerExceptions e) {
				
				e.printStackTrace();
			}
	  }
  }
 
}
