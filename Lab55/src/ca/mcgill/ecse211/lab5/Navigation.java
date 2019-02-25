package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation{
  //private Odometer odometer;
 
  //private Odometer odometer;
  private static final int FORWARD_SPEED = 150;
  public static final String[] colors = {"red","green","blue","yellow"};
  private int targetColour;
  private char planningType;
  private int LLx;
  private boolean firstSide;
  private int URx;
  private int LLy;
  private int URy;
  public boolean obstacle;
  public static final double tileSize = 30.48;
  private static final int ROTATE_SPEED = 100;
  private static final double TILE_SIZE = 30.48;
  public static Odometer odometer;
  private static EV3LargeRegulatedMotor leftMotor ;
  private static EV3LargeRegulatedMotor rightMotor;
  private ColorClassification colorDetector;
  static final double leftRadius = 2.10;
  static final double rightRadius = 2.10;
  static final double track = 13.6;
  private static final int safeDistance= 8;
  private SampleProvider gyAngles;
  private SampleProvider us;
  private float[] usValues;
  private float[] angles; 
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
  public Navigation(EV3LargeRegulatedMotor leftmotor,EV3LargeRegulatedMotor rightmotor,int targetColour,int URx,int LLx,int URy,int LLy,ColorClassification colorDetector,SampleProvider gyAngle,float[] angles,SampleProvider us, float[] usValues) throws OdometerExceptions {
	this.LLx = LLx;
	this.LLy = LLy;
	this.URx = URx;
	this.URy = URy;
	this.us = us;
	this.usValues = usValues;
	this.angles = angles;
	this.gyAngles = gyAngle;
	this.planning = false;
	this.targetColour = targetColour;
	this.obstacle = false;
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
   * with the odometer and Odometer correction classes allow testing their functionality.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param width
   */
  public void travelTo(double x,double y ) throws OdometerExceptions{
	  if (!planning) {
	    	double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
	    	if(distance1 < 3) {
	    		return;
	    	}
	    } 
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
    System.out.println("aim at "+ x + " " + y);
    //TO DO SEE POSITION TO NEXT POSITION 
   // angle = odometer.getXYT()[2];
    
    dX = x - odometer.getXYT()[0];
    dY = y - odometer.getXYT()[1];
    double oldx = odometer.getXYT()[0];
    double oldy = odometer.getXYT()[1];
    System.out.println(oldx+" "+oldy);
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
    double correctAngle = gyroFetch();
    if (Math.abs(correctAngle - theta)>2) {
    	angleCorrection();
    	travelTo(x,y);
    	return;
    }
    this.travelling = true;
   if(search) {
    leftMotor.setSpeed((int)(FORWARD_SPEED*0.8));
    rightMotor.setSpeed((int)(FORWARD_SPEED*0.8));
   }else {
	   leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED); 
   }
   leftMotor.forward();
   rightMotor.forward();
   // leftMotor.rotate(convertDistance(leftRadius, distance), true);
   // rightMotor.rotate(convertDistance(rightRadius, distance), true);
    while(true) {
    	//if(obstacle) {
    	directionCorrection(correctAngle,FORWARD_SPEED);
    	if(usFetch() < 10) {
    		avoid(usFetch());
    		back = true;
    		if(planning) {
    		if(found) {
    			planning = false;
    			travelTo(URx*tileSize,URy*tileSize);
    			System.exit(0);
    		}
    		}
    		if (!planning) {
    			travelTo(jobx,joby);   
    		}
    		break;
    	}
    if (!planning) {
    	double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	if(distance1 < 3) {
    		break;
    	}
    	if (losePath(oldx,oldy)) {
    		travelTo(jobx,joby);
    	}
    } else {
    				
    			
    			double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	    	if(distance1 > 0.5) {
    	    		distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	    	}else {
    	    		break;
    	    	}
    	    	if (losePath(oldx,oldy)) {
    	    		travelTo(jobx,joby);
    	    	}
    		}

    }
    carStop();
    travelling = false;
   
  }
  
  public  void avoid(double distance) throws OdometerExceptions{
	int color = 4;
    leftMotor.stop(true);
	rightMotor.stop();
	if(planning) {
		leftMotor.setSpeed(FORWARD_SPEED/3);
	    rightMotor.setSpeed(FORWARD_SPEED/3);
	    leftMotor.rotate(convertDistance(leftRadius, 7), true);
	    rightMotor.rotate(convertDistance(rightRadius, 7), false);
	    carStop();
	  color = colorDetector.findColor();
	  if (color == targetColour) {
	    Sound.beep();
	    this.found = true;
	    return;
	  }else {
	    Sound.twoBeeps();
	  }
	  leftMotor.setSpeed(FORWARD_SPEED/3);
	  rightMotor.setSpeed(FORWARD_SPEED/3);
	  leftMotor.rotate(-convertDistance(leftRadius, 7), true);
	  rightMotor.rotate(-convertDistance(rightRadius, 7), false);
	  carStop();
	}
	travelling = false;
	  if (predictPath() == 1){
	    RightAvoid(color);
	  }
	  else if (predictPath() == 0){
	    leftAvoid(color);
	  }
	  if (found) {
			return;
		}
	
	this.obstacle = false;
	   
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
	 // angle = odometer.getXYT()[2];
	  angle = gyroFetch();
	    if((theta - angle) > 180) {
	    	//smallestAngle = theta - angle - 360;
	    	turnLeft(360-(theta-angle));
	    }
	    else if((theta - angle) < -180) {
	    	//smallestAngle = theta - angle + 360;
	    	turnRight(360 -(angle-theta));
	    }else if ((theta - angle) > -180 && (theta - angle) < 0) {
	    	turnLeft((angle-theta));
	    }
	    else {
	    	smallestAngle = theta - angle;
	    	turnRight(smallestAngle);
	    }
	        // turn 90 degrees clockwise
	      //  leftMotor.setSpeed(ROTATE_SPEED);
	      //  rightMotor.setSpeed(ROTATE_SPEED);

	       // leftMotor.rotate(convertAngle(leftRadius, track, smallestAngle), true);
	       // rightMotor.rotate(-convertAngle(rightRadius, track, smallestAngle), false);
  }
   boolean  isNavigating(){
	  return this.travelling;
  }
  public int predictPath() {

		double currx = odometer.getXYT()[0];
		double curry = odometer.getXYT()[1];
		double currTheta = odometer.getXYT()[2];
		
		if (currTheta > 340 || currTheta <= 20) {//going up
			if (currx < (LLx+0.5)*tileSize) {
				return 1;
				//wallFollowRight();            // 1 represents right dodge and 0 represents left dodge
			} 
			else if (currx > (URx-0.5)*tileSize){
				return 0;
				//wallFollowLeft();
			}
		} 
		else if(currTheta >= 70 && currTheta < 110){//going right
			if (curry < (LLy+0.5)*tileSize) {
				return 0;
				//wallFollowLeft();
			} 
			else if (curry > (URy-0.5)*tileSize) {
				return 1;
				//wallFollowRight();
			}
		}
		else if(currTheta > 160 && currTheta < 200){//going down
			if (currx < (LLx+0.5)*tileSize) {
				return 0;
				//wallFollowLeft();
			} 
			else if (currx > (URx-0.5)*tileSize) {
				return 1;
				//wallFollowRight();
			}
		}
		else if(currTheta > 250 && currTheta < 290){//going left
			if (curry <= (LLy+0.5)*tileSize ) {
				return 1;
				//wallFollowRight();
			} 
			else if (curry > (URy-0.5)*tileSize) {
				System.out.println("8");
				return 0;
				//wallFollowLeft();
			}
		}
			//wallFollowRight();
			//return 1;
			if (this.planningType == 'V') {
				if (this.firstSide) {
					return 0;
				}else {
					return 1;
				}
			}else if (this.planningType == 'H'){
				if(this.firstSide) {
					return 1;
			}else {
				return 0;
			}
				
			}else {
				return 0;
			}
  
	
	}
  public void RightAvoid(int color) {
	  avoid = true;
	  back =false;
	  carStop();
	 /* leftMotor.setSpeed(ROTATE_SPEED/2);
	  rightMotor.setSpeed(ROTATE_SPEED/2);
	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
      rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
      carStop();*/
	  turnRight(90);
      /*leftMotor.setSpeed(FORWARD_SPEED/2);
      rightMotor.setSpeed(FORWARD_SPEED/2);
      leftMotor.rotate(convertDistance(leftRadius, 1), true);
      rightMotor.rotate(convertDistance(rightRadius, 15), false);
      carStop();*/
	  goStraightLine(15,FORWARD_SPEED/2);
     /* leftMotor.setSpeed(ROTATE_SPEED/2);
	  rightMotor.setSpeed(ROTATE_SPEED/2);
	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
      rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
      carStop();*/
      turnLeft(90);
      /*leftMotor.setSpeed(FORWARD_SPEED/2);
      rightMotor.setSpeed(FORWARD_SPEED/2);
      leftMotor.rotate(convertDistance(leftRadius, 17), true);
      rightMotor.rotate(convertDistance(rightRadius, 17), false);
      carStop();*/
      goStraightLine(17,FORWARD_SPEED/2);
      if (color == -1) {
    	  /*leftMotor.setSpeed(ROTATE_SPEED/2);
    	  rightMotor.setSpeed(ROTATE_SPEED/2);
    	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
          rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
          carStop();*/
    	  turnLeft(90);
          leftMotor.setSpeed(FORWARD_SPEED/3);
  	      rightMotor.setSpeed(FORWARD_SPEED/3);
  	      leftMotor.rotate(convertDistance(leftRadius, 7), true);
  	      rightMotor.rotate(convertDistance(rightRadius, 7), false);
  	      carStop();
          color = colorDetector.findColor();
    	  if (color == targetColour) {
    	    Sound.beep();
    	    this.found = true;
    	    return;
    	  }else {
    	    Sound.twoBeeps();
    	  }
    	  leftMotor.setSpeed(FORWARD_SPEED/3);
  	      rightMotor.setSpeed(FORWARD_SPEED/3);
  	      leftMotor.rotate(-convertDistance(leftRadius, 7), true);
  	      rightMotor.rotate(-convertDistance(rightRadius, 7), false);
  	      carStop();
          if (search && !endOfSearch()) {
        	  /*leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              carStop();*/
        	  turnRight(90);
             /* leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
        	  goStraightLine(15,FORWARD_SPEED/2);
              carStop();
             /* leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
              carStop();*/
              turnLeft(90);
              /*leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
              goStraightLine(15,FORWARD_SPEED/2);
              carStop();
              
              /*leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              carStop();*/
              turnRight(90);
              back = true;
          }
      }else {
    	  if (search && !endOfSearch()) {
              /*leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();*/
    		  goStraightLine(15,FORWARD_SPEED/2);
             /* leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
              carStop();*/
              turnLeft(90);
              /*leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();*/
              goStraightLine(15,FORWARD_SPEED/2);
              /*leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              carStop();*/
              carStop();
              back = true;
    	  }
      }
      avoid = false;
      
	  
	}
	
	
	public void leftAvoid (int color) {
		back  = false;
		avoid = true;
		carStop();
		 /* leftMotor.setSpeed(ROTATE_SPEED/2);
		  rightMotor.setSpeed(ROTATE_SPEED/2);
		  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	      rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	      carStop();*/
		  turnLeft(90);
	      leftMotor.setSpeed(FORWARD_SPEED/2);
	      rightMotor.setSpeed(FORWARD_SPEED/2);
	      leftMotor.rotate(convertDistance(leftRadius, 17), true);
	      rightMotor.rotate(convertDistance(rightRadius, 17), false);
	      carStop();
	      gyroFetch();
		  //goStraightLine(15,FORWARD_SPEED/2);
	      
	      /*leftMotor.setSpeed(ROTATE_SPEED/2);
		  rightMotor.setSpeed(ROTATE_SPEED/2);
		  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	      rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	      carStop();*/
	      turnRight(90);
	      leftMotor.setSpeed(FORWARD_SPEED/2);
	      rightMotor.setSpeed(FORWARD_SPEED/2);
	      leftMotor.rotate(convertDistance(leftRadius, 17), true);
	      rightMotor.rotate(convertDistance(rightRadius, 17), false);
	      carStop();
	      gyroFetch();
	     // goStraightLine(17,FORWARD_SPEED/2);
	      carStop();
	      if (color == -1) {
	    	 /* leftMotor.setSpeed(ROTATE_SPEED/2);
	    	  rightMotor.setSpeed(ROTATE_SPEED/2);
	    	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	          rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);   	  
	          carStop();
	          */
	    	  turnRight(90);
	          leftMotor.setSpeed(FORWARD_SPEED/3);
	  	      rightMotor.setSpeed(FORWARD_SPEED/3);
	  	      leftMotor.rotate(convertDistance(leftRadius, 7), true);
	  	      rightMotor.rotate(convertDistance(rightRadius, 7), false);
	  	      carStop();
	  	      gyroFetch();
	          color = colorDetector.findColor();
	    	  if (color == targetColour) {
	    	    Sound.beep();
	    	    this.found = true;
	    	    return;
	    	  }else {
	    	    Sound.twoBeeps();
	    	  }
	    	  leftMotor.setSpeed(FORWARD_SPEED/3);
	  	      rightMotor.setSpeed(FORWARD_SPEED/3);
	  	      leftMotor.rotate(convertDistance(leftRadius, 7), true);
	  	      rightMotor.rotate(convertDistance(rightRadius, 7), false);
	  	      gyroFetch();
	  	      carStop();
	          if (search && !endOfSearch()) {
	        	  /*leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();*/
	        	  turnLeft(90);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 17), true);
	              rightMotor.rotate(convertDistance(rightRadius, 17), false);
	        	  //goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              gyroFetch();
	             /* leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();*/
	              turnRight(90);
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 17), true);
	              rightMotor.rotate(convertDistance(rightRadius, 17), false);
	             // goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              gyroFetch();
	              /*leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();*/
	              turnLeft(90);
	              back = true;
	          }
	      }else {
	    	  if (search && !endOfSearch() ) {
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	    		 // goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              gyroFetch();
	             /* leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();*/
	              turnRight(90);
	               leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              //goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              /*leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();*/
	              turnLeft(90);
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
	  this.planningType = 'V';
	  this.firstSide = true;
	  this.planning = true;
	  for(int i = 0;i <= Ry;i++) {
		  
		  if(this.firstSide) {
			  turnTo(90);
		  }else {
			  turnTo(270);
		  }
		  System.out.println(usFetch());
		  if (usFetch()<(Rx*tileSize)) {
			  back = true;
			  this.search = true;
			  if(this.firstSide) {
				  for(int j = 1;j<=Rx;j++) {
				  travelTo((this.LLx+j)*tileSize,(this.LLy+i)*tileSize);
				  }
			  }else {
				  for(int j = 1;j<=Rx;j++) {
				  travelTo((this.URx-j)*tileSize,(this.LLy+i)*tileSize);
				  }
			  }
			 this.firstSide = !this.firstSide;
		  }
		  if (!back) {
			  if (this.firstSide) {
				 /* leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();*/
				  turnLeft(90);
	              /*leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
				  goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	             /* leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();*/
	              turnLeft(90);
	              /*leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
	              goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnRight(90);
			  }else {
				  /*leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();*/
				  turnRight(90);
	              /*leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
				  goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	             /* leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();*/
	              turnRight(90);
	             /* leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
	              goStraightLine(15,FORWARD_SPEED/2);
	              turnLeft(90);
			  }
		  }
		 this.search = false; 
		  if (this.found) {
			  break;
		  }
		  if(i < Ry) {
			    if(this.firstSide) {
			      travelTo(this.LLx*tileSize,(this.LLy+i+1)*tileSize);
			    }else {
			    	travelTo(this.URx*tileSize,(this.LLy+i+1)*tileSize);
			    }
			  }
	  }
	  planning = false;
	  travelTo(URx*tileSize,URy*tileSize);
  }
  public void horizontalPlanning(int Rx,int Ry) throws OdometerExceptions {
	  this.planningType = 'H';
	   this.firstSide = true;
	  this.planning = true;
	  for(int i = 0;i <= Rx;i++) {
		  if(this.firstSide) {
			  turnTo(0);
		  }else {
			  turnTo(180);
		  }
		  if ((usFetch())<(Rx*tileSize)) {
			  back = true;
			  this.search = true;
			  if(this.firstSide) {
				  for(int j = 1;j<=Ry;j++) {
				  travelTo((this.LLx+i)*tileSize,(this.LLy+j)*tileSize);
				  }
			  }else {
				  for(int j = 1;j<=Ry;j++) {
				  travelTo((this.LLx+i)*tileSize,(this.URy-j)*tileSize);
			  }
			  }
			  this.firstSide = !this.firstSide;
		  }
		  if (!back) {
			  if (this.firstSide) {
				  /*leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();*/
				  turnRight(90);
	             /* leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
				  goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              /*leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();*/
	              turnRight(90);
	              /*leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
	              goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnLeft(90);
			  }else {
				  /*leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();*/
				  turnLeft(90);
	             /* leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
				  goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	             /* leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();*/
	              turnLeft(90);
	             /* leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);*/
	              goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnRight(90);
			  }
		  }
		  this.search = false;
		  if (this.found) {
			  break;
		  }
		  if(i < Rx) {
			    if(this.firstSide) {
			      travelTo((this.LLx+i+1)*tileSize,this.LLy*tileSize);
			    }else {
			    	travelTo((this.LLx+i+1)*tileSize,this.URy*tileSize);
			    }
			  }
	  
	  }
	  planning = false;
	  travelTo(URx*tileSize,URy*tileSize);
	  
  }
  public boolean closeSearchEnd() {
	 return (Math.abs(odometer.getXYT()[0]-jobx) <= 3)&&(Math.abs(odometer.getXYT()[1]-joby) <= 15) || (Math.abs(odometer.getXYT()[0]-jobx) <= 18)&&(Math.abs(odometer.getXYT()[1]-joby) <= 3);
  }
  public void work() {
	 
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
  private float usFetch() {
	  us.fetchSample(usValues, 0);
	  if ((usValues[0]*100)>200) {
		  return 200;
	  }else {
		  return usValues[0]*100;
	  }
  }
  private double gyroFetch() {
	  this.gyAngles.fetchSample(angles, 0);
	  angleCorrection();
	  return odometer.getXYT()[2];
  }
  private void angleCorrection() {
	  gyAngles.fetchSample(angles, 0);
	  if (angles[0] >= 0) {
		  odometer.setXYT(odometer.getXYT()[0], odometer.getXYT()[1],angles[0]);
	  }else {
		  odometer.setXYT(odometer.getXYT()[0], odometer.getXYT()[1], 360+angles[0]);
	  }
  }
  private void directionCorrection(double degree,int speed) {
		if(gyroFetch() - degree  >= 1) {
			leftMotor.setSpeed(speed);
	        rightMotor.setSpeed(speed+8);
	        leftMotor.forward();
  		rightMotor.forward();
	this.gyAngles.fetchSample(angles, 0);
  }else if(gyroFetch() - degree <=-2) {
  		leftMotor.setSpeed(speed+8);
	        rightMotor.setSpeed(speed);
	        leftMotor.forward();
  		rightMotor.forward();
	this.gyAngles.fetchSample(angles, 0);
  	}
  }
  private void goStraightLine(double distance,int speed) {
	  carStop();
	  double angle = gyroFetch();
	  double p1 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*leftRadius*Math.PI/360;
	  double p2 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*leftRadius*Math.PI/360;
	 double distance1 = p2 - p1;
	//double  oldx = odometer.getXYT()[0]; 
	//double  curx = odometer.getXYT()[0]; 
	//double  oldy = odometer.getXYT()[1]; 
	//double  cury = odometer.getXYT()[1];
	int speed1;
	//double distance1 = Math.hypot(curx - oldx,cury - oldy);
	while(distance1 < distance) {
		speed1 =(int) (speed - 15/((distance - distance1)+1));
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
		leftMotor.forward();
		rightMotor.forward();
		for(int i = 0;i<3;i++);
		directionCorrection(angle,speed1);
		p2 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*leftRadius*Math.PI/360;
		distance1 = p2 - p1;
	}
	  carStop();
  }

  private void turnRight(double degree) {
	  if(degree <= 1) {
		  return;
	  }
	  double minAngle = 0;
	    int speed;
		 double angle = gyroFetch();
		 double angle1 = gyroFetch();
		 while((Math.abs(angle1 - angle - degree)>=1) && (Math.abs((angle - angle1) - (360-degree))>=1)){
			 minAngle = Math.min((Math.abs(angle1 - angle - degree)), Math.abs((angle - angle1) - (360-degree)));
			 speed = (int)(80 - 30/(minAngle+1));
			 leftMotor.setSpeed(speed);
		     rightMotor.setSpeed(speed);
		     leftMotor.forward();
		     rightMotor.backward();
		     angle1 = gyroFetch();
		 }
			leftMotor.stop(true);
			rightMotor.stop();
		}
		private void turnLeft(double degree) {
			if (degree <= 1.5) {
				return;
			}
			int speed;
			double minAngle = 0;
			 double angle = gyroFetch();
			 double angle1 = gyroFetch();
			 while((Math.abs(angle - angle1 - degree)>=1) && (Math.abs((angle1 - angle) - (360-degree))>=1)){
				 minAngle = Math.min((Math.abs(angle - angle1 - degree)), Math.abs((angle1 - angle) - (360-degree)));
				 speed = (int)(80 - 30/(minAngle+1));
				 leftMotor.setSpeed(speed);
			     rightMotor.setSpeed(speed);
			     leftMotor.backward();
			     rightMotor.forward();
			     angle1 = gyroFetch();
			 }
				leftMotor.stop(true);
				rightMotor.stop();
			}
 private boolean endOfSearch() {
	 if (planningType == 'H') {
		 if(firstSide) {
			 return (Math.abs(joby - URy*tileSize) <= 1);
		 }else {
			 return (Math.abs(joby - LLy*tileSize) <= 1);
		 }
	 }else {
		 if(firstSide) {
			 return (Math.abs(jobx - URx*tileSize) <= 1);
		 }else {
			 return (Math.abs(jobx - LLx*tileSize) <= 1);
		 }
	 }
 }
 private boolean losePath(double x,double y) {
	 double curx = odometer.getXYT()[0];
	 double cury = odometer.getXYT()[1];
	 System.out.println(curx+" "+cury);
	 boolean x1=false,y1=false;
	 if (jobx < x && curx < x) {
		 x1 = (jobx - curx > 8);
	 }else if (jobx > x && curx > x) {
		 x1 = ((curx - jobx) >8);
	 }else if ((jobx > x && curx < x)||(jobx < x && curx > x)) {
		 x1 = (Math.abs(curx - x) > 4);
	 }
	 if (joby < y && cury < y) {
		 y1 = (joby - cury > 8);
	 }else if (joby > y && cury > y) {
		 y1 = ((cury - joby) >8);
	 }else if ((joby > y && cury < y)||(joby < y && cury > y)) {
		 y1 = Math.abs(cury - y) > 4;
	 }
	 if (x1||y1) {
		 System.out.println(x1);
		 System.out.println(y1);
		 System.out.println("lose");
	 }
	 return x1||y1;
 }
  private void carStop() {
	    leftMotor.stop(true);
	    rightMotor.stop();
	  }
}
