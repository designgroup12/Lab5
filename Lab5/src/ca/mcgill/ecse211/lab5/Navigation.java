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
	    	System.out.println("stuck");
	    	double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
	    	if(distance1 < 3) {
	    		return;
	    	}
	    } else {
	    		if (search && !back) {
	    				if (closeSearchEnd()) {
	    					System.out.println("close end");
	    					back = true;
	    					return;
	    			}
	    		}
	    }
	  System.out.println("travel to" + x + y);
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
   if(search) {
    leftMotor.setSpeed((int)(FORWARD_SPEED*0.6));
    rightMotor.setSpeed((int)(FORWARD_SPEED*0.6));
   }else {
	   leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED); 
   }
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), true);
    while(true) {
    	if(obstacle) {
    		avoid(SensorPoller.realDistance);
    		if(planning) {
    		if(found) {
    			break;
    		}
    		}
    		if (!planning || search) {
    			travelTo(jobx,joby);   
    		}
    		break;
    	}
    if (!planning) {
    	System.out.println("stuck");
    	double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	if(distance1 < 3) {
    		break;
    	}
    } else {
    				if ( search && !back && closeSearchEnd()) {
    					System.out.println("close end");
    					break;
    				
    			
    		}else {
    			double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	    	if(distance1 > 3) {
    	    		distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	    	}else {
    	    		break;
    	    	}
    		}
    }
    }
    System.out.println("travel end");
    travelling = false;
   
  }
  
  public  void avoid(double distance) throws OdometerExceptions{
	System.out.println("avoid");
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
	  if (found) {
			return;
		}
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
	  System.out.println("prediction");

		double currx = odometer.getXYT()[0];
		double curry = odometer.getXYT()[1];
		double currTheta = odometer.getXYT()[2];
		
		if (currTheta > 340 || currTheta <= 20) {//going up
			if (currx < (LLx+0.5)*tileSize) {
				System.out.println("1");
				return 1;
				//wallFollowRight();            // 1 represents right dodge and 0 represents left dodge
			} 
			else if (currx > (URx-0.5)*tileSize){
				System.out.println("2");
				return 0;
				//wallFollowLeft();
			}
		} 
		else if(currTheta >= 70 && currTheta < 110){//going right
			if (curry < (LLy+0.5)*tileSize) {
				System.out.println("3");
				return 0;
				//wallFollowLeft();
			} 
			else if (curry > (URy-0.5)*tileSize) {
				System.out.println("4");
				return 1;
				//wallFollowRight();
			}
		}
		else if(currTheta > 160 && currTheta < 200){//going down
			if (currx < (LLx+0.5)*tileSize) {
				System.out.println("5");
				return 0;
				//wallFollowLeft();
			} 
			else if (currx > (URx-0.5)*tileSize) {
				System.out.println("6");
				return 1;
				//wallFollowRight();
			}
		}
		else if(currTheta > 250 && currTheta < 290){//going left
			if (curry <= (LLy+0.5)*tileSize ) {
				System.out.println("7");
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
					System.out.println("9");
					return 0;
				}else {
					System.out.println("10");
					return 1;
				}
			}else if (this.planningType == 'H'){
				if(this.firstSide) {
					System.out.println("11");
					return 1;
			}else {
				System.out.println("12");
				return 0;
			}
				
			}else {
				return 0;
			}
  
	
	}
  public void RightAvoid(int color) {
	  System.out.println("R");
	  avoid = true;
	  back =false;
	  carStop();
	  leftMotor.setSpeed(ROTATE_SPEED/2);
	  rightMotor.setSpeed(ROTATE_SPEED/2);
	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
      rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
      carStop();
      leftMotor.setSpeed(FORWARD_SPEED/2);
      rightMotor.setSpeed(FORWARD_SPEED/2);
      leftMotor.rotate(convertDistance(leftRadius, 15), true);
      rightMotor.rotate(convertDistance(rightRadius, 15), false);
      carStop();
      leftMotor.setSpeed(ROTATE_SPEED/2);
	  rightMotor.setSpeed(ROTATE_SPEED/2);
	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
      rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
      carStop();
      leftMotor.setSpeed(FORWARD_SPEED/2);
      rightMotor.setSpeed(FORWARD_SPEED/2);
      leftMotor.rotate(convertDistance(leftRadius, 15), true);
      rightMotor.rotate(convertDistance(rightRadius, 15), false);
      carStop();
      if (color == -1) {
    	  leftMotor.setSpeed(ROTATE_SPEED/2);
    	  rightMotor.setSpeed(ROTATE_SPEED/2);
    	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
          rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
          carStop();
          color = colorDetector.findColor();
    	  if (color == targetColour) {
    	    Sound.beep();
    	    this.found = true;
    	    return;
    	  }else {
    	    Sound.twoBeeps();
    	  }
          if (search == true && ((Math.abs(odometer.getXYT()[0]-(jobx)) > 3)&&(Math.abs(odometer.getXYT()[1]-(joby)) > 3))) {
        	  System.out.println("b");
        	  leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              carStop();
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
              carStop();
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              carStop();
              back = true;
          }
      }else {
    	  if (search == true && ((Math.abs(odometer.getXYT()[0]-jobx) > 3)&&(Math.abs(odometer.getXYT()[1]-joby) > 3))) {
    		  System.out.println("b");
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
              carStop();
              leftMotor.setSpeed(FORWARD_SPEED/2);
              rightMotor.setSpeed(FORWARD_SPEED/2);
              leftMotor.rotate(convertDistance(leftRadius, 15), true);
              rightMotor.rotate(convertDistance(rightRadius, 15), false);
              carStop();
              leftMotor.setSpeed(ROTATE_SPEED/2);
        	  rightMotor.setSpeed(ROTATE_SPEED/2);
        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
              carStop();
              back = true;
    	  }
      }
      avoid = false;
      
	  
	}
	
	
	public void leftAvoid(int color) {
		System.out.println("L");
		back  = false;
		avoid = true;
		carStop();
		  leftMotor.setSpeed(ROTATE_SPEED/2);
		  rightMotor.setSpeed(ROTATE_SPEED/2);
		  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	      rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	      carStop();
	      leftMotor.setSpeed(FORWARD_SPEED/2);
	      rightMotor.setSpeed(FORWARD_SPEED/2);
	      leftMotor.rotate(convertDistance(leftRadius, 15), true);
	      rightMotor.rotate(convertDistance(rightRadius, 15), false);
	      carStop();
	      leftMotor.setSpeed(ROTATE_SPEED/2);
		  rightMotor.setSpeed(ROTATE_SPEED/2);
		  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	      rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	      carStop();
	      leftMotor.setSpeed(FORWARD_SPEED/2);
	      rightMotor.setSpeed(FORWARD_SPEED/2);
	      leftMotor.rotate(convertDistance(leftRadius, 15), true);
	      rightMotor.rotate(convertDistance(rightRadius, 15), false);
	      carStop();
	      if (color == -1) {
	    	  leftMotor.setSpeed(ROTATE_SPEED/2);
	    	  rightMotor.setSpeed(ROTATE_SPEED/2);
	    	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	          rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	          carStop();
	          color = colorDetector.findColor();
	    	  if (color == targetColour) {
	    	    Sound.beep();
	    	    this.found = true;
	    	    return;
	    	  }else {
	    	    Sound.twoBeeps();
	    	  }
	          if (search == true && ((Math.abs(odometer.getXYT()[0]-(jobx)) > 3)&&(Math.abs(odometer.getXYT()[1]-(joby)) > 3))) {
	        	  System.out.println("b");
	        	  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();
	              back = true;
	          }
	      }else {
	    	  if (search == true && ((Math.abs(odometer.getXYT()[0]-(jobx)) > 3)&&(Math.abs(odometer.getXYT()[1]-(joby)) > 3))) {
	    		  System.out.println("b");
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();
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
	  System.out.println("V");
	  this.planningType = 'V';
	  this.firstSide = true;
	  this.planning = true;
	  for(int i = 1;i <= Ry;i++) {
		  
		  if(this.firstSide) {
			  turnTo(90);
		  }else {
			  turnTo(270);
		  }
		  System.out.println(SensorPoller.realDistance);
		  if ((SensorPoller.realDistance)<(Rx*tileSize)) {
			  back = true;
			  this.search = true;
			  if(this.firstSide) {
				  travelTo(this.URx*tileSize,(this.LLy+i)*tileSize);
			  }else {
				  travelTo(this.LLx*tileSize,(this.LLy+i)*tileSize);
			  }
			 this.firstSide = !this.firstSide;
		  }
		  if (!back) {
			  System.out.println("B");
			  if (this.firstSide) {
				  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
			  }else {
				  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();
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
		  if(i < Ry) {
			    if(this.firstSide) {
			      travelTo(this.LLx*tileSize,(this.LLy+i+1)*tileSize);
			    }else {
			    	travelTo(this.URx*tileSize,(this.LLy+i+1)*tileSize);
			    }
			  }
	  }
	  planning = false;
	  System.out.println("end");
	  travelTo(URx*tileSize,URy*tileSize);
  }
  public void horizontalPlanning(int Rx,int Ry) throws OdometerExceptions {
	  System.out.println("H");
	  this.planningType = 'H';
	   this.firstSide = true;
	  this.planning = true;
	  for(int i = 0;i <= Rx;i++) {
		  if(this.firstSide) {
			  turnTo(0);
		  }else {
			  turnTo(180);
		  }
		  System.out.println(SensorPoller.realDistance);
		  if (((SensorPoller.realDistance))<(Rx*tileSize)) {
			  back = true;
			  this.search = true;
			  if(this.firstSide) {
				  travelTo((this.LLx+i)*tileSize,this.URy*tileSize);
			  }else {
				  travelTo((this.LLx+i)*tileSize,this.LLy*tileSize);
			  }
			  this.firstSide = !this.firstSide;
		  }
		  if (!back) {
			  System.out.println("B");
			  if (this.firstSide) {
				  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(-convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
			  }else {
				  leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
	              leftMotor.setSpeed(ROTATE_SPEED/2);
	        	  rightMotor.setSpeed(ROTATE_SPEED/2);
	        	  leftMotor.rotate(-convertAngle(leftRadius, track, 90), true);
	              rightMotor.rotate(convertAngle(rightRadius, track, 90), false);
	              carStop();
	              leftMotor.setSpeed(FORWARD_SPEED/2);
	              rightMotor.setSpeed(FORWARD_SPEED/2);
	              leftMotor.rotate(convertDistance(leftRadius, 15), true);
	              rightMotor.rotate(convertDistance(rightRadius, 15), false);
	              carStop();
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
  public void run() {
	 
	  /*try {
		travelTo(LLx * tileSize,LLy*tileSize);
	} catch (OdometerExceptions e1) {
		// TODO Auto-generated catch block
		e1.printStackTrace();
	}*/
	  System.out.println("Start");
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
  private void carStop() {
	    leftMotor.stop(true);
	    rightMotor.stop();
	  }
}
