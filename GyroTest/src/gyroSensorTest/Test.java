package gyroSensorTest;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Test {
	 public static final double WHEEL_RAD = 2.10;
	  public static final double TRACK = 13.6 ;
	 static EV3GyroSensor gySensor = new EV3GyroSensor(SensorPort.S4);
	 static SampleProvider gyAngle = gySensor.getAngleMode();
	 static float[] angles = new float[gyAngle.sampleSize()];
	 static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
     static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
     static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
     static SampleProvider us = usSensor.getMode("Distance");
     static float[] usValues = new float[us.sampleSize()];
	public static void main(String[] args) {
		// TODO Auto-generated method stub
         gySensor.reset();
        gyAngle.fetchSample(angles, 0);
        System.out.println(angles[0]);
        //goStraightLine(7*30.48,180);
        leftMotor.setSpeed(180);
        rightMotor.setSpeed(180);
        leftMotor.rotate(convertDistance(WHEEL_RAD, 7*30.48), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, 7*30.48), false);
		/*gyAngle.fetchSample(angles, 0);
        System.out.println(angles[0]);
        leftMotor.rotate(convertDistance(WHEEL_RAD, 30.48), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, 30.48), false);
		gyAngle.fetchSample(angles, 0);
        System.out.println(angles[0]);
        leftMotor.rotate(convertDistance(WHEEL_RAD, 60.96), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, 60.96), false);
		gyAngle.fetchSample(angles, 0);
        System.out.println(angles[0]);*/
      //  for(int i = 0;i<7;i++) {
        /*	leftMotor.forward();
    		rightMotor.forward();
    		while(usfetch() > 5){
    			gyAngle.fetchSample(angles, 0);
    			if(angles[0] >= 2) {
    				leftMotor.setSpeed(180);
    		        rightMotor.setSpeed(185);
    		        leftMotor.forward();
    	    		rightMotor.forward();
    		gyAngle.fetchSample(angles, 0);
            System.out.println(angles[0]);
            }else if(angles[0]<=-2) {
            		leftMotor.setSpeed(185);
    		        rightMotor.setSpeed(180);
    		        leftMotor.forward();
    	    		rightMotor.forward();
    		gyAngle.fetchSample(angles, 0);
            System.out.println(angles[0]);
            	}
            }
    			leftMotor.stop(true);
    			rightMotor.stop();
    			
    		}*/
        //}
		
        // GyroPoller gyPoller = new GyroPoller(); 
	}
	private static void turnRight(double degree) {
	 gyAngle.fetchSample(angles, 0);
	 double angle = angles[0];
	 while(angles[0] - angle != 90){
		 
		 leftMotor.setSpeed(70);
	     rightMotor.setSpeed(70);
	     leftMotor.forward();
	     rightMotor.backward();
	     gyAngle.fetchSample(angles, 0);
	 }
		leftMotor.stop(true);
		rightMotor.stop();
	}
	private static void turnLeft(double degree) {
		 gyAngle.fetchSample(angles, 0);
		 double angle = angles[0];
		 while(angle - angles[0] != 90){
			 
			 leftMotor.setSpeed(70);
		     rightMotor.setSpeed(70);
		     leftMotor.backward();
		     rightMotor.forward();
		     gyAngle.fetchSample(angles, 0);
		 }
			leftMotor.stop(true);
			rightMotor.stop();
		}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double wideltaTh, double angle) {
		return convertDistance(radius, Math.PI * wideltaTh * angle / 360.0);
	}
	private static float usfetch() {
		  us.fetchSample(usValues, 0);
		  if ((usValues[0]*100)>200) {
			  return 200;
		  }else {
			  return usValues[0]*100;
		  }
	  }
	private static void goStraightLine(double distance,int speed) {
		  carStop();
		  double angle = gyroFetch();
		  double p1 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*2.09*Math.PI/360;
		  double p2 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*2.09*Math.PI/360;
		 double distance1 = p2 - p1;
		//double  oldx = odometer.getXYT()[0]; 
		//double  curx = odometer.getXYT()[0]; 
		//double  oldy = odometer.getXYT()[1]; 
		//double  cury = odometer.getXYT()[1];
		int speed1;
		//double distance1 = Math.hypot(curx - oldx,cury - oldy);
		while(distance1 < distance) {
			speed1 =(int) (speed - 25/((distance - distance1)+1));
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			leftMotor.forward();
			rightMotor.forward();
			for(int i = 0;i<3;i++);
			directionCorrection(angle,speed1);
			p2 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*2.09*Math.PI/360;
			distance1 = p2 - p1;
		}
		  carStop();
	  }
	private static void directionCorrection(double degree,int speed) {
		double degree1 = gyroFetch();
		if(degree1 - degree  >= 2) {
			leftMotor.setSpeed(speed);
	        rightMotor.setSpeed((float)(speed+(degree1 - degree)*6));
	        leftMotor.forward();
  		rightMotor.forward();
  }else if(degree1 - degree <=-2) {
  		leftMotor.setSpeed((float)(speed+(degree - degree1)*6));
	        rightMotor.setSpeed(speed);
	        leftMotor.forward();
  		rightMotor.forward();
  	}
  }
	private static double gyroFetch() {
		  gyAngle.fetchSample(angles, 0);
		  //angleCorrection();
		  return angles[0];
	  }
	private static void carStop() {
	    leftMotor.stop(true);
	    rightMotor.stop();
	  }
}
