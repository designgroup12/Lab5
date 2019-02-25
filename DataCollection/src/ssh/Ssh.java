package ssh;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class Ssh {

  public static void main(String[] args) throws InterruptedException, FileNotFoundException, UnsupportedEncodingException  {
		// TODO Auto-generated method stub
	PrintWriter writer = new PrintWriter("Ydata.txt","UTF-8");
	EV3ColorSensor sensor = new EV3ColorSensor(SensorPort.S2);
	SensorMode sampleProviderLight;
	sampleProviderLight = sensor.getRGBMode();
	float[][] value = new float[3][10];
	float[] rgbData = new float[sampleProviderLight.sampleSize()];
	float[] red = new float[10];
	double[] nreds = new double[10];
	float[] green = new float[10];
	double[] ngreens = new double[10];
	float[] blue = new float[10];
	double[] nblues = new double[10];
	float mBlue = 0,sBlue = 0,mGreen = 0,sGreen = 0,mRed = 0,sRed = 0,snRed = 0,snGreen = 0,snBlue = 0;
	double nmBlue = 0,nmGreen = 0,nmRed = 0;
	try{
	  System.out.println("the detect value is:");
	  writer.write("the detect value is:\n");
	  for (int i = 0;i<10;i++) {
		sampleProviderLight.fetchSample(rgbData,0);
		value[0][i] = rgbData[0];
		red[i] = value[0][i];
		value[1][i] = rgbData[1];
		green[i] = value[1][i];
		value[2][i] = rgbData[2];
		//System.out.println(rgbData[2]);
		blue[i] = value[2][i];
		nreds[i] = red[i] /Math.sqrt(Math.pow(red[i], 2)+Math.pow(green[i], 2)+Math.pow(blue[i], 2));
		ngreens[i] = green[i] /Math.sqrt(Math.pow(red[i], 2)+Math.pow(green[i], 2)+Math.pow(blue[i], 2));
		nblues[i] = blue[i] /Math.sqrt(Math.pow(red[i], 2)+Math.pow(green[i], 2)+Math.pow(blue[i], 2));
		System.out.print(String.format("%d: %f %f %f %f %f %f%n",System.currentTimeMillis(),red[i],green[i],blue[i],nreds[i],ngreens[i],nblues[i]));
		writer.write(red[i] + "  " + green[i] + "  " + blue[i] + "  " + nreds[i] +"  " + ngreens[i] + "  " +  nblues[i] + "\n");
		  Thread.sleep(1000);
	  }
	  for (int i = 0;i<10; i++) {
		sBlue += blue[i];
		sGreen += green[i];
		sRed += red[i];
		snRed += nreds[i];
		snGreen += ngreens[i];
		snBlue += nblues[i];
 	  }
	  
	  
	  System.out.println();
	  System.out.println();
	  writer.write("\n");
	  writer.write("\n");
	  mRed = sRed / 10;
	  System.out.println("mean of red: " + mRed);
	  writer.write("mean of red: " + mRed+"\n");
	  mGreen = sGreen/10;
	  System.out.println("mean of green: " + mGreen);
	  writer.write("mean of green: " + mGreen+"\n");
	  mBlue = sBlue/10;
	  System.out.println("mean of blue: " + mBlue);
	  writer.write("mean of blue: " + mBlue+"\n");
	  nmRed = mRed /Math.sqrt(Math.pow(mRed, 2)+Math.pow(mGreen, 2)+Math.pow(mBlue, 2));
	  System.out.println("normal mean of red: " + nmRed);
	  writer.write("normal mean of red: " + nmRed+"\n");
	  nmGreen = mGreen/Math.sqrt(Math.pow(mRed, 2)+Math.pow(mGreen, 2)+Math.pow(mBlue, 2));
	  System.out.println("normal mean of green: " + nmGreen);
	  writer.write("normal mean of green: " + nmGreen+"\n");
	  nmBlue = mBlue/Math.sqrt(Math.pow(mRed, 2)+Math.pow(mGreen, 2)+Math.pow(mBlue, 2));
	  System.out.println("normal mean of blue: " + nmBlue);
	  writer.write("normal mean of blue: " + nmBlue+"\n");
	  System.out.println("mean of normal red: " + snRed/10 );
	  writer.write("mean of normal red: " + snRed/10 + "\n");
	  System.out.println("mean of normal green: " + snGreen/10 );
	  writer.write("mean of normal green: " + snGreen/10 + "\n");
	  System.out.println("mean of normal blue: " + snBlue/10 );
	  writer.write("mean of normal blue: " + snBlue/10 + "\n");
	}finally {
	  writer.close();
	  sensor.close();
	}
  }

}
