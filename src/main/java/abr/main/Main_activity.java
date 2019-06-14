/**
 * Rescue Robotics 2016 App
 * Developed by Cognitive Anteater Robotics Laboratory at University of California, Irvine
 * Controls wheeled robot through IOIO
 * Parts of code adapted from OpenCV blob follow
 * Before running, connect phone to IOIO with a bluetooth connection
 * If you would like to uncomment sections for message passing, first connect peer phones using wifi direct
 */
package abr.main;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.common.api.GoogleApiClient.ConnectionCallbacks;
import com.google.android.gms.common.api.GoogleApiClient.OnConnectionFailedListener;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Iterator;
import java.util.List;

import ioio.lib.util.IOIOLooper;
import ioio.lib.util.IOIOLooperProvider;
import ioio.lib.util.android.IOIOAndroidApplicationHelper;

public class Main_activity extends Activity implements IOIOLooperProvider, SensorEventListener, ConnectionCallbacks, OnConnectionFailedListener,
		CvCameraViewListener2 // implements IOIOLooperProvider: from IOIOActivity
{
	private final IOIOAndroidApplicationHelper helper_ = new IOIOAndroidApplicationHelper(this, this); // from IOIOActivity

	// ioio variables
	IOIO_thread m_ioio_thread;

	//blob detection variables
	private CameraBridgeViewBase mOpenCvCameraView;
	private Mat mRgba;
	private Mat mAttnMap;
	private Mat mAttnMapSmall;
	private Mat mTmp1;
	private Mat mTmp2;
	private Mat mImgThr;
	private Mat mImgGray;
	private Mat mImgGraySmallSize;
	private Mat mImgThrPos;
	private Mat mImgThrNeg;
	private Mat mThrMask;
	private Mat mThrMaskNot;
	private Mat mask;
	private Size smallSize;
	private Size origSize;
	private List<Mat> channels;
	List<MatOfPoint> contours;
	List<MatOfPoint> contourThr;
	List<MatOfPoint> contourMax;
	private Scalar CONTOUR_COLOR_RED;
	private Scalar CONTOUR_COLOR_GREEN;

	// variables for tracking
	private double momentX;	//added
	private double momentY;	//added
	private double centerX;	//added
	private double centerY;	//added


	//app state variables
	private boolean autoMode;

	//variables for logging
	private Sensor mGyroscope;
	private Sensor mGravityS;
	float[] mGravityV;
	float[] mGyro;

	//location variables
	private GoogleApiClient mGoogleApiClient;
	private double curr_lat;
	private double curr_lon;
	private Location curr_loc;
	private LocationRequest mLocationRequest;
	private LocationListener mLocationListener;
	Location dest_loc;
	float distance = 0;

	//variables for compass
	private SensorManager mSensorManager;
	private Sensor mCompass, mAccelerometer;
	float[] mGravity;
	float[] mGeomagnetic;
	public float heading = 0;
	public float bearing;
	private int cnt = 0;
	private int delta = 32;

	//ui variables
	TextView sonar1Text;
	TextView sonar2Text;
	TextView sonar3Text;
	TextView distanceText;
	TextView bearingText;
	TextView headingText;

	//pan and tilt variables
	int panVal=1500;
	int tiltVal=1500;
	boolean panningRight = false;
	boolean tiltingUp = false;
	int panInc;
	int tiltInc;
	//obstacle avoidance variable
	boolean avoidingObstacle;

	int forward_speed = 100;
	int turn_speed = 80;


	//sockets for message passing
	Boolean isClient = true;
	ServerSocket serverSocket;
	Socket socket;
	Socket clientSocket;
	DataInputStream dataInputStream;
	DataOutputStream dataOutputStream;

	// called to use OpenCV libraries contained within the app as opposed to a separate download
	static {
		if (!OpenCVLoader.initDebug()) {
			// Handle initialization error
		}
	}

	// called whenever the activity is created
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.main);

		helper_.create(); // from IOIOActivity

		//set up opencv camera
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
		mOpenCvCameraView.enableView();

		//initialize textviews
		sonar1Text = (TextView) findViewById(R.id.sonar1);
		sonar2Text = (TextView) findViewById(R.id.sonar2);
		sonar3Text = (TextView) findViewById(R.id.sonar3);
		distanceText = (TextView) findViewById(R.id.distanceText);
		bearingText = (TextView) findViewById(R.id.bearingText);
		headingText = (TextView) findViewById(R.id.headingText);

		dest_loc = new Location("");

		autoMode = false;

		//add functionality to autoMode button
		Button buttonAuto = (Button) findViewById(R.id.btnAuto);
		buttonAuto.setOnClickListener(new OnClickListener() {
			public void onClick(View v) {
				if (!autoMode) {
					v.setBackgroundResource(R.drawable.button_auto_on);
					autoMode = true;
				} else {
					v.setBackgroundResource(R.drawable.button_auto_off);
					m_ioio_thread.move(1500);
					m_ioio_thread.turn(1500);
					m_ioio_thread.pan(1500);
					m_ioio_thread.tilt(1500);
					panVal = 1500;
					tiltVal = 1500;
					autoMode = false;
				}
			}
		});

		//set starting autoMode button color
		if (autoMode) {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_on);
		} else {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_off);
		}

		//set up location listener
		mLocationListener = new LocationListener() {
			public void onLocationChanged(Location location) {
				curr_loc = location;
				distance = location.distanceTo(dest_loc);
				bearing = location.bearingTo(dest_loc);
			}

			@SuppressWarnings("unused")
			public void onStatusChanged(String provider, int status, Bundle extras) {
			}

			@SuppressWarnings("unused")
			public void onProviderEnabled(String provider) {
			}

			@SuppressWarnings("unused")
			public void onProviderDisabled(String provider) {
			}
		};

		//set up compass
		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		mCompass = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
		mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		mGravityS = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);

		//set up sockets for communication with other robots
	    /*
	    if (isClient) {
	    	try {
	    		Object[] objects = (new FileClientAsyncTask()).execute().get(); 
	    		socket = (Socket) objects[0];
	    		dataOutputStream = (DataOutputStream) objects[1];
	    		dataInputStream = (DataInputStream) objects[2];
	    	} catch(Exception e) {
	    		Log.e("rescue robotics", e.getMessage());
	    	}  	
	    }
	    else {
	    	try {
	    		Object[] objects = (new FileServerAsyncTask()).execute().get();
	    		serverSocket = (ServerSocket) objects[0];
	    		clientSocket = (Socket) objects[1];
	    		dataInputStream = (DataInputStream) objects[2];
	    		dataOutputStream = (DataOutputStream) objects[3];
	    	} catch(Exception e) {
	    		Log.e("rescue robotics", e.getMessage());
	    	}
	    }
	    */
		// phone must be Android 2.3 or higher and have Google Play store
		// must have Google Play Services: https://developers.google.com/android/guides/setup
		buildGoogleApiClient();
		mLocationRequest = new LocationRequest();
		mLocationRequest.setInterval(2000);
		mLocationRequest.setFastestInterval(500);
		mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
	}

	//Method necessary for google play location services
	protected synchronized void buildGoogleApiClient() {
		mGoogleApiClient = new GoogleApiClient.Builder(this)
				.addConnectionCallbacks(this)
				.addOnConnectionFailedListener(this)
				.addApi(LocationServices.API)
				.build();
	}

	//Method necessary for google play location services
	@Override
	public void onConnected(Bundle connectionHint) {
		// Connected to Google Play services
		curr_loc = LocationServices.FusedLocationApi.getLastLocation(mGoogleApiClient);
		startLocationUpdates();
	}

	//Method necessary for google play location services
	protected void startLocationUpdates() {
		LocationServices.FusedLocationApi.requestLocationUpdates(mGoogleApiClient, mLocationRequest, mLocationListener);
	}

	//Method necessary for google play location services
	@Override
	public void onConnectionSuspended(int cause) {
		// The connection has been interrupted.
		// Disable any UI components that depend on Google APIs
		// until onConnected() is called.
	}

	//Method necessary for google play location services
	@Override
	public void onConnectionFailed(ConnectionResult result) {
		// This callback is important for handling errors that
		// may occur while attempting to connect with Google.
		//
		// More about this in the 'Handle Connection Failures' section.
	}

	@Override
	public final void onAccuracyChanged(Sensor sensor, int accuracy) {
		// Do something here if sensor accuracy changes.
	}

	//Called whenever the value of a sensor changes
	@Override
	public final void onSensorChanged(SensorEvent event) {
		if (m_ioio_thread != null) {
			setText("sonar1: " + m_ioio_thread.get_sonar1_reading(), sonar1Text);
			setText("sonar2: " + m_ioio_thread.get_sonar2_reading(), sonar2Text);
			setText("sonar3: " + m_ioio_thread.get_sonar3_reading(), sonar3Text);
			setText("distance: " + distance, distanceText);
			setText("bearing: " + bearing, bearingText);
			setText("heading: " + heading, headingText);
		}

		if (event.sensor.getType() == Sensor.TYPE_GRAVITY)
			mGravityV = event.values;
		if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
			mGyro = event.values;
		if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
			mGravity = event.values;
		if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
			mGeomagnetic = event.values;
		if (mGravity != null && mGeomagnetic != null) {
			float[] temp = new float[9];
			float[] R = new float[9];
			//Load rotation matrix into R
			SensorManager.getRotationMatrix(temp, null, mGravity, mGeomagnetic);
			//Remap to camera's point-of-view
			SensorManager.remapCoordinateSystem(temp, SensorManager.AXIS_X, SensorManager.AXIS_Z, R);
			//Return the orientation values
			float[] values = new float[3];
			SensorManager.getOrientation(R, values);
			//Convert to degrees
			for (int i = 0; i < values.length; i++) {
				Double degrees = (values[i] * 180) / Math.PI;
				values[i] = degrees.floatValue();
			}
			//Update the compass direction
			heading = values[0] + 12;
			heading = (heading * 5 + fixWraparound(values[0] + 12)) / 6; //add 12 to make up for declination in Irvine, average out from previous 2 for smoothness
		}
	}

	//Scan for QR code and save information to phone
	public String scan(Mat frame) {
		Bitmap bMap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
		Utils.matToBitmap(frame, bMap);
		int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
		//copy pixel data from the Bitmap into the 'intArray' array  
		bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

		LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);

		BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
		Reader reader = new QRCodeReader();

		String text;

		try {
			Result result = reader.decode(bitmap);
			text = result.getText();
			Calendar calendar = Calendar.getInstance();
			java.util.Date now = calendar.getTime();
			java.sql.Timestamp currentTimestamp = new java.sql.Timestamp(now.getTime());
			String time = currentTimestamp.toString();
			curr_lat = curr_loc.getLatitude();
			curr_lon = curr_loc.getLongitude();
			String info = text + " ,Lat:" + curr_lat + " ,Lon:" + curr_lon + " ,Time:" + time;
			try {
				File newFolder = new File(Environment.getExternalStorageDirectory(), "RescueRobotics");
				if (!newFolder.exists()) {
					newFolder.mkdir();
				}
				try {
					File file = new File(newFolder, time + ".txt");
					file.createNewFile();
					FileOutputStream fos = new FileOutputStream(file);
					try {
						byte[] b = info.getBytes();
						fos.write(b);
						fos.close();
						ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
						toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 200);
					} catch (IOException e) {
						Log.e("app.main", "Couldn't write to SD");
					}
				} catch (Exception ex) {
					Log.e("app.main", "Couldn't write to SD");
				}
			} catch (Exception e) {
				Log.e("app.main", "Couldn't write to SD");
			}
			Log.i("rescue robotics", text);
			return text;
		} catch (NotFoundException e) {
			e.printStackTrace();
			text = "no code found";
		} catch (ChecksumException e) {
			e.printStackTrace();
			text = "checksum error";
		} catch (FormatException e) {
			e.printStackTrace();
			text = "format error";
		}
		Log.i("rescue robotics", text);

		return text;
	}

	//Called whenever activity resumes from pause
	@Override
	public void onResume() {
		super.onResume();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.enableView();
		mSensorManager.registerListener(this, mCompass, SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(this, mGravityS, SensorManager.SENSOR_DELAY_NORMAL);
		if (mGoogleApiClient.isConnected()) {
			startLocationUpdates();
		}
	}

	//Called when activity pauses
	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		mSensorManager.unregisterListener(this);
		stopLocationUpdates();
	}

	protected void stopLocationUpdates() {
		LocationServices.FusedLocationApi.removeLocationUpdates(mGoogleApiClient, mLocationListener);
	}

	//Called when activity restarts. onCreate() will then be called
	@Override
	public void onRestart() {
		super.onRestart();
		Log.i("activity cycle", "main activity restarting");
	}

	public void onCameraViewStarted(int width, int height) {
		smallSize = new org.opencv.core.Size(1600/25,900/25);
		origSize = new org.opencv.core.Size(1600, 900);
		mRgba = new Mat(height, width, CvType.CV_8UC4);
		mAttnMap = new Mat(origSize, CvType.CV_32FC1);
		channels = new ArrayList(4);
		mAttnMapSmall = new Mat(smallSize, CvType.CV_32FC1);
		mTmp1 = new Mat(smallSize, CvType.CV_32FC1);
		mTmp2 = new Mat(smallSize, CvType.CV_32FC1);
		mImgThr = new Mat(smallSize, CvType.CV_8UC1);
		mImgThrPos = new Mat(smallSize, CvType.CV_8UC1);
		mImgThrNeg = new Mat(smallSize, CvType.CV_8UC1);
		mThrMask = new Mat(smallSize, CvType.CV_8UC1);
		mThrMaskNot = new Mat(smallSize, CvType.CV_8UC1);
		mask = new Mat(smallSize, CvType.CV_8UC1);
		mImgGray = new Mat(origSize, CvType.CV_8UC1);
		mImgGraySmallSize = new Mat(smallSize, CvType.CV_8UC1);
		contours = new ArrayList<MatOfPoint>();
		contourThr = new ArrayList<MatOfPoint>();
		contourMax = new ArrayList<MatOfPoint>();
		CONTOUR_COLOR_RED = new Scalar(255, 0, 0, 255);
		CONTOUR_COLOR_GREEN = new Scalar(0, 255, 0, 255);

	}

	//Called when camera view stops
	public void onCameraViewStopped() {
		mRgba.release();
	}
	//Called at every camera frame. Main controls of the robot movements are in this function

	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {

		int rows, cols;
		int cntContour = 0;
		double maxArea;
		boolean foundSalientObject = false;
		mRgba = inputFrame.rgba();
		mImgThrPos.setTo(new Scalar(0));
		mImgThrNeg.setTo(new Scalar(0));
		mAttnMapSmall.setTo(new Scalar(0));
		mask.setTo(new Scalar(255));
		MatOfPoint maxContour = new MatOfPoint();

		Core.split(mRgba,channels);

		for (int chans = 0; chans < 3; chans++) {

			Imgproc.resize(channels.get(chans), mImgGraySmallSize, smallSize);

			// Log.d("computeSaliency", "Rows=" + rows + " Cols=" + cols);
			rows = mImgGraySmallSize.rows();
			cols = mImgGraySmallSize.cols();

			// Log.d("computeSaliency", "Min=" + Core.minMaxLoc(mImgGraySmallSize).minVal + " Max=" + Core.minMaxLoc(mImgGraySmallSize).maxVal);

			for (int i=delta; i < 256; i=i+delta) {
				Imgproc.threshold(mImgGraySmallSize, mImgThr, i, 256.0, Imgproc.THRESH_BINARY);
				mThrMask = mImgThr;
				Core.bitwise_not(mThrMask, mThrMaskNot);

				double[] elem;
//		Log.d("computeSaliency", "mImgThr.Rows=" + mImgThr.rows() + " mImgThr.Cols=" + mImgThr.cols());
				for (int j = 0; j < rows; j++) {
					elem = mImgThr.get(0, j);
					if (elem[0] != 1) {
						Imgproc.floodFill(mImgThr, new Mat(), new Point(0, j), new Scalar(1), new Rect(), new Scalar(0), new Scalar(0), 8);
					}
					elem = mImgThr.get(j, cols - 1);
					// Log.d("computeSaliency", "j=" + j + " elem[0]=" + elem[0]);
					if (elem[0] < 128) {
						// Log.d("computeSaliency", "j=" + j);
						Imgproc.floodFill(mImgThr, new Mat(), new Point(cols - 1, j), new Scalar(1), new Rect(), new Scalar(0), new Scalar(0), 8);
					}
				}
				for (int j = 0; j < cols; j++) {
					elem = mImgThr.get(0, j);
					if (elem[0] != 1) {
						Imgproc.floodFill(mImgThr, new Mat(), new Point(j, 0), new Scalar(1), new Rect(), new Scalar(0), new Scalar(0), 8);
					}
					elem = mImgThr.get(rows - 1, j);
					if (elem[0] != 1) {
						Imgproc.floodFill(mImgThr, new Mat(), new Point(j, rows - 1), new Scalar(1), new Rect(), new Scalar(0), new Scalar(0), 8);
					}
				}

//			Log.d("computeSaliency", i + ": mThrMask.Min = " + Core.minMaxLoc(mThrMask).minVal);
//			Log.d("computeSaliency", i + ": mThrMask.Max = " + Core.minMaxLoc(mThrMask).maxVal);
//			Log.d("computeSaliency", i + ": mThrMaskNot.Min = " + Core.minMaxLoc(mThrMaskNot).minVal);
//			Log.d("computeSaliency", i + ": mThrMaskNot.Max = " + Core.minMaxLoc(mThrMaskNot).maxVal);
				Core.bitwise_and(mImgThr, mThrMask, mImgThrPos);
				Core.bitwise_and(mImgThr, mThrMaskNot, mImgThrNeg);
				//Log.d("computeSaliency", i + ": mImgThrPos.Max = " + Core.minMaxLoc(mImgThrPos).maxVal);
				//Log.d("computeSaliency", i + ": mImgThrNeg.Max = " + Core.minMaxLoc(mImgThrNeg).maxVal);
				Imgproc.dilate(mImgThrPos, mImgThrPos, new Mat());
				Imgproc.dilate(mImgThrNeg, mImgThrNeg, new Mat());
				Core.add(mImgThrPos,mImgThrNeg,mTmp1, new Mat (), CvType.CV_32FC1);
			}
			Core.add(mAttnMapSmall,mTmp1, mTmp2, new Mat (), CvType.CV_32FC1);
			mAttnMapSmall=mTmp2;
			// Log.d("computeSaliency", "mAttnMapSmall.Max(" + chans + ") = " + Core.minMaxLoc(mAttnMapSmall).maxVal);
		}

//		Core.divide(mAttnMapSmall,new Scalar(5),mAttnMapSmall);
		//Log.d("computeSaliency", "mAttnMapSmall.Max = " + Core.minMaxLoc(mAttnMapSmall).maxVal);
		Imgproc.threshold(mAttnMapSmall, mAttnMapSmall, 0.75*Core.minMaxLoc(mAttnMapSmall).maxVal, Core.minMaxLoc(mAttnMapSmall).maxVal, Imgproc.THRESH_BINARY);
		//Log.d("computeSaliency", "mAttnMapSmall.Max = " + Core.minMaxLoc(mAttnMapSmall).maxVal);

//		Core.normalize(mAttnMapSmall,mAttnMapSmall,0, 255, Core.NORM_MINMAX);
		mAttnMapSmall.convertTo(mAttnMapSmall,CvType.CV_8UC1);
		Imgproc.resize(mAttnMapSmall,mAttnMap,origSize);

		contours.clear();
		contourThr.clear();
		contourMax.clear();
		centerX = (double)mAttnMap.cols()/2;
		centerY = (double)mAttnMap.rows()/2;
		Imgproc.findContours(mAttnMap, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		Iterator<MatOfPoint> each = contours.iterator();
		each = contours.iterator();

		maxArea = 0;
		foundSalientObject = false;
		while (each.hasNext()) {
			MatOfPoint contour = each.next();

			if (Imgproc.contourArea(contour) > 10000 && Imgproc.contourArea(contour) < 100000 && Imgproc.boundingRect(contour).width > 100 && Imgproc.boundingRect(contour).height < 700) {
				foundSalientObject = true;
				//Log.d("computeSaliency", "Area=" + Imgproc.contourArea(contour) + " H=" + Imgproc.boundingRect(contour).height + " W=" + Imgproc.boundingRect(contour).width);
				contourThr.add(contour);
				if (Imgproc.contourArea(contour)/(Imgproc.boundingRect(contour).height*Imgproc.boundingRect(contour).width) > maxArea) {
					maxContour = contour;
					maxArea = Imgproc.contourArea(contour)/(Imgproc.boundingRect(contour).height*Imgproc.boundingRect(contour).width);
				}
			}
		}

		if (foundSalientObject) {
			contourMax.add(maxContour);
			calculateCOM(maxContour);
			momentX = getMomentX();
			momentY = getMomentY();
		}
		else {
			momentX = 0;
			momentY = 0;
		}

		//Log.d("computeSaliency", "--------------------");

/*		Log.d("computeSaliency", "Contours count: " + contours.size());
		for (int c=0; c < contours.size(); c++) {
			Log.d("computeSaliency", "Area " + c + ": " + Imgproc.contourArea(contours.get(c)));
			if (Imgproc.contourArea(contours.get(c)) > 10000) {
				Imgproc.drawContours(mRgba, contours.get(c), -1, CONTOUR_COLOR, 10);
				cntContour++;
			}
		}*/

		Imgproc.drawContours(mRgba, contourThr, -1, CONTOUR_COLOR_GREEN, 10);
		Imgproc.drawContours(mRgba, contourMax, -1, CONTOUR_COLOR_RED, 10);

		if (autoMode) { // only move if autoMode is on
			//adjust pan/tilt

			if(!foundSalientObject){
				tiltVal = 1500;
				if(panningRight){
					m_ioio_thread.pan(panVal+=30);
					if(panVal >= 2000)
						panningRight = false;
				} else {
					m_ioio_thread.pan(panVal-=30);
					if(panVal <= 1000)
						panningRight = true;
				}

			} else {
				// panInc = 40 + (int) Math.exp(.03* Math.abs(momentX));
				if(momentX > 25){
					panVal += (int) Math.exp(.01* Math.abs(momentX));
					if (panVal > 2000) {
						panVal = 2000;
					}
					m_ioio_thread.pan(panVal);
				}
				else if(momentX < -25){
					panVal -= (int) Math.exp(.01* Math.abs(momentX));
					if (panVal < 1000) {
						panVal = 1000;
					}
					m_ioio_thread.pan(panVal);
				}
				// tiltInc = 20 + (int) Math.exp(.03* Math.abs(momentY));
				if(momentY > 25){
					tiltVal += 20 + (int) Math.exp(.01* Math.abs(momentY));
					if(tiltVal > 1600) {
						tiltVal = 1600;
					}
					m_ioio_thread.tilt(tiltVal);
				}
				else if(momentY < -25){
					tiltVal -= 20+ (int) Math.exp(.01* Math.abs(momentY));
					if(tiltVal < 1000) {
						tiltVal = 1000;
					}
					m_ioio_thread.tilt(tiltVal);
				}
				Log.d("computeSaliency", "Salient Object: X=" + momentX + " Y=" + momentY + " Pan=" + panVal + " Tilt=" + tiltVal);
			}

			//move
			//handle obstacles
			int sonar1 = m_ioio_thread.get_sonar1_reading();
			int sonar2 = m_ioio_thread.get_sonar2_reading();
			int sonar3 = m_ioio_thread.get_sonar3_reading();

			if (sonar2 < 30 && !avoidingObstacle){
				avoidingObstacle = true;
			}
			else if(avoidingObstacle){
				if(sonar2 < 30){
					if(sonar1 > sonar3){
						m_ioio_thread.turn(1500-80);
					} else {
						m_ioio_thread.turn(1500+80);
					}
				} else {
					m_ioio_thread.move(1500+forward_speed);
				}
				if(sonar1 >= 20 && sonar2 >= 20 && sonar3 >= 20){
					avoidingObstacle = false;
				}

			}
			//follow blob
			else {
				if (foundSalientObject) {
					if(!(panVal > 1300 && panVal < 1700)){
						m_ioio_thread.move(1500+forward_speed);
						if(panVal>1500){
							m_ioio_thread.turn(1500+turn_speed);
						}
						else {
							m_ioio_thread.turn(1500-turn_speed);
						}
					}
					else{
						m_ioio_thread.turn(1500);
						m_ioio_thread.move(1500+forward_speed);
					}
				} else  {
					m_ioio_thread.turn(1500);
					m_ioio_thread.move(1500+forward_speed);
				}
			}

		}
/*		else {
			m_ioio_thread.move(1500);
			m_ioio_thread.turn(1500);
			m_ioio_thread.pan(1500);
			m_ioio_thread.tilt(1500);
			panVal = 1500;
			tiltVal = 1500;
		}*/

		return mRgba;
	}

	//send an integer using output stream from socket
	public void sendInt(int intToSend){
		if(dataOutputStream != null)
			try {
				dataOutputStream.writeInt(intToSend);
				Log.i("rescue robotics", "grid sent");
			} catch (IOException e) {
				Log.e("rescue robotics", e.getMessage());
			}
	}

	//receive an integer using input stream from socket
	public int getInt(){
		try {
			if(dataInputStream != null && dataInputStream.available() >= 4*17) {
				return dataInputStream.readInt();
			}
		} catch (IOException e) {
			Log.e("rescue robotics", e.getMessage());
		}
		return 0;
	}

	//revert any degree measurement back to the -179 to 180 degree scale
	public float fixWraparound(float deg){
		if(deg <= 180.0 && deg > -179.99)
			return deg;
		else if(deg > 180)
			return deg-360;
		else
			return deg+360;

	}

	//determine whether 2 directions are roughly pointing in the same direction, correcting for angle wraparound
	public boolean sameDir(float dir1, float dir2){
		float dir = bearing%360;
		float headingMod = heading%360;
		//return (Math.abs((double) (headingMod - dir)) < 22.5 || Math.abs((double) (headingMod - dir)) > 337.5);
		return (Math.abs((double) (headingMod - dir)) < 2.5 || Math.abs((double) (headingMod - dir)) > 357.5);
	}

	//set the text of any text view in this application
	public void setText(final String str, final TextView tv)
	{
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				tv.setText(str);
			}
		});
	}

	/****************************************************** functions from IOIOActivity *********************************************************************************/

	/**
	 * Create the {@link IOIO_thread}. Called by the
	 * {@link IOIOAndroidApplicationHelper}. <br>
	 * Function copied from original IOIOActivity.
	 * 
	 * @see {@link #get_ioio_data()} {@link #start_IOIO()}
	 * */
	@Override
	public IOIOLooper createIOIOLooper(String connectionType, Object extra) {
		if (m_ioio_thread == null
				&& connectionType
						.matches("ioio.lib.android.bluetooth.BluetoothIOIOConnection")) {
			m_ioio_thread = new IOIO_thread(this);
			return m_ioio_thread;
		} else
			return null;
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		Log.i("activity cycle","main activity being destroyed");
		helper_.destroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	protected void onStart() {
		super.onStart();
		Log.i("activity cycle","main activity starting");
		helper_.start();
		mGoogleApiClient.connect();
	}

	@Override
	protected void onStop() {
		Log.i("activity cycle","main activity stopping");
		super.onStop();
		helper_.stop();
		mGoogleApiClient.disconnect();
		try {
			if(socket != null)
				socket.close();
			if(serverSocket != null)
				serverSocket.close();
			if(clientSocket != null)
				clientSocket.close();
		} catch (IOException e) {
			Log.e("rescue robotics", e.getMessage());
		}
		
	}

	@Override
	protected void onNewIntent(Intent intent) {
		super.onNewIntent(intent);
			if ((intent.getFlags() & Intent.FLAG_ACTIVITY_NEW_TASK) != 0) {
			helper_.restart();
		}
	}

	//Added - calculates center of mass of blob with largest area
	public void calculateCOM(MatOfPoint contour){
		List<Point> contourPoints = contour.toList();
		momentX = 0;
		momentY = 0;
		for(int i = 0; i < contourPoints.size(); i++){
			momentX += contourPoints.get(i).x;
			momentY += contourPoints.get(i).y;
		}
		momentX = momentX/contourPoints.size();
		momentY = momentY/contourPoints.size();

		momentX = momentX-centerX;
		momentY = momentY-centerY;
	}

	//Added
	public double getMomentX(){
		return momentX;
	}

	//Added
	public double getMomentY(){
		return momentY;
	}

}
	


