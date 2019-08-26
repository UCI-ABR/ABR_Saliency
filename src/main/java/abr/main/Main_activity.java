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
import android.hardware.GeomagneticField;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Criteria;
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
import android.widget.Toast;

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
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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
import java.util.List;
import java.util.Random;

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
    private Mat mRgbaNorm;
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
    private boolean contourDisplayed;

    //variables for logging
    private Sensor mGyroscope;
    private Sensor mGravityS;
    float[] mGravityV;
    float[] mGyro;

    //location variables
    private GoogleApiClient mGoogleApiClient;
    private double curr_lat;
    private double curr_lon;
    private double dest_lat;
    private double dest_lon;
    private Location curr_loc;
    private LocationRequest mLocationRequest;
    private LocationListener mLocationListener;
    Location dest_loc;
    float distance = 0;
    int locInx = 0;
    private Criteria criteria ;


    //variables for compass
    private SensorManager mSensorManager;
    private Sensor mCompass, mAccelerometer;
    float[] mGravity;
    float[] mGeomagnetic;
    public float heading = 0;
    public float bearing;
    public float myHeading;
    private int cnt = 0;
    private int delta = 32;

    private ArrayList<Location> waypoints;
    private double[] pWait;
    private static final int T = 150;
    private static final double pRwd = 0.5;
    private double reward;

    private long timeWayPtStart;
    private Random randNumGen;
    private boolean skipWayPt;
    private boolean reachedLastWayPt = false;
    private boolean reachedWayPt = false;

    // variables for writing out data
    File rrFile;
    File recordingFile;
    FileOutputStream fosRR;

    //ui variables
    TextView sonar1Text;
    TextView sonar2Text;
    TextView sonar3Text;
    TextView distanceText;
    TextView bearingText;
    TextView headingText;
    String wayTxt = "WayPoints ";

    //pan and tilt variables - NOT USED IN LINE FOLLOW
    /* int panVal=1500;
    int tiltVal=1400;
    boolean panningRight = false;
    boolean tiltingUp = false;
    int panInc;
    int tiltInc;
    */
    //obstacle avoidance variable
    boolean avoidingObstacle;

    int forward_speed = 100;
    int turn_speed = 80;
    int turn_right = 1575;
    int turn_left = 1425;
    int turn_right_sharp = 1600;
    int turn_left_sharp = 1400;
    int turn_none = 1500;
    int forward_slow = 1600;
    int forward_fast = 1700;
    int forward_stop = 1500;


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
                    // JLK code block for creating file.  Test test
                    try {
                        Calendar calendar = Calendar.getInstance();
                        java.util.Date now = calendar.getTime();
                        java.sql.Timestamp currentTimestamp = new java.sql.Timestamp(now.getTime());
                        String time = currentTimestamp.toString();
                        time = time.replaceAll("[|?*<\":>+\\[\\]/']", "_");

                        File[] externalDirs = getExternalFilesDirs(null);
                        if(externalDirs.length > 1) {
                            //rrFile = new File(externalDirs[1].getAbsolutePath() + "/rescuerobotics/"+time);
                            rrFile = new File(Environment.getExternalStorageDirectory() + "/lineFollow/");
                            if (!rrFile.exists()) {
                                rrFile.mkdirs();
                            }
                        } else {
                            //rrFile = new File(externalDirs[0].getAbsolutePath() + "/rescuerobotics/"+time);
                            rrFile = new File(Environment.getExternalStorageDirectory() + "/lineFollow/");
                            if (!rrFile.exists()) {
                                rrFile.mkdirs();
                            }
                        }
                        Log.d("lineFollow", "created directory " + Environment.getExternalStorageDirectory() + "/lineFollow/");
                        recordingFile = new File(rrFile, time+".csv");
                        recordingFile.createNewFile();

                        fosRR = new FileOutputStream(recordingFile);
                        String labels = "Time,Lat,Lon,Waypt,Reward\n";
                        byte[] b = labels.getBytes();
                        fosRR.write(b);
                        Log.i("test","made folder");
                    } catch (IOException e) {
                        Log.e("lineFollow", e.toString());
                    }
                    String mill_timestamp = System.currentTimeMillis() + "";
                    String info = mill_timestamp + "," + curr_loc.getLatitude() + "," + curr_loc.getLongitude() + ",0," + pRwd + "\n";
                    try {
                        byte[] b = info.getBytes();
                        fosRR.write(b);
                        Log.i("test", "wrote");
                    } catch (IOException e) {
                        Log.e("lineFollow", e.toString());
                    }
                    timeWayPtStart = System.currentTimeMillis();
                    autoMode = true;
                } else {
                    v.setBackgroundResource(R.drawable.button_auto_off);
                    m_ioio_thread.move(1500);
                    m_ioio_thread.turn(1500);

                    // Pan and tilt set in IOIO thread on initialization.
                    // Never changed in Main Activity
                    // m_ioio_thread.pan(panVal);
                    // m_ioio_thread.tilt(tiltVal);
                    autoMode = false;
                }
            }
        });

        //set starting autoMode button color
        if (autoMode) {
            buttonAuto.setBackgroundResource(R.drawable.button_contour_on);
        } else {
            buttonAuto.setBackgroundResource(R.drawable.button_contour_off);
        }

        contourDisplayed = false;

        // JLK removed contour button - used only for line following
/*        Button buttonContour = (Button) findViewById(R.id.btnCnt);
        buttonContour.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                if (!contourDisplayed) {
                    v.setBackgroundResource(R.drawable.button_contour_on);
                    contourDisplayed = true;
                } else {
                    contourDisplayed = false;
                }
            }
        });

        //set starting autoMode button color
        if (contourDisplayed) {
            buttonContour.setBackgroundResource(R.drawable.button_contour_on);
        } else {
            buttonContour.setBackgroundResource(R.drawable.button_contour_off);
        }*/

        waypoints = new ArrayList<Location>();

        // Aldrich Park - Grass loop going from inner ring down the hill
/*        Location l0 = new Location(""); l0.setLatitude(33.644819); l0.setLongitude(-117.843193); waypoints.add(l0);
        Location l1 = new Location(""); l1.setLatitude(33.644820); l1.setLongitude(-117.843472); waypoints.add(l1);
        Location l2 = new Location(""); l2.setLatitude(33.644963); l2.setLongitude(-117.843709); waypoints.add(l2);
        Location l3 = new Location(""); l3.setLatitude(33.645233); l3.setLongitude(-117.843717); waypoints.add(l3);
        Location l4 = new Location(""); l4.setLatitude(33.645430); l4.setLongitude(-117.843739); waypoints.add(l4);
        Location l5 = new Location(""); l5.setLatitude(33.645613); l5.setLongitude(-117.843649); waypoints.add(l5);
        Location l6 = new Location(""); l6.setLatitude(33.645714); l6.setLongitude(-117.843472); waypoints.add(l6);
        Location l7 = new Location(""); l7.setLatitude(33.645799); l7.setLongitude(-117.843271); waypoints.add(l7);
        Location l8 = new Location(""); l8.setLatitude(33.645732); l8.setLongitude(-117.843031); waypoints.add(l8);*/

        // removed 1st 3 points in Aldrich Park because the GPS was unreliable
//        Location l0 = new Location(""); l0.setLatitude(33.646443); l0.setLongitude(-117.841360); waypoints.add(l0);
//        Location l1 = new Location(""); l1.setLatitude(33.646345); l1.setLongitude(-117.841318); waypoints.add(l1);
//        Location l2 = new Location(""); l2.setLatitude(33.646175); l2.setLongitude(-117.841247); waypoints.add(l2);
//      Aldrich Park Waypoints from SSL to the Infinity Pool near Physics
/*
        Location l0 = new Location(""); l0.setLatitude(33.645987); l0.setLongitude(-117.841210); waypoints.add(l0);
        Location l1 = new Location(""); l1.setLatitude(33.645775); l1.setLongitude(-117.841203); waypoints.add(l1);
        Location l2 = new Location(""); l2.setLatitude(33.645490); l2.setLongitude(-117.841283); waypoints.add(l2);
        Location l3 = new Location(""); l3.setLatitude(33.645277); l3.setLongitude(-117.841391); waypoints.add(l3);
        Location l4 = new Location(""); l4.setLatitude(33.645113); l4.setLongitude(-117.841528); waypoints.add(l4);
        Location l5 = new Location(""); l5.setLatitude(33.644937); l5.setLongitude(-117.841764); waypoints.add(l5);
        Location l6 = new Location(""); l6.setLatitude(33.644817); l6.setLongitude(-117.841991); waypoints.add(l6);
        Location l7 = new Location(""); l7.setLatitude(33.644728); l7.setLongitude(-117.842278); waypoints.add(l7);
        Location l8 = new Location(""); l8.setLatitude(33.644673); l8.setLongitude(-117.842555); waypoints.add(l8);
        Location l9 = new Location(""); l9.setLatitude(33.644696); l9.setLongitude(-117.842776); waypoints.add(l9);
        Location l10 = new Location(""); l10.setLatitude(33.644689); l10.setLongitude(-117.843044); waypoints.add(l10);
        Location l11 = new Location(""); l11.setLatitude(33.644772); l11.setLongitude(-117.843362); waypoints.add(l11);
        Location l12 = new Location(""); l12.setLatitude(33.644906); l12.setLongitude(-117.843658); waypoints.add(l12);
*/

        // Cardiff CA 92007 - Somerset to Sheffield to Oxford to Somerset
/*        Location l0 = new Location(""); l0.setLatitude(33.027226); l0.setLongitude(-117.279064); waypoints.add(l0);
        Location l1 = new Location(""); l1.setLatitude(33.027721); l1.setLongitude(-117.279075); waypoints.add(l1);
        Location l2 = new Location(""); l2.setLatitude(33.027991); l2.setLongitude(-117.279558); waypoints.add(l2);
        Location l3 = new Location(""); l3.setLatitude(33.027874); l3.setLongitude(-117.280062); waypoints.add(l3);
        Location l4 = new Location(""); l4.setLatitude(33.027406); l4.setLongitude(-117.280126); waypoints.add(l4);
        Location l5 = new Location(""); l5.setLatitude(33.026875); l5.setLongitude(-117.280158); waypoints.add(l5);
        Location l6 = new Location(""); l6.setLatitude(33.026426); l6.setLongitude(-117.279997); waypoints.add(l6);
        Location l7 = new Location(""); l7.setLatitude(33.026354); l7.setLongitude(-117.279364); waypoints.add(l7);
        Location l8 = new Location(""); l8.setLatitude(33.026803); l8.setLongitude(-117.279086); waypoints.add(l8);*/

        // Encinitas Community park - lower parking lot to Skate Park
        Location l0 = new Location(""); l0.setLatitude(33.033778); l0.setLongitude(-117.281151); waypoints.add(l0);
        Location l1 = new Location(""); l1.setLatitude(33.034045); l1.setLongitude(-117.281410); waypoints.add(l1);
        Location l2 = new Location(""); l2.setLatitude(33.034188); l2.setLongitude(-117.281804); waypoints.add(l2);
        Location l3 = new Location(""); l3.setLatitude(33.034082); l3.setLongitude(-117.282002); waypoints.add(l3);
        Location l4 = new Location(""); l4.setLatitude(33.033813); l4.setLongitude(-117.281982); waypoints.add(l4);
        Location l5 = new Location(""); l5.setLatitude(33.033504); l5.setLongitude(-117.281880); waypoints.add(l5);
        Location l6 = new Location(""); l6.setLatitude(33.033204); l6.setLongitude(-117.281558); waypoints.add(l6);
        Location l7 = new Location(""); l7.setLatitude(33.033183); l7.setLongitude(-117.281233); waypoints.add(l7);
        Location l8 = new Location(""); l8.setLatitude(33.033298); l8.setLongitude(-117.280955); waypoints.add(l8);
        Location l9 = new Location(""); l9.setLatitude(33.033516); l9.setLongitude(-117.280965); waypoints.add(l9);

        // Set parameters for serotonin based Bayesian decision model
        pWait = new  double[T];
        randNumGen = new Random();
        skipWayPt = false;

        if (pRwd  < 0.51) {
            // Probability of waiting for a 50% reward to be delivered 40+/-20
            pWait[0] = 1;
            pWait[1] = 1;
            pWait[2] = 1;
            pWait[3] = 1;
            pWait[4] = 1;
            pWait[5] = 1;
            pWait[6] = 1;
            pWait[7] = 1;
            pWait[8] = 1;
            pWait[9] = 1;
            pWait[10] = 1;
            pWait[11] = 1;
            pWait[12] = 1;
            pWait[13] = 1;
            pWait[14] = 1;
            pWait[15] = 1;
            pWait[16] = 1;
            pWait[17] = 1;
            pWait[18] = 1;
            pWait[19] = 1;
            pWait[20] = 1;
            pWait[21] = 1;
            pWait[22] = 1;
            pWait[23] = 1;
            pWait[24] = 1;
            pWait[25] = 1;
            pWait[26] = 1;
            pWait[27] = 1;
            pWait[28] = 1;
            pWait[29] = 1;
            pWait[30] = 1;
            pWait[31] = 1;
            pWait[32] = 1;
            pWait[33] = 1;
            pWait[34] = 1;
            pWait[35] = 1;
            pWait[36] = 1;
            pWait[37] = 1;
            pWait[38] = 1;
            pWait[39] = 1;
            pWait[40] = 1;
            pWait[41] = 1;
            pWait[42] = 1;
            pWait[43] = 1;
            pWait[44] = 1;
            pWait[45] = 1;
            pWait[46] = 1;
            pWait[47] = 0.99999;
            pWait[48] = 0.99999;
            pWait[49] = 0.99998;
            pWait[50] = 0.99997;
            pWait[51] = 0.99995;
            pWait[52] = 0.99991;
            pWait[53] = 0.99985;
            pWait[54] = 0.99976;
            pWait[55] = 0.9996;
            pWait[56] = 0.99934;
            pWait[57] = 0.99891;
            pWait[58] = 0.99824;
            pWait[59] = 0.99718;
            pWait[60] = 0.99553;
            pWait[61] = 0.993;
            pWait[62] = 0.98919;
            pWait[63] = 0.98354;
            pWait[64] = 0.97533;
            pWait[65] = 0.96365;
            pWait[66] = 0.9474;
            pWait[67] = 0.92534;
            pWait[68] = 0.89616;
            pWait[69] = 0.85866;
            pWait[70] = 0.81196;
            pWait[71] = 0.75569;
            pWait[72] = 0.69032;
            pWait[73] = 0.6172;
            pWait[74] = 0.53869;
            pWait[75] = 0.45786;
            pWait[76] = 0.3782;
            pWait[77] = 0.30313;
            pWait[78] = 0.23549;
            pWait[79] = 0.17722;
            pWait[80] = 0.12918;
            pWait[81] = 0.09124;
            pWait[82] = 0.062495;
            pWait[83] = 0.041559;
            pWait[84] = 0.026869;
            pWait[85] = 0.016917;
            pWait[86] = 0.01039;
            pWait[87] = 0.0062369;
            pWait[88] = 0.0036656;
            pWait[89] = 0.0021134;
            pWait[90] = 0.0011974;
            pWait[91] = 0.00066781;
            pWait[92] = 0.00036726;
            pWait[93] = 0.00019945;
            pWait[94] = 0.00010712;
            pWait[95] = 5.6967e-05;
            pWait[96] = 3.0035e-05;
            pWait[97] = 1.5716e-05;
            pWait[98] = 8.1696e-06;
            pWait[99] = 4.2224e-06;
            pWait[100] = 2.1715e-06;
            pWait[101] = 1.112e-06;
            pWait[102] = 5.6733e-07;
            pWait[103] = 2.8854e-07;
            pWait[104] = 1.4635e-07;
            pWait[105] = 7.4058e-08;
            pWait[106] = 3.7403e-08;
            pWait[107] = 1.8859e-08;
            pWait[108] = 9.4955e-09;
            pWait[109] = 4.7753e-09;
            pWait[110] = 2.3992e-09;
            pWait[111] = 1.2044e-09;
            pWait[112] = 6.0415e-10;
            pWait[113] = 3.0289e-10;
            pWait[114] = 1.5178e-10;
            pWait[115] = 7.6027e-11;
            pWait[116] = 3.807e-11;
            pWait[117] = 1.9058e-11;
            pWait[118] = 9.5381e-12;
            pWait[119] = 4.7728e-12;
            pWait[120] = 2.388e-12;
            pWait[121] = 1.1946e-12;
            pWait[122] = 5.9754e-13;
            pWait[123] = 2.9887e-13;
            pWait[124] = 1.4948e-13;
            pWait[125] = 7.4754e-14;
            pWait[126] = 3.7383e-14;
            pWait[127] = 1.8694e-14;
            pWait[128] = 9.3481e-15;
            pWait[129] = 4.6744e-15;
            pWait[130] = 2.3374e-15;
            pWait[131] = 1.1688e-15;
            pWait[132] = 5.844e-16;
            pWait[133] = 2.9221e-16;
            pWait[134] = 1.4611e-16;
            pWait[135] = 7.3056e-17;
            pWait[136] = 3.6528e-17;
            pWait[137] = 1.8264e-17;
            pWait[138] = 9.1323e-18;
            pWait[139] = 4.5662e-18;
            pWait[140] = 2.2831e-18;
            pWait[141] = 1.1416e-18;
            pWait[142] = 5.7078e-19;
            pWait[143] = 2.8539e-19;
            pWait[144] = 1.427e-19;
            pWait[145] = 7.1348e-20;
            pWait[146] = 3.5674e-20;
            pWait[147] = 1.7837e-20;
            pWait[148] = 8.9185e-21;
            pWait[149] = 4.4593e-21;
        }
        else {
            // Probability of waiting for a 95% reward to be delivered 40+/-20
            pWait[0] = 1;
            pWait[1] = 1;
            pWait[2] = 1;
            pWait[3] = 1;
            pWait[4] = 1;
            pWait[5] = 1;
            pWait[6] = 1;
            pWait[7] = 1;
            pWait[8] = 1;
            pWait[9] = 1;
            pWait[10] = 1;
            pWait[11] = 1;
            pWait[12] = 1;
            pWait[13] = 1;
            pWait[14] = 1;
            pWait[15] = 1;
            pWait[16] = 1;
            pWait[17] = 1;
            pWait[18] = 1;
            pWait[19] = 1;
            pWait[20] = 1;
            pWait[21] = 1;
            pWait[22] = 1;
            pWait[23] = 1;
            pWait[24] = 1;
            pWait[25] = 1;
            pWait[26] = 1;
            pWait[27] = 1;
            pWait[28] = 1;
            pWait[29] = 1;
            pWait[30] = 1;
            pWait[31] = 1;
            pWait[32] = 1;
            pWait[33] = 1;
            pWait[34] = 1;
            pWait[35] = 1;
            pWait[36] = 1;
            pWait[37] = 1;
            pWait[38] = 1;
            pWait[39] = 1;
            pWait[40] = 1;
            pWait[41] = 1;
            pWait[42] = 1;
            pWait[43] = 1;
            pWait[44] = 1;
            pWait[45] = 1;
            pWait[46] = 1;
            pWait[47] = 1;
            pWait[48] = 1;
            pWait[49] = 1;
            pWait[50] = 1;
            pWait[51] = 1;
            pWait[52] = 1;
            pWait[53] = 1;
            pWait[54] = 1;
            pWait[55] = 1;
            pWait[56] = 1;
            pWait[57] = 1;
            pWait[58] = 1;
            pWait[59] = 1;
            pWait[60] = 1;
            pWait[61] = 1;
            pWait[62] = 1;
            pWait[63] = 1;
            pWait[64] = 1;
            pWait[65] = 1;
            pWait[66] = 1;
            pWait[67] = 1;
            pWait[68] = 1;
            pWait[69] = 1;
            pWait[70] = 1;
            pWait[71] = 1;
            pWait[72] = 1;
            pWait[73] = 1;
            pWait[74] = 1;
            pWait[75] = 1;
            pWait[76] = 1;
            pWait[77] = 1;
            pWait[78] = 1;
            pWait[79] = 1;
            pWait[80] = 1;
            pWait[81] = 1;
            pWait[82] = 0.99999;
            pWait[83] = 0.99996;
            pWait[84] = 0.99988;
            pWait[85] = 0.99966;
            pWait[86] = 0.99915;
            pWait[87] = 0.99797;
            pWait[88] = 0.99544;
            pWait[89] = 0.99038;
            pWait[90] = 0.98092;
            pWait[91] = 0.96441;
            pWait[92] = 0.93753;
            pWait[93] = 0.89673;
            pWait[94] = 0.83915;
            pWait[95] = 0.76371;
            pWait[96] = 0.67218;
            pWait[97] = 0.56939;
            pWait[98] = 0.46253;
            pWait[99] = 0.35955;
            pWait[100] = 0.2673;
            pWait[101] = 0.19018;
            pWait[102] = 0.12973;
            pWait[103] = 0.085071;
            pWait[104] = 0.053793;
            pWait[105] = 0.032913;
            pWait[106] = 0.019554;
            pWait[107] = 0.011319;
            pWait[108] = 0.0064044;
            pWait[109] = 0.0035531;
            pWait[110] = 0.001938;
            pWait[111] = 0.0010419;
            pWait[112] = 0.00055327;
            pWait[113] = 0.00029076;
            pWait[114] = 0.00015147;
            pWait[115] = 7.8334e-05;
            pWait[116] = 4.0264e-05;
            pWait[117] = 2.0592e-05;
            pWait[118] = 1.0487e-05;
            pWait[119] = 5.3222e-06;
            pWait[120] = 2.6934e-06;
            pWait[121] = 1.3599e-06;
            pWait[122] = 6.8533e-07;
            pWait[123] = 3.4484e-07;
            pWait[124] = 1.7329e-07;
            pWait[125] = 8.6998e-08;
            pWait[126] = 4.364e-08;
            pWait[127] = 2.1876e-08;
            pWait[128] = 1.096e-08;
            pWait[129] = 5.489e-09;
            pWait[130] = 2.748e-09;
            pWait[131] = 1.3754e-09;
            pWait[132] = 6.8823e-10;
            pWait[133] = 3.4433e-10;
            pWait[134] = 1.7225e-10;
            pWait[135] = 8.6156e-11;
            pWait[136] = 4.3091e-11;
            pWait[137] = 2.155e-11;
            pWait[138] = 1.0777e-11;
            pWait[139] = 5.3892e-12;
            pWait[140] = 2.6949e-12;
            pWait[141] = 1.3476e-12;
            pWait[142] = 6.7382e-13;
            pWait[143] = 3.3693e-13;
            pWait[144] = 1.6847e-13;
            pWait[145] = 8.4237e-14;
            pWait[146] = 4.2119e-14;
            pWait[147] = 2.106e-14;
            pWait[148] = 1.053e-14;
            pWait[149] = 5.2651e-15;
        }

 /*       for (int i = 0; i< T; i++) {
            Log.d("lineFollow", "pWait(" + i + ") = "  + pWait[i]);
        }*/

         criteria = new Criteria();
         criteria.setAccuracy(Criteria.ACCURACY_FINE);

        dest_loc = waypoints.get(0);

        //set up location listener
        mLocationListener = new LocationListener() {
            public void onLocationChanged(Location location) {

                Context context = getApplicationContext();
                CharSequence text;
                Toast toast;
                ToneGenerator toneG;

                // This code gets the declination which is about:
                //      11.665 in Irvine
                //      11.435 in Cardiff
/*                GeomagneticField geoField = new GeomagneticField(
                        Double.valueOf(location.getLatitude()).floatValue(),
                        Double.valueOf(location.getLongitude()).floatValue(),
                        Double.valueOf(location.getAltitude()).floatValue(),
                        System.currentTimeMillis()
                );
                Log.d("decline", "Declination = " + geoField.getDeclination());*/

                curr_loc = location;
                curr_lat = curr_loc.getLatitude();
                curr_lon = curr_loc.getLongitude();
//                Log.d("lineFollow", "Curr = ("+ curr_lat + ", " + curr_lon + ")");

                if (autoMode) {
                    int timeElapsed = (int) ((System.currentTimeMillis() - timeWayPtStart) / 1000);

                    if (reachedLastWayPt) {
                        toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
                        toneG.startTone(ToneGenerator.TONE_CDMA_ABBR_ALERT, 500);

                        try {
                            fosRR.close();
                        } catch (IOException e) {
                            Log.e("lineFollow", e.toString());
                        }
                    } else {
                        // skipWayPt = true; // JLK temporary for testing
                        if (curr_loc.distanceTo(waypoints.get(locInx)) < 20) {
                            context = getApplicationContext();
                            text = "Reached WayPoint" + locInx + " in " + timeElapsed + " secs";
                            toast = Toast.makeText(context, text, Toast.LENGTH_SHORT);
                            toast.show();
                            toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
                            toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 200);
                            ++locInx;
                            if (locInx >= waypoints.size()) {
                                reachedLastWayPt = true;
                                reward = 10;
                            } else {
                                dest_loc = waypoints.get(locInx);
                                timeWayPtStart = System.currentTimeMillis();
                                skipWayPt = false;
                                reachedWayPt = true;
                                if (randNumGen.nextDouble() < pRwd) {
                                    reward = 1;
                                }
                                else {
                                    reward = 0;
                                }
                            }
                            String mill_timestamp = System.currentTimeMillis() + "";
                            String info = mill_timestamp + "," + curr_loc.getLatitude() + "," + curr_loc.getLongitude() + "," + locInx + "," + reward + "\n";
                            try {
                                byte[] b = info.getBytes();
                                fosRR.write(b);
                                Log.i("test", "wrote");
                            } catch (IOException e) {
                                Log.e("lineFollow", e.toString());
                            }
                        } else {
                            reachedWayPt = false;
                            // At least one second has elapsed, update pWait
                            if (timeElapsed > 0) {

                                Log.d("lineFollow", "WayPoint" + locInx + " timeElapsed = " + timeElapsed);

                                // If not skipping a waypoint or heading toward last waypoint and waited too long, then skip to a random waypoint
                                if (!skipWayPt && locInx < (waypoints.size() - 1) && randNumGen.nextDouble() > pWait[timeElapsed - 1]) {
                                    locInx = randNumGen.nextInt(waypoints.size() - (locInx + 1)) + (locInx + 1);
                                    context = getApplicationContext();
                                    text = "Skipping to WayPoint" + locInx;
                                    toast = Toast.makeText(context, text, Toast.LENGTH_SHORT);
                                    toast.show();
                                    toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
                                    toneG.startTone(ToneGenerator.TONE_CDMA_ABBR_INTERCEPT, 200);
                                    dest_loc = waypoints.get(locInx);
                                    timeWayPtStart = System.currentTimeMillis();
                                    skipWayPt = true;
                                    String mill_timestamp = System.currentTimeMillis() + "";
                                    String info = mill_timestamp + "," + curr_loc.getLatitude() + "," + curr_loc.getLongitude() + "," + locInx + ",-1\n";
                                    try {
                                        byte[] b = info.getBytes();
                                        fosRR.write(b);
                                        Log.i("test", "wrote");
                                    } catch (IOException e) {
                                        Log.e("lineFollow", e.toString());
                                    }
                                }
                            }
                        }
                    }
                }

                distance = location.distanceTo(dest_loc);
                bearing = location.bearingTo(dest_loc);
                if (bearing < 0) {
                    bearing += 360.0;
                }
                setText("distance: " + distance, distanceText);
                setText("bearing: " + bearing, bearingText);
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
        mLocationRequest.setInterval(1000);
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
            // heading = values[0] + 11.665f; // from geoField.getDeclination() in Irvine
            heading = values[0] + 11.435f; // from geoField.getDeclination() in Cardiff
            if (heading < 0) {
                heading += 360.0;
            }
//            heading = (heading * 5 + fixWraparound(values[0] + 12)) / 6; //add 12 to make up for declination in Irvine, average out from previous 2 for smoothness
        }
        setText("heading: " + heading, headingText);
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
            String info = text +" ,Lat:"+curr_lat+" ,Lon:"+curr_lon+" ,Time:"+time;
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
                        Log.e("app.main.main", "Couldn't write to SD");
                    }
                } catch (Exception ex) {
                    Log.e("app.main.main", "Couldn't write to SD");
                }
            } catch (Exception e) {
                Log.e("app.main.main", "Couldn't write to SD");
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
        Log.i("activity cycle", "main.main activity restarting");
    }

    public void onCameraViewStarted(int width, int height) {
        smallSize = new org.opencv.core.Size(width/10,height/10);
        origSize = new org.opencv.core.Size(width, height);
//		origSize = new org.opencv.core.Size(1920, 1080);
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mRgbaNorm = new Mat(height, width, CvType.CV_8UC4);
        mAttnMap = new Mat(origSize, CvType.CV_8UC1);
        channels = new ArrayList(4);
        mAttnMapSmall = new Mat(smallSize, CvType.CV_32FC1);
        mTmp1 = new Mat(origSize, CvType.CV_32FC1);
        mTmp2 = new Mat(origSize, CvType.CV_32FC1);
        mImgThr = new Mat(origSize, CvType.CV_8UC1);
        mImgThrPos = new Mat(origSize, CvType.CV_8UC1);
        mImgThrNeg = new Mat(origSize, CvType.CV_8UC1);
        mThrMask = new Mat(origSize, CvType.CV_8UC1);
        mThrMaskNot = new Mat(origSize, CvType.CV_8UC1);
        mImgGray = new Mat(origSize, CvType.CV_8UC1);
        mImgGraySmallSize = new Mat(origSize, CvType.CV_8UC1);
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
        boolean foundLine = false;
        mRgba = inputFrame.rgba();
        mImgThrPos.setTo(new Scalar(0));
        mImgThrNeg.setTo(new Scalar(0));
        mAttnMap.setTo(new Scalar(255));
        MatOfPoint maxContour = new MatOfPoint();
        Mat lines = new Mat();
        int threshold = 50;
        int minLineSize = 50;
        int lineGap = 10;
        double lineCnt = 0.0;
        double lineStartX = 0.0;
        double lineEndX = 0.0;
        double lineStartY = 0.0;
        double lineEndY = 0.0;

 /*       Imgproc.cvtColor(mRgba, mImgGray, Imgproc.COLOR_RGB2GRAY, 4);
        Imgproc.medianBlur(mImgGray,mImgGray,35);
        Imgproc.adaptiveThreshold(mImgGray,mImgGray,255,Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C,Imgproc.THRESH_BINARY,11,2);
        Imgproc.Canny(mImgGray, mImgGray, 10, 25);
        Imgproc.dilate(mImgGray, mImgGray, new Mat());
        foundLine = false;

        if (autoMode || contourDisplayed) {
            double ang = Math.PI/180.0;
            Imgproc.HoughLinesP(mImgGray, lines, 1, ang, threshold, minLineSize, lineGap);
            for (int x = 0; x < lines.rows(); x++) {
                double[] vec = lines.get(x, 0);
                double xStart, xEnd, yStart, yEnd;

                if (vec[1] > vec[3]) {
                    xStart = vec[0];
                    yStart = vec[1];
                    xEnd = vec[2];
                    yEnd = vec[3];
                }
                else {
                    xStart = vec[2];
                    yStart = vec[3];
                    xEnd = vec[0];
                    yEnd = vec[1];
                }

                // double dist = Math.sqrt(Math.pow(x1 - x2, 2.0) + Math.pow(y1 - y2, 2.0));

                if (yStart > 750 && (yStart-yEnd) > 250) {
                    ++lineCnt;
                    lineStartX += xStart;
                    lineEndX += xEnd;
                    lineStartY += yStart;
                    lineEndY += yEnd;
                    Point start = new Point(xStart, yStart);
                    Point end = new Point(xEnd, yEnd);
                    Imgproc.line(mRgba, start, end, new Scalar(0, 255, 0), 5);
                    //Log.d("lineFollow", x + ": start=" + start + " end=" + end);

                }
            }

            if (lineCnt > 0.5) {
                foundLine = true;
                lineStartX = lineStartX / lineCnt;
                lineStartY = lineStartY / lineCnt;
                lineEndX = lineEndX / lineCnt;
                lineEndY = lineEndY / lineCnt;
                Point start = new Point(lineStartX, lineStartY);
                Point end = new Point(lineEndX, lineEndY);
                Imgproc.line(mRgba, start, end, new Scalar(255, 0, 0), 10);
                // Log.d("lineFollow", "start=" + start + " end=" + end);
            }
        }*/

        // JLK for testing purposes without the robot
/*        if (Math.abs(bearing-heading) < 20.0 || Math.abs(bearing-heading) > 340) {
            setText("No Turn", sonar1Text);
        }
        else if (heading > bearing) {
            if ((heading - bearing) < 180.0) {
                setText("Left H>B", sonar1Text);
            }
            else {
                setText("Right H>B", sonar1Text);
            }
        }
        else { // heading < bearing
            if ((bearing - heading) < 180.0) {
                setText("Right B>H", sonar1Text);
            }
            else {
                setText("Left B>H", sonar1Text);
            }
        }*/

        if (autoMode) { // only move if autoMode is on

            int sonar1 = m_ioio_thread.get_sonar1_reading();
            int sonar2 = m_ioio_thread.get_sonar2_reading();
            int sonar3 = m_ioio_thread.get_sonar3_reading();

            m_ioio_thread.move(forward_fast);

            if (reachedWayPt || reachedLastWayPt){
                m_ioio_thread.move(forward_stop);
                m_ioio_thread.turn(turn_none);
            }
            // If middle sonar close, stop and wait
            else if (sonar2 < 10){
                m_ioio_thread.move(forward_stop);
                m_ioio_thread.turn(turn_none);
            }
            // If left sonar close, turn right
            else if (sonar1 < 10){
                m_ioio_thread.move(forward_slow);
                m_ioio_thread.turn(turn_right_sharp);
            }
            // If right sonar close, turn left
            else if (sonar3 < 10){
                m_ioio_thread.move(forward_slow);
                m_ioio_thread.turn(turn_left_sharp);
            }
            // go to way point or follow road...
            else {
                if (Math.abs(bearing-heading) < 10.0 || Math.abs(bearing-heading) > 350) {
                    m_ioio_thread.turn(turn_none);
                }
                else if (heading > bearing) {
                    if ((heading - bearing) < 180.0) {
                        m_ioio_thread.turn(turn_left);
                    }
                    else {
                        m_ioio_thread.turn(turn_right);
                    }
                }
                else { // heading < bearing
                    if ((bearing - heading) < 180.0) {
                        m_ioio_thread.turn(turn_right);
                    }
                    else {
                        m_ioio_thread.turn(turn_left);
                    }
                }

 /*               if (foundLine) {
                    if (lineStartX < (mRgba.width()/2.0-200.0)) {
                        m_ioio_thread.turn(turn_left);
                        Log.d("lineFollow", "Turn Left: w=" + mRgba.width() + " x=" + lineStartX);
                    }
                    else if (lineStartX > (mRgba.width()/2.0+200.0)) {
                        m_ioio_thread.turn(turn_right);
                        Log.d("lineFollow", "Turn Right: w=" + mRgba.width() + " x=" + lineStartX);
                    }
                    else {
                        m_ioio_thread.turn(turn_none);
                        Log.d("lineFollow", "No Turn: w=" + mRgba.width() + " x=" + lineStartX);
                    }
                } else  {
                    m_ioio_thread.turn(turn_none);
                    Log.d("lineFollow", "No Line");
                }*/
            }
        }
        return mRgba;

 /*       if (autoMode || contourDisplayed) {
            return mRgba;
        }
        else {
            return mImgGray;
        }*/
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
        Log.i("activity cycle","main.main activity being destroyed");
        helper_.destroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    protected void onStart() {
        super.onStart();
        Log.i("activity cycle","main.main activity starting");
        helper_.start();
        mGoogleApiClient.connect();
    }

    @Override
    protected void onStop() {
        Log.i("activity cycle","main.main activity stopping");
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
	


