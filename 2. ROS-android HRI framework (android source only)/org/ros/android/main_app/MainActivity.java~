/*
 * Copyright (C) 2017 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 * 
 * Developed by Sami Salama Hajjaj, and, Ismail Ibrahim Al Mahdi 
 * As the android components of the ROS-android HRI framework
 * As a component of the AgriWorld Framework
 * As part of PhD studies of Mr. Hajjaj  
 */

package org.ros.android.main_app;

import android.animation.AnimatorSet;
import android.animation.ObjectAnimator;
import android.app.Dialog;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.net.Uri;
import android.os.Bundle;
import android.view.View;
import android.support.v4.app.Fragment;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.Window;
import android.view.WindowManager;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;

import android.os.Bundle;
import android.support.v4.app.FragmentActivity;

import com.google.android.gms.appindexing.Action;
import com.google.android.gms.appindexing.AppIndex;
import com.google.android.gms.appindexing.Thing;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.maps.CameraUpdate;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.MapFragment;
import com.google.android.gms.maps.MapView;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.GooglePlayServicesUtil;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.android.view.RosTextView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.lang.String;
import java.net.URI;

import nav_msgs.Odometry;
import sensor_msgs.CompressedImage;
import sensor_msgs.NavSatFix;
import smart_battery_msgs.SmartBatteryStatus;
import std_msgs.Int8;

public class MainActivity extends RosActivity {

    // Main Executor and nodeConfig for this App
    private NodeMainExecutor nodeMainExecutor;
    private NodeConfiguration nodeConfiguration;

    // variables

    private static final int ERROR_DIALOG_REQUEST = 9001;
    private String robotJob_status;
    private int command_id, robotJob_status_id = 0; // initial job status = 0 = idle
    private double robotLatitude, robotLongitude;
    // flags
    private boolean isControls;
    private boolean isLive;
    private boolean isShowSpeed;
    private boolean isShowBattery;
    private boolean isMap;


    // GUI items

    private RelativeLayout menu;
    private RelativeLayout mainPage;

    private ImageButton speedButton;
    private LinearLayout speedBar;
    private ProgressBar speedProgPs;
    private ProgressBar speedProgNs;
    private TextView speedProgLabel;

    private ImageButton batteryButton;
    private ImageButton batteryProg;
    private LinearLayout batteryGroup;
    private TextView batteryProgLabel;

    private ImageButton liveFeedButton;

    private ImageButton ControlsButton;

    private ImageButton MapButton;
    private GoogleMap mMap;
    private RelativeLayout mapWrapper;

    private ImageButton menuXbutton;
    private ImageButton menuButton;


    private ImageButton LivefeedButtonSmall;
    private ImageButton MapButtonSmall;
    private ImageButton ControlsButtonSmall;
    private ImageButton BatteryButtonSmall;
    private ImageButton SpeedButtonSmall;

    private ImageButton ControlGoHomeButton;
    private ImageButton ControlStopButton;
    private ImageButton ControlResumeButton;
    private ImageButton ControlWorkButton;
    private LinearLayout ControlGroup;

    private double
            robot_lat = 2.974208,
            robot_lng = 101.728672;


    // ROS PROPERTIES
    // RosTextView Listeners
    private RosTextView<Int8> robotJobStatusListener;      // jobStatus Listener
    private RosTextView<NavSatFix> gpsListener;         // GPS listener

    // battery listener
    private RosTextView<SmartBatteryStatus> batteryInfoView;
    // topic name : /laptop_charge
    // message type smart_battery_msgs/SmartBatteryStatus

    // speed listener
    private RosTextView<Odometry> speedInfoView;
    // liveFeed listener
    private RosImageView<CompressedImage> liveCamView;


    // Publishers (so far, only one publisher is needed)
    private robotCommandPublisher robotCommandPublisher;
    /**
     * ATTENTION: This was auto-generated to implement the App Indexing API.
     * See https://g.co/AppIndexing/AndroidStudio for more information.
     */



// ----- App Initiations --------------

    // Construct to pre-populate Notification & Ticker bars
    public MainActivity() {
        super("ROS-android HRI", "ROS-android HRI"); // , URI.create("http://172.17.32.21:11311")
    }

    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);

        setContentView(R.layout.main);

        // order matters here
        startAndroidGUIs();                                // start any other GUIs for this app
        initListeners();                                   // start rosTextView listeners
        addListenerOnButton();                            // on button click listeners



    }


    // housekeeping, putting all views here (other than RosTextViews)
    public void startAndroidGUIs() {

        // initial flag values
        isControls = false; // hide the controllers views
        isLive = false; // hide the live feed views
        isShowBattery = true; // show the battery views
        isShowSpeed = true; // show the speed views
        isMap = false;


        // speed view
        speedButton = (ImageButton) findViewById(R.id.SpeedButton);
        speedBar = (LinearLayout) findViewById(R.id.speedBar);
        speedProgPs = (ProgressBar) findViewById(R.id.SpeedProgPs);
        speedProgNs = (ProgressBar) findViewById(R.id.SpeedProgNs);
        speedProgLabel = (TextView) findViewById(R.id.SpeedProgLabel);

        // battery view
        batteryButton = (ImageButton) findViewById(R.id.BatteryButton);
        batteryProg = (ImageButton) findViewById(R.id.BatteryProg);
        batteryProgLabel = (TextView) findViewById(R.id.BatteryProgLabel);
        batteryGroup= (LinearLayout) findViewById(R.id.batteryGroup);


        // LiveFeed view
        liveFeedButton = (ImageButton) findViewById(R.id.LiveFeedButton);

        // Commands view
        ControlsButton = (ImageButton) findViewById(R.id.ControlsButton);
        ControlGroup = (LinearLayout) findViewById(R.id.ControlsGroup);

        // map view
        MapButton = (ImageButton) findViewById(R.id.MapButton);
        mapWrapper = (RelativeLayout) findViewById(R.id.mapWrapper);
        if(servicesOK()){
            if(initMap()){
                gotoLocation(robot_lat,robot_lng,19);
            }else {
                Toast.makeText(this,"Map not connected!", Toast.LENGTH_SHORT).show();
            }
        }

        // menu x button
        menuXbutton = (ImageButton) findViewById(R.id.xbutton);

        // show menu button
        menuButton = (ImageButton) findViewById(R.id.level2menu);

        // small  menu bar icons
        LivefeedButtonSmall = (ImageButton) findViewById(R.id.LiveFeedButtonSmall);
        MapButtonSmall = (ImageButton) findViewById(R.id.MapButtonSmall);
        ControlsButtonSmall = (ImageButton) findViewById(R.id.ControlsButtonSmall);
        BatteryButtonSmall = (ImageButton) findViewById(R.id.BatteryButtonSmall);
        SpeedButtonSmall = (ImageButton) findViewById(R.id.SpeedButtonSmall);

        //menu
        menu = (RelativeLayout) findViewById(R.id.menu);
        mainPage = (RelativeLayout) findViewById(R.id.mainPage);


        // Control buttons
        ControlGoHomeButton = (ImageButton) findViewById(R.id.ControlGoHome);
        ControlStopButton = (ImageButton) findViewById(R.id.ControlStop);
        ControlResumeButton = (ImageButton) findViewById(R.id.ControlResume);
        ControlWorkButton= (ImageButton) findViewById(R.id.ControlWork);
        controlsEventListeners();

        // others (add comment for each, specifying name)
    }


    // Start RosTextView Listeners
    public void initListeners() {

      /* ---- For each RosTextView listener -----
        Initiate rosTextView, using given message type
        setTopic              using given topic
        setMessageType        using given message type
        setMessageToStringCallable => define what to do with message once captured
      */

        //  robotJobStatus Listener
        robotJobStatusListener = (RosTextView<Int8>) findViewById(R.id.intMessage);
        robotJobStatusListener.setTopicName("robotJobStatus");
        robotJobStatusListener.setMessageType(Int8._TYPE);
        robotJobStatusListener.setMessageToStringCallable(new MessageCallable<String, Int8>() {
            @Override
            public String call(Int8 msg) {
                robotJob_status_id = msg.getData();
                return updateRobotStatus(robotJob_status_id);
            }
        });

        // GPS position Listener
        gpsListener = (RosTextView<NavSatFix>) findViewById(R.id.gpsMessage);
        gpsListener.setTopicName("fix");
        gpsListener.setMessageType("sensor_msgs/NavSatFix");
        gpsListener.setMessageToStringCallable(new MessageCallable<String, NavSatFix>() {

            @Override
            public String call(NavSatFix message) {

                robotLatitude = message.getLatitude();     // extract coordinates
                robotLongitude = message.getLongitude();
                setMapView(robotLatitude, robotLongitude);  // pass coordinates to MapView

                return "Current Robot Location is: Latitude: " + robotLatitude
                        + ", Longitude: " + robotLongitude; // show message
            }
        });

        // Speed Listener
        speedInfoView = (RosTextView<Odometry>) findViewById(R.id.speedInfo);
        speedInfoView.setTopicName("/odom");
        speedInfoView.setMessageType(Odometry._TYPE);
        speedInfoView.setMessageToStringCallable(new MessageCallable<String, Odometry>() {
            @Override
            public String call(Odometry msg) {
                double speed = msg.getTwist().getTwist().getLinear().getX();
                if (speed != Math.abs(speed)) {// it is -ve
                    speedProgNs.setProgress((int) Math.round(speed * 100) * -1 );// updating the custom bar for the speed
                } else { // it is +ve
                    speedProgPs.setProgress((int) Math.round(speed * 100));// updating the custom bar for the speed
                }
                speedProgLabel.setText(Math.round((speed * 100)) + "cm/s");
                return "";
            }


        });

        // Battery Listener
        batteryInfoView = (RosTextView<SmartBatteryStatus>) findViewById(R.id.batteryInfo);
        batteryInfoView.setTopicName("/laptop_charge"); // CHANGE TOPIC NAME HERE !!
        batteryInfoView.setMessageType(SmartBatteryStatus._TYPE);
        batteryInfoView.setMessageToStringCallable(new MessageCallable<String, SmartBatteryStatus>() {
            @Override
            public String call(SmartBatteryStatus msg) {
                byte ChargeState = msg.getChargeState();

                int percentage = msg.getPercentage();
                batteryProgLabel.setText(percentage + "%");

                if (ChargeState == 0) {
                 if(percentage < 20){
                        batteryProg.setImageResource(R.drawable.battery10pre);
                    }else if(percentage < 60){
                        batteryProg.setImageResource(R.drawable.battery25pre);
                    }else if(percentage < 90){
                        batteryProg.setImageResource(R.drawable.battery50pre);
                    }else if(percentage <= 100){
                        batteryProg.setImageResource(R.drawable.battery100pre);
                    }
                } else {
                    batteryProg.setImageResource(R.drawable.charge);
                }
                return "";
            }

        });

        // LiveFeed Listener (imageListener)
        liveCamView = (RosImageView<CompressedImage>) findViewById(R.id.livecamView);
        liveCamView.setTopicName("camera/rgb/image_color/compressed"); // setting the topic name here
        // camera/rgb/image_color/compressed
        liveCamView.setMessageType(CompressedImage._TYPE);
        liveCamView.setMessageToBitmapCallable(new BitmapFromCompressedImage());

        // other Listeners ..
    }

    public String updateRobotStatus(int robotJob_status_id) {
        switch (robotJob_status_id) {
            case 1:
                robotJob_status = "Robot Status: Go Home";
                break;
            case 2:
                robotJob_status = "Robot Status: STOP!";
                break;
            case 3:
                robotJob_status = "Robot Status: Resume";
                break;
            case 4:
                robotJob_status = "Robot Status: Work";
                break;
            case 5:
                robotJob_status = "Robot Status: Something Else";
                break;
            default:
                robotJob_status = "Robot Status: Idle";
                // do nothing
                // when given an invalid command (only for commandline)
                break;
        }
        return robotJob_status;
    }


    // Start viewing maps
    public void setMapView(double robot_latitude, double robot_longitude) {

        robot_lat = robot_latitude;
        robot_lng= robot_longitude;
        gotoLocation(robot_lat,robot_lng,19);
    }


    // start other views, fragments .. if any

    // ----- Android-to-Robot Interactions --------------

    // regardless of input type, the process should follow these two steps:
    // Step 1 - setOnEventListener, onClick, onVoice, onMovement, etc
    // Step 2 - Process user input, based on input, set command_id
    // Step 3 - Call robotCommandPublisher



    // -------------- Buttons input  -----------------------------

    public void controlsEventListeners(){
        // Step 1 - setOnEventListener
        ControlGoHomeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                // Step 2 - Process user input
                robotJob_status_id = 1;
                ControlGoHomeButton.setImageResource(R.drawable.gohomeblue);
                ControlStopButton.setImageResource(R.drawable.stopblack);
                ControlResumeButton.setImageResource(R.drawable.resumeblack);
                ControlWorkButton.setImageResource(R.drawable.work);
                // Step 3 - Call robotCommandPublisher
                robotCommandPublisher(command_id); // start commandPublisher
            }
        });
        ControlStopButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                // Step 2 - Process user input
                robotJob_status_id = 2;
                ControlGoHomeButton.setImageResource(R.drawable.gohomeblack);
                ControlStopButton.setImageResource(R.drawable.stopblue);
                ControlResumeButton.setImageResource(R.drawable.resumeblack);
                ControlWorkButton.setImageResource(R.drawable.work);
                // Step 3 - Call robotCommandPublisher
                robotCommandPublisher(command_id); // start commandPublisher
            }
        });
        ControlResumeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                // Step 2 - Process user input
                robotJob_status_id = 3;
                ControlGoHomeButton.setImageResource(R.drawable.gohomeblack);
                ControlStopButton.setImageResource(R.drawable.stopblack);
                ControlResumeButton.setImageResource(R.drawable.resumeblue);
                ControlWorkButton.setImageResource(R.drawable.work);
                // Step 3 - Call robotCommandPublisher
                robotCommandPublisher(command_id); // start commandPublisher
            }
        });
        ControlWorkButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                // Step 2 - Process user input
                robotJob_status_id = 4;
                ControlGoHomeButton.setImageResource(R.drawable.gohomeblack);
                ControlStopButton.setImageResource(R.drawable.stopblack);
                ControlResumeButton.setImageResource(R.drawable.resumeblack);
                ControlWorkButton.setImageResource(R.drawable.workblue);
                // Step 3 - Call robotCommandPublisher
                robotCommandPublisher(command_id); // start commandPublisher

            }
        });


    }

    // -------------- Voice Input  -----------------------------

    // Step 1 - OnEventListener
    // Step 2 - Process user input
    // Step 3 - Call robotCommandPublisher

    // -------------- Gesture Input  -----------------------------

    // Step 1 - OnEventListener
    // Step 2 - Process user input
    // Step 3 - Call robotCommandPublisher


    // -------------- Publish Commands to Robot--------------------

    // method to publish robotCommand, using User input
    public void robotCommandPublisher(int command_id) {

        // initial new ROS Publisher (Java Class)
        robotCommandPublisher = new robotCommandPublisher("turtlebotCommands"); // define topic
        robotCommandPublisher.setPublisherID(command_id);   // set command id
        robotCommandPublisher.setPublishOnce(true);         // supress ROS usual behavior and publish just one message

        // start ROS node  (ROS node)
        startNode(robotCommandPublisher, "robotCommandPublisher_android"); // same nodeName to avoid concurrent robotCommands
    }



    public boolean servicesOK() {
        //check if the android deivce have google services

        int isAvailable = GooglePlayServicesUtil.isGooglePlayServicesAvailable(this);

        if (isAvailable == ConnectionResult.SUCCESS) {
            return true;
        } else if (GooglePlayServicesUtil.isUserRecoverableError(isAvailable)) {
            Dialog dialog = GooglePlayServicesUtil.getErrorDialog(isAvailable, this, ERROR_DIALOG_REQUEST);
            dialog.show();
        } else {
            Toast.makeText(this, "Can not connect to mapping service", Toast.LENGTH_SHORT).show();
        }

        return false;
    }

    private boolean initMap(){
        if(mMap == null){
            MapFragment mapFragment = (MapFragment) getFragmentManager().findFragmentById(R.id.map);
            mMap = mapFragment.getMap();

            mMap.setOnMarkerClickListener(new GoogleMap.OnMarkerClickListener() {
                @Override
                public boolean onMarkerClick(Marker marker) {
                    String msg = marker.getTitle() + "  ( " +
                            marker.getPosition().latitude + ", " +
                            marker.getPosition().longitude+ " ) ";
                    Toast.makeText(MainActivity.this,msg,Toast.LENGTH_SHORT).show();
                    return true;
                }
            });
        }
        return (mMap != null);
    }

    private void gotoLocation(double lat, double lng,float zoom){
        LatLng latlng = new LatLng(lat,lng);
        CameraUpdate update = CameraUpdateFactory.newLatLngZoom(latlng,zoom);
        mMap.animateCamera(update);
        mMap.setMapType(GoogleMap.MAP_TYPE_NORMAL);
        MarkerOptions options =new MarkerOptions()
                .title("Robot")
                .position(latlng)
                .icon(BitmapDescriptorFactory.fromResource(R.drawable.ic_action_ball));
        mMap.addMarker(options);
        mMap.getUiSettings().setIndoorLevelPickerEnabled(true);
        mMap.getUiSettings().setZoomGesturesEnabled(false);
        mMap.getUiSettings().setScrollGesturesEnabled(false);
        mMap.getUiSettings().setTiltGesturesEnabled(false);
        mMap.getUiSettings().setRotateGesturesEnabled(false);
    }

    private void addListenerOnButton() { // app Nav

        batteryButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                // fade in and out animation for the battery bar start here..
                if (isShowBattery) {
                    batteryButton.setImageResource(R.drawable.batteryblack);
                    BatteryButtonSmall.setVisibility(View.GONE);
                    batteryGroup.setVisibility(View.GONE);
                } else {
                    batteryButton.setImageResource(R.drawable.batterblue);
                    BatteryButtonSmall.setVisibility(View.VISIBLE);
                    batteryGroup.setVisibility(View.VISIBLE);
                }
                isShowBattery = !isShowBattery; // toggle the flag value
                // fade in and out animation for the battery bar ends here..
            }
        });

        speedButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                // fade in and out animation for the capacity bar start here..
                if (isShowSpeed) {

                    SpeedButtonSmall.setVisibility(View.GONE);
                    speedButton.setImageResource(R.drawable.speedblack);
                    speedBar.setVisibility(View.GONE);
                    speedProgLabel.setVisibility(View.GONE);
//
                } else {
                    speedButton.setImageResource(R.drawable.speedblue);
                    SpeedButtonSmall.setVisibility(View.VISIBLE);
                    speedBar.setVisibility(View.VISIBLE);
                    speedProgLabel.setVisibility(View.VISIBLE);
                }
                isShowSpeed = !isShowSpeed; // toggle the flag value
                // fade in and out animation for the capacity bar ends here..

            }
        });

        ControlsButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //  "Go home" ROS prodcastor can be here ...

                // checking if the robot is going home
                if (isControls) {

                    ControlsButton.setImageResource(R.drawable.controlblack);
                    ControlsButtonSmall.setVisibility(View.GONE);
                    ControlGroup.setVisibility(View.GONE);
                } else {

                    ControlsButton.setImageResource(R.drawable.controlblue);
                    ControlsButtonSmall.setVisibility(View.VISIBLE);
                    ControlGroup.setVisibility(View.VISIBLE);
                }

                isControls = !isControls; // toggle the flag value
            }
        });

        liveFeedButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (isLive) {

                    liveFeedButton.setImageResource(R.drawable.livefeedblack);
                    LivefeedButtonSmall.setVisibility(View.GONE);
                    ObjectAnimator fadeOUT= ObjectAnimator.ofFloat(liveCamView,"Alpha",1f,0f).setDuration(500);
                    fadeOUT.start();

                } else {
                    // the Cam Ros Image View is NOT live ..
                    liveFeedButton.setImageResource(R.drawable.livefeedblue);
                    LivefeedButtonSmall.setVisibility(View.VISIBLE);
                    ObjectAnimator fadeIN= ObjectAnimator.ofFloat(liveCamView,"Alpha",0f,1f).setDuration(500);
                    fadeIN.start();

                }

                isLive = !isLive;// toggle the flag value
            }
        });

        MapButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (isMap) {
                    // the Cam Ros Image View is live ..
                    MapButton.setImageResource(R.drawable.mapblack);
                    MapButtonSmall.setVisibility(View.GONE);
                    ObjectAnimator fadeOUT= ObjectAnimator.ofFloat(mapWrapper,"Alpha",1f,0f).setDuration(500);
                    fadeOUT.start();
                    batteryProgLabel.setTextColor(Color.parseColor("#FFFFFF"));
                    speedProgLabel.setTextColor(Color.parseColor("#FFFFFF"));

                } else {
                    // the Cam Ros Image View is NOT live ..
                    MapButton.setImageResource(R.drawable.mapblue);
                    MapButtonSmall.setVisibility(View.VISIBLE);
                    ObjectAnimator fadeIN= ObjectAnimator.ofFloat(mapWrapper,"Alpha",0f,1f).setDuration(500);
                    fadeIN.start();
                    batteryProgLabel.setTextColor(Color.parseColor("#000000"));
                    speedProgLabel.setTextColor(Color.parseColor("#000000"));

                }
                isMap = !isMap;// toggle the flag value
            }
        });


        // show and hide the menu
        menuXbutton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                ObjectAnimator mOut = ObjectAnimator.ofFloat(menu, "Alpha", 1f, 0f).setDuration(200);
                ObjectAnimator pOut = ObjectAnimator.ofFloat(mainPage, "Alpha", 0f, 1f).setDuration(200);
                AnimatorSet FadeIn = new AnimatorSet();
                FadeIn.playTogether(mOut, pOut);
                FadeIn.start();
                menu.setVisibility(View.GONE);
                mainPage.setVisibility(View.VISIBLE);
            }
        });


        menuButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                ObjectAnimator mOut = ObjectAnimator.ofFloat(menu, "Alpha", 0f, 1f).setDuration(200);
                ObjectAnimator pOut = ObjectAnimator.ofFloat(mainPage, "Alpha", 1f, 0f).setDuration(200);
                AnimatorSet FadeIn = new AnimatorSet();
                FadeIn.playTogether(mOut, pOut);
                FadeIn.start();
                menu.setVisibility(View.VISIBLE);
                mainPage.setVisibility(View.GONE);

            }
        });
        // SmallButtons
        LivefeedButtonSmall.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                isLive= false;
                LivefeedButtonSmall.setVisibility(View.GONE);
                liveFeedButton.setImageResource(R.drawable.livefeedblack);
                ObjectAnimator fadeOUT= ObjectAnimator.ofFloat(liveCamView,"Alpha",1f,0f).setDuration(500);
                fadeOUT.start();



            }
        });

        MapButtonSmall.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                MapButtonSmall.setVisibility(View.GONE);
                MapButton.setImageResource(R.drawable.mapblack);
                ObjectAnimator fadeOUT= ObjectAnimator.ofFloat(mapWrapper,"Alpha",1f,0f).setDuration(500);
                fadeOUT.start();
                batteryProgLabel.setTextColor(Color.parseColor("#FFFFFF"));
                speedProgLabel.setTextColor(Color.parseColor("#FFFFFF"));
                isMap = false;

            }
        });

        ControlsButtonSmall.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                ControlsButtonSmall.setVisibility(View.GONE);
                ControlsButton.setImageResource(R.drawable.controlblack);
                ControlGroup.setVisibility(View.GONE);
                isControls = false;

            }
        });

        BatteryButtonSmall.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                BatteryButtonSmall.setVisibility(View.GONE);
                batteryButton.setImageResource(R.drawable.batteryblack);
                batteryGroup.setVisibility(View.GONE);
                isShowBattery = false;

            }
        });

        SpeedButtonSmall.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                isShowSpeed= false;
                SpeedButtonSmall.setVisibility(View.GONE);
                speedButton.setImageResource(R.drawable.speedblack);
                speedBar.setVisibility(View.GONE);
                speedProgLabel.setVisibility(View.GONE);

            }
        });

    }


    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        // Concurrency issues: for publishers & subscribers to start on demand
        // NodeMaineExecutor ... inherited from RosActivity, must be available throughout app
        // nodeConfiguration ... declared above, but configured here
        // each node must have a unique nodeName (expect for robotCommandPublisher, explained above)

        this.nodeMainExecutor = nodeMainExecutor;
        nodeConfiguration = NodeConfiguration.newPublic(getRosHostname(), getMasterUri());

        // start all ROS listeners (that need to be started on App start)
        // IMPORTANT: Node name must NOT contain spaces
        startNode(robotJobStatusListener, "robotJobStatusListener_android");
        startNode(gpsListener, "robotGPSListener_android");
        // speed Listener
        startNode(batteryInfoView, "string_subscriber");
        // battery Listener
        startNode(speedInfoView, "int_subscriber");
        // liveFeed Listener
        startNode(liveCamView, "image_subscriber");
        // Other Listener

    }

    // Utility method: start ROS node (withing ROS domain) .. be it publisher or subscriber
    protected void startNode(NodeMain node, String nodeName) {
        // Assign new name and Start Node
        nodeConfiguration.setNodeName(nodeName);
        nodeMainExecutor.execute(node, nodeConfiguration);
    }

}
