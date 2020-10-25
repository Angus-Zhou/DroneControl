package com.example.dronecontroller;

import androidx.appcompat.app.AppCompatActivity;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "MainActivity";

    private static final double radiusToDegreeFactor = 180.0 / Math.PI;

    private static final int telloCommandPort = 8889;
    private static final String telloCommandAddress = "192.168.10.1";
    private static final int telloStatePort = 8890;

    private DatagramSocket telloCommandSocket;
    private DatagramSocket telloStateSocket;

    private long currentTime = System.currentTimeMillis();
    private long lastUpdateTime = currentTime;
    private static final long updateInterval = 50;
    private Thread loopThread;
    private boolean takeoff = false;
    private Thread droneCommandFeedbackListenThread;
    private Thread droneStateListenThread;

    private Button stopButton;
    private Button connectButton;
    private Button imuSwitchButton;
    private TextView droneStateTextView;
    private TextView yawStateTextView;
    private TextView deviceYawStateTextView, deviceRollStateTextView, devicePitchStateTextView;

    private boolean sendLock = false;
    private boolean imuControlOn = false;
    private boolean imuInit = true;

    private SensorManager sensorManager;
    private Sensor rotationVectorSensor;

    private float[] rotationVector;
    private float[] orientation = new float[3];
    private float[] initOrientation = new float[3];
    private float[] relativeOrientation = new float[3];
    private float[] droneOrientation = new float[3];

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        stopButton = findViewById(R.id.takeoff_land_button);
        stopButton.setVisibility(View.INVISIBLE);
        connectButton = findViewById(R.id.connect_button);
        droneStateTextView = findViewById(R.id.drone_state);
        yawStateTextView = findViewById(R.id.yaw_state);
        imuSwitchButton = findViewById(R.id.imu_switch_button);
        imuSwitchButton.setVisibility(View.INVISIBLE);
        deviceYawStateTextView = findViewById(R.id.device_yaw_state);
        devicePitchStateTextView = findViewById(R.id.device_pitch_state);
        deviceRollStateTextView = findViewById(R.id.device_roll_state);

        sensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    }

    @Override
    protected void onResume() {
        super.onResume();

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if (sensorEvent.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            rotationVector = sensorEvent.values;
            float[] rotMat = new float[9];
            SensorManager.getRotationMatrixFromVector(rotMat, rotationVector);
            SensorManager.getOrientation(rotMat, orientation);
            if (imuInit) {
                imuInit = false;
                initOrientation[0] = orientation[0];
            }
            relativeOrientation[0] = (float)((orientation[0] - initOrientation[0]) * radiusToDegreeFactor);
            if (relativeOrientation[0] > 180) {
                relativeOrientation[0] = relativeOrientation[0] - 360;
            } else if (relativeOrientation[0] < -180) {
                relativeOrientation[0] = relativeOrientation[0] + 360;
            }
            deviceYawStateTextView.setText("Device yaw: " + (int)relativeOrientation[0]);
            devicePitchStateTextView.setText("Device pitch: " + orientation[1]);
            deviceRollStateTextView.setText("Device roll: " + orientation[2]);
//            Log.d(TAG, "onSensorChanged: " + orientation[0] + "; " + orientation[1] + "; " + orientation[2]);
        }
    }

    private class RepeatRunnable implements Runnable {
        @Override
        public void run() {
            while(!Thread.currentThread().isInterrupted()) {
                currentTime = System.currentTimeMillis();
                if (currentTime - lastUpdateTime >= updateInterval) {
                    lastUpdateTime = currentTime;
                    Log.d(TAG, "run: my turn");
                    if (!sendLock) {
                        int yawDiff = (int)(relativeOrientation[0] - droneOrientation[0]);
                        if (Math.abs(yawDiff) <= 5) {
                            // do nothing
                        } else {
                            if (yawDiff > 0) {
                                if (yawDiff > 180) {
                                    new Thread(new DroneCommandSendRunnable("ccw " + Integer.toString(yawDiff - 180))).start();
                                } else {
                                    new Thread(new DroneCommandSendRunnable("cw " + Integer.toString(yawDiff))).start();
                                }
                            } else if (yawDiff < 0) {
                                if (yawDiff < -180) {
                                    new Thread(new DroneCommandSendRunnable("cw " + Integer.toString(yawDiff + 360))).start();
                                } else {
                                    new Thread(new DroneCommandSendRunnable("ccw " + Integer.toString(-yawDiff))).start();
                                }
                            }
                        }
                    }
                }
            }
            Log.d(TAG, "run: loopThread interrupted");
        }
    }

    private class DroneCommandListenRunnable implements Runnable {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                byte[] message = new byte[512];
                DatagramPacket packet = new DatagramPacket(message, message.length);
                try {
                    telloCommandSocket.receive(packet);
                    String text = new String(message, 0, packet.getLength());
                    Log.d(TAG, "run: " + text);
                    if (text.equals("ok")) {
                        sendLock = false;
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                droneStateTextView.setText("Ready");
                            }
                        });
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private class DroneCommandSendRunnable implements Runnable {
        final String command;
        public DroneCommandSendRunnable(String command) {
            this.command = command;
        }

        @Override
        public void run() {
            try {
                InetAddress serverAddress = InetAddress.getByName(telloCommandAddress);
                byte[] buf = this.command.getBytes();
                DatagramPacket packet = new DatagramPacket(buf, buf.length, serverAddress, telloCommandPort);
                Log.d(TAG, "droneCommandSendRunnable send: " + command);
                while (sendLock);
                sendLock = true;
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        droneStateTextView.setText("Busy");
                    }
                });
                telloCommandSocket.send(packet);
            } catch (SocketException e) {
                Log.e(TAG, "droneCommandSendRunnable send: " + command, e);
            } catch (IOException e) {
                Log.e(TAG, "droneCommandSendRunnable send: " + command, e);
            }
        }
    }

    private class DroneStateRunnable implements Runnable {
        @Override
        public void run() {
            Log.d(TAG, "run: UDP Server is running -- listening to drone state");

            while (!Thread.currentThread().isInterrupted()) {
                byte[] buf = new byte[256];
                DatagramPacket packet = new DatagramPacket(buf, buf.length);
                Log.d(TAG, "run: UDP Server is running");
                try {
                    telloStateSocket.receive(packet);
                    final String message = new String(buf, 0, packet.getLength());
                    Log.d(TAG, "run: received message: " + message);
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            int begin = message.indexOf("yaw:");
                            int end = message.indexOf(';', begin);
                            droneOrientation[0] = Integer.parseInt(message.substring(begin+4, end));
                            yawStateTextView.setText("yaw: " + droneOrientation[0]);
                        }
                    });
                } catch (IOException e) {
                    Log.e(TAG, "run: tello_state_socket failed to receive", e);
                }
            }
        }
    }

    public void clearSendLock(View view) {
        sendLock = false;
        droneStateTextView.setText("Ready");
    }

    public void connect(View view) {
        try {
            telloCommandSocket = new DatagramSocket(telloCommandPort);
            telloStateSocket = new DatagramSocket(telloStatePort);
            Log.d(TAG, "connect: socket created");
            droneCommandFeedbackListenThread = new Thread(new DroneCommandListenRunnable());
            droneCommandFeedbackListenThread.start();
            new Thread(new DroneCommandSendRunnable("command")).start();
            droneStateListenThread = new Thread(new DroneStateRunnable());
            droneStateListenThread.start();
            connectButton.setText("Disconnect");
            stopButton.setVisibility(View.VISIBLE);
            connectButton.setVisibility(View.INVISIBLE);
        } catch (SocketException e) {
            e.printStackTrace();
            telloCommandSocket.close();
            Log.d(TAG, "connect: socket closed");
            droneCommandFeedbackListenThread.interrupt();
            droneStateListenThread.interrupt();
            telloStateSocket.close();
            connectButton.setText("Connect");
            stopButton.setVisibility(View.INVISIBLE);
        }
    }

    public void clockwiseRotate(View view) {
        new Thread(new DroneCommandSendRunnable("cw 5")).start();
    }

    public void counterclockwiseRotate(View view) {
        new Thread(new DroneCommandSendRunnable("ccw 5")).start();
    }

    public void takeoffLand(View view) {
        if (takeoff) {
            new Thread(new DroneCommandSendRunnable("land")).start();
            stopButton.setText("Takeoff");
            connectButton.setVisibility(View.VISIBLE);
            imuSwitchButton.setVisibility(View.INVISIBLE);
            takeoff = false;
        } else {
            imuSwitchButton.setVisibility(View.VISIBLE);
            new Thread(new DroneCommandSendRunnable("takeoff")).start();
            stopButton.setText("Land");
            connectButton.setVisibility(View.INVISIBLE);
            takeoff = true;
        }
    }

    public void imuControlSwitch(View view) {
        if (imuControlOn) {
            imuControlOn = false;
            imuSwitchButton.setText("Enable IMU Control");
            sensorManager.unregisterListener(this);
            loopThread.interrupt();
        } else {
            imuControlOn = true;
            imuInit = true;
            imuSwitchButton.setText("Disable IMU Control");
            sensorManager.registerListener(this, rotationVectorSensor, SensorManager.SENSOR_DELAY_UI);
            loopThread = new Thread(new RepeatRunnable());
            loopThread.start();
        }
    }

    public void forward(View view) {
        new Thread(new DroneCommandSendRunnable("forward 20")).start();
    }

    public void backward(View view) {
        new Thread(new DroneCommandSendRunnable("back 20")).start();
    }

    public void left(View view) {
        new Thread(new DroneCommandSendRunnable("left 20")).start();
    }

    public void right(View view) {
        new Thread(new DroneCommandSendRunnable("right 20")).start();
    }

    public void stop(View view) {
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    InetAddress serverAddress = InetAddress.getByName(telloCommandAddress);
                    byte[] buf = "stop".getBytes();
                    DatagramPacket packet = new DatagramPacket(buf, buf.length, serverAddress, telloCommandPort);
                    Log.d(TAG, "droneCommandSendRunnable send: " + "stop");
//                    while (sendLock);
//                    sendLock = true;
//                    runOnUiThread(new Runnable() {
//                        @Override
//                        public void run() {
//                            droneStateTextView.setText("Busy");
//                        }
//                    });
                    telloCommandSocket.send(packet);
                } catch (SocketException e) {
                    Log.e(TAG, "droneCommandSendRunnable send: " + "stop", e);
                } catch (IOException e) {
                    Log.e(TAG, "droneCommandSendRunnable send: " + "stop", e);
                }
            }
        }).start();
    }

}