package org.usfirst.frc.team1806.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.util.concurrent.TimeoutException;

import org.usfirst.frc.team1806.robot.RobotStates.VisionTrackingState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JetsonReceiver extends Thread {

	DatagramSocket ds;
	DatagramPacket dp;
	ByteBuffer bb;

	String answer;
	String serverAddress = "1234";
	// Socket s;
	BufferedReader input;

	int isGoal;
	double distance;
	double angle;

	double then;
	double now;
	double deltaTime;

	public JetsonReceiver() {
		try {

			ds = new DatagramSocket(5800);
			bb = ByteBuffer.allocate(20);
			// s = new Socket(serverAddress, 9090);
			// input = new BufferedReader(new
			// InputStreamReader(s.getInputStream()));
		} catch (Exception e) {
			System.out.println("Error initializing client: " + e);
		}

		now = System.currentTimeMillis();
		then = System.currentTimeMillis();

	}

	public void run() {

		try {
			ds.setSoTimeout((int) Constants.jetsonConnectionLostTimeout * 1000);
		} catch (SocketException se) {
			se.printStackTrace();
		}

		while (true) {
			try {

				//System.out.println("Beginning to listening for packet...");
				bb.position(0);
				dp = new DatagramPacket(bb.array(), bb.capacity());
				ds.receive(dp);
				isGoal = bb.getInt();
				distance = bb.getDouble();
				angle = bb.getDouble();
				
				if(angle > 180){
					angle = angle - 360;
				}
				
				angle = angle + Constants.ShootingJetsonCameraAngleOffset;
				
				//angle = -angle;
				// System.out.println("Goal found? : " + isGoal);
				// System.out.println("Distance from goal: " + distance);
				// System.out.println("angle from goal: " + angle);

				SmartDashboard.putNumber("Goal found", isGoal);
				SmartDashboard.putNumber("Distance from goal", distance / 12);
				SmartDashboard.putNumber("ANgle from goal", angle);

				// System.out.println("run successfully");
				then = now;
				now = System.currentTimeMillis();
				deltaTime = now - then;

				//System.out.println("Time since last packet: " + deltaTime);

				// If this try block completes all the way to here, it means you
				// had no errors and you successfully received
				// data from the jetson. So that means you should switch back to
				// using image data from the Jetson if you were't already :-)
				Robot.states.visionTrackingStateTracker = VisionTrackingState.JETSON;

			} catch (Exception e) {
				if (e instanceof SocketTimeoutException) {
					// Failed to receive socket in time.
					Robot.states.visionTrackingStateTracker = VisionTrackingState.ROBORIO;
				} else if (e instanceof IOException) {
					e.printStackTrace();
				}
			}
		}
	}

	public double getAngleToGoal() {
		return angle;
	}
	
	public boolean isAngleAcceptable(){
		//Is the angle within the range specified in Constants?
		return Math.abs(angle) < Constants.ShootingmaxAngleError;
	}
	public boolean isDistanceAcceptable(){
		//Is the distance within range specified in Constants?
		return ( distance > Constants.ShootingMinGoalDistance && distance < Constants.ShootingMaxGoalDistance);
	}
	
	public double getDistanceFromGoal(){
		return distance;
	}

	public boolean isGoalFound() {
		if (isGoal == 1) {
			return true;
		} else {
			return false;
		}
	}

	public double getTimeSinceLastPacketReceived() {
		return System.currentTimeMillis() - then;
	}

}
