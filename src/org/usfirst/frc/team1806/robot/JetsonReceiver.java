package org.usfirst.frc.team1806.robot;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Socket;
import java.nio.ByteBuffer;

import org.usfirst.frc.team1806.robot.RobotStates.VisionTrackingState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JetsonReceiver extends Thread {

	DatagramSocket ds;
	DatagramPacket dp;
	ByteBuffer bb;

	String answer;
	String serverAddress = "1234";
	//Socket s;
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
			//s = new Socket(serverAddress, 9090);
			//input = new BufferedReader(new InputStreamReader(s.getInputStream()));
		} catch (Exception e) {
			System.out.println("Error initializing client: " + e);
		}
		
		now = System.currentTimeMillis();
		then = System.currentTimeMillis();
		
	}

	public void run() {

		while (true) {
			try {

				System.out.println("Beginning to listening for packet...");
				bb.position(0);
				dp = new DatagramPacket(bb.array(), bb.capacity());
				ds.receive(dp);
				isGoal = bb.getInt();
				distance = bb.getDouble();
				angle = bb.getDouble();
				//System.out.println("Goal found? : " + isGoal);
				//System.out.println("Distance from goal: " + distance);
				//System.out.println("angle from goal: " + angle);

				SmartDashboard.putNumber("Goal found", isGoal);
				SmartDashboard.putNumber("Distance from goal", distance/12);
				SmartDashboard.putNumber("ANgle from goal", angle);

				//System.out.println("run successfully");
				then = now;
				now = System.currentTimeMillis();
				deltaTime = now - then;
				
				System.out.println("Time since last packet: " + deltaTime);
				
				if(getTimeSinceLastPacketReceived() > Constants.jetsonConnectionLostTimeout){
					Robot.states.visionTrackingStateTracker = VisionTrackingState.ROBORIO;
				}
				
			} catch (Exception e) {
				System.out.println("Error receiving data: " + e);
				// oops
			}
		}

	}
	
	public double getAngleToGoal(){
		return angle;
	}
	
	public boolean isGoalFound(){
		if(isGoal == 1){
			return true;
		}else {
			return false;
		}
	}
	
	public double getTimeSinceLastPacketReceived(){
		return System.currentTimeMillis() - then;
	}
	
}
