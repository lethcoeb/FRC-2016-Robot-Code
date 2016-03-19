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

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JetsonReceiver extends Thread {

	int isGoal;
	boolean isGoalBool;
	double distance;
	double rawAngle;
	double offsetAngle;
	
	NetworkTable towerTracker;
	
	public JetsonReceiver() {
		towerTracker = NetworkTable.getTable("TowerTracker");
	}

	public void run() {
		
		
		
		while (true) {
			
				isGoal = (int) towerTracker.getNumber("GoalFound", 0);
				distance = towerTracker.getNumber("DistanceToGoal", 0);
				rawAngle = towerTracker.getNumber("AngleToGoal", 0);
				
				if(isGoal == 1){
					isGoalBool = true;
				}else{
					isGoalBool = false;
				}
								
				makeAngleUsable();
				
				
				SmartDashboard.putBoolean("IsGoalFound", isGoalBool);
				SmartDashboard.putNumber("DistanceToGoal", distance);
				SmartDashboard.putNumber("AngleToGoal", offsetAngle);
				
				

		}
	}
	
	public void makeAngleUsable(){
		if(rawAngle > 180){
			rawAngle = rawAngle - 360;
		}
		
		offsetAngle = rawAngle + Constants.ShootingJetsonCameraAngleOffset;
	}
	
	public double getAngleToGoal() {
		return offsetAngle;
	}
	
	public boolean isAngleAcceptable(){
		//Is the angle within the range specified in Constants?
		//System.out.println("acceptable angle");
		return Math.abs(offsetAngle) < Constants.ShootingmaxAngleError;
		
	}
	public boolean isDistanceAcceptable(){
		//Is the distance within range specified in Constants?
		return ( distance > Constants.ShootingMinGoalDistance && distance < Constants.ShootingMaxGoalDistance);
	}
	
	public double getDistanceFromGoal(){
		return distance;
	}

	public boolean isGoalFound() {
		return isGoalBool;
	}

}
