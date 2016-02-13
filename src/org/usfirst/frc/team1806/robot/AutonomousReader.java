package org.usfirst.frc.team1806.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousReader extends Thread implements Runnable {

	// Auto routine receiver vars
	private ArrayList<String> commandStrings;
	private boolean autonRoutineReceived = false;
	private String autonString = "";
	private boolean shouldRun = true;

	public AutonomousReader() {
		// Auto routine receiver var inits
		commandStrings = new ArrayList<String>();
	}

	public void run() {

		while (shouldRun) {
			synchronized (this) {
				try {
					autonString = "";
					autonString = SmartDashboard.getString("CommandString");

				} catch (Exception e) {
					//System.out.println("Error in getting Autonomous String from SmartDashboard");
				}

				// This will continually update the ArrayList with new Strings
				// from
				// the SDB
				if (autonString != null) {
					commandStrings.clear();
					// You've received a new string from the smartdashboard
					// widget :-)
					for (String s : autonString.split(":")) {
						// add all strings from command, separated by colons
						commandStrings.add(s);
						//SmartDashboard.putString(String.valueOf(commandStrings.indexOf(s)), s);
						//System.out.println(s);
					}
					autonRoutineReceived = true;
				}

				try {
					this.wait(50);
				} catch (Exception e) {
					e.printStackTrace();
				}

			}
		}

	}

	public boolean CommandsReceived() {
		return autonRoutineReceived;
	}

	public ArrayList<String> getCommandStringArray() {
		return commandStrings;
	}

	public void stopThread() {
		shouldRun = false;
	}

}
