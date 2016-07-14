package org.usfirst.frc.team1806.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SensorSuite {

	ArrayList<Double> navxYawTable;
	ArrayList<Long> timeTable;

	ArrayList<Double> voltageTable;

	double addedAverage = 0;
	double rollingYawAverage = 0;

	double addedVoltageAverage = 0;
	double rollingVoltageAverage = 0;

	public SensorSuite() {
		navxYawTable = new ArrayList<>();
		timeTable = new ArrayList<>();
		voltageTable = new ArrayList<>();
	}

	public void update() {

		calculateAngularVelocity();
		calculateVoltage();

		SmartDashboard.putNumber("Angular Velocity... : ", rollingYawAverage);

	}

	void calculateAngularVelocity() {
		// Add new yaw value to table
		navxYawTable.add(Robot.drivetrainSS.getTrueAngle());

		// Remove values until table has only 5 entries
		// keep size at or below 5
		while (navxYawTable.size() > 6) {
			navxYawTable.remove(0);
		}

		timeTable.add(System.currentTimeMillis());

		while (timeTable.size() > 6) {
			timeTable.remove(0);
		}

		addedAverage = 0;

		int loops = 0;

		if (navxYawTable.size() > 1 && timeTable.size() > 1) {

			for (int i = 0; i < navxYawTable.size() - 1; i++) {

				// Calculate dx/dt
				addedAverage += (Math.IEEEremainder(navxYawTable.get(i + 1) - navxYawTable.get(i), 360)
						/ ((timeTable.get(i + 1).doubleValue() - timeTable.get(i).doubleValue()) / 100));
				loops++;

			}

		}

		rollingYawAverage = addedAverage / loops;

		if (Math.abs(rollingYawAverage) >= 35) {
			rollingYawAverage = 0;
		}
	}

	public void calculateVoltage() {

		voltageTable.add(Robot.pdp.getVoltage());

		while (voltageTable.size() > 10) {
			voltageTable.remove(0);
		}

		int loops = 0;
		addedVoltageAverage = 0;
		
		if (voltageTable.size() > 0) {

			for (int i = 0; i < voltageTable.size(); i++) {

				addedVoltageAverage += voltageTable.get(i);
				loops++;

			}

		}
		
		rollingVoltageAverage = addedVoltageAverage / loops;

	}

	public double getAngularVelocity() {
		// returns angular velocity in deg/s
		return rollingYawAverage;
	}

	public double getVoltage() {
		return rollingVoltageAverage;
	}

}
