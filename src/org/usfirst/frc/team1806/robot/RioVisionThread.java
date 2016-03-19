package org.usfirst.frc.team1806.robot;

import java.util.Comparator;
import java.util.Vector;

import org.usfirst.frc.team1806.robot.RobotStates.VisionTrackingState;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.Point;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.vision.USBCamera;

public class RioVisionThread extends Thread {

	USBCamera camera;
	CameraServer server;
	Image frame;
	Image binaryFrame;
	Image binaryFrame2;
	int imaqError;
	
	public double deltaAngle;
	
	//degrees
	private final double kCameraHorizontalFOV = 54;
	private final double kCameraVerticalFOV = 40;
	private final double kCameraXResolution = 640;

	public RioVisionThread() {
		
		camera = new USBCamera("cam3");
		camera.setExposureManual(0);
		camera.setExposureHoldCurrent();
		camera.setBrightness(0);
		
		server = CameraServer.getInstance();
		server.setQuality(50);

	}

	public void run() {
		CameraServer.getInstance().startAutomaticCapture(camera);
		while (true) {
			
			if (Robot.states.visionTrackingStateTracker == VisionTrackingState.ROBORIO) {
				
				
				
			} else {

				// else do nothing
				try {
					RioVisionThread.sleep(200);
				} catch (InterruptedException e) {
					System.out.println(e);
				}
			}
		}
	}
}
