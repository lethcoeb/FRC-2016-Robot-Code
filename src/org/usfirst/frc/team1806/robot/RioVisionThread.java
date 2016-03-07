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
		
		camera = new USBCamera("cam2");
		// 10.18.6.14
		// 10.1.91.100
		server = CameraServer.getInstance();
		server.setQuality(50);
		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		binaryFrame2 = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM,
				100.0, 0, 0);
		double areaMin = .1;
		criteria[0].lower = (float) areaMin;

		SmartDashboard.putNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
		SmartDashboard.putNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
		SmartDashboard.putNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
		SmartDashboard.putNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
		SmartDashboard.putNumber("Tote val min", TOTE_LUM_RANGE.minValue);
		SmartDashboard.putNumber("Tote val max", TOTE_LUM_RANGE.maxValue);
		SmartDashboard.putNumber("Area min %", AREA_MINIMUM);
		
		deltaAngle = 0;

	}

	public class Scores {
		double Area;
		double Aspect;
	};

	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport> {
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;

		public int compareTo(ParticleReport r) {
			return (int) (r.Area - this.Area);
		}

		public int compare(ParticleReport r1, ParticleReport r2) {
			return (int) (r1.Area - r2.Area);
		}
	};

	double ratioToScore(double ratio) {
		return (Math.max(0, Math.min(100 * (1 - Math.abs(1 - ratio)), 100)));
	}

	double AreaScore(ParticleReport report) {
		double boundingArea = (report.BoundingRectBottom - report.BoundingRectTop)
				* (report.BoundingRectRight - report.BoundingRectLeft);
		// Tape is 7" edge so 49" bounding rect. With 2" wide tape it covers 24"
		// of the rect.
		return ratioToScore((320 / 96) * report.Area / boundingArea);
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the
	 * retro-reflective target. Target is 7"x7" so aspect should be 1
	 */
	double AspectScore(ParticleReport report) {
		return ratioToScore(((report.BoundingRectRight - report.BoundingRectLeft)
				/ (report.BoundingRectBottom - report.BoundingRectTop)));
	}

	// Constants
	// FIXME These ranges
	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(90, 30); // Default hue
																// range for
																// yellow tote
	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(12, 255); // Default
																	// saturation
																	// range for
																	// yellow
																	// tote
	NIVision.Range TOTE_LUM_RANGE = new NIVision.Range(200, 255);// Default
																	// value
																	// range for
																	// yellow
																	// tote
	double AREA_MINIMUM = 0.1; // Default Area minimum for particle as a
								// percentage of total image area
	// double LONG_RATIO = 2.22; //Tote long side = 26.9 / Tote height = 12.1 =
	// 2.22
	// double SHORT_RATIO = 1.4; //Tote short side = 16.9 / Tote height = 12.1 =
	// 1.4
	double SCORE_MIN = 1; // Minimum score to be considered a tote
	double ASPECT_MIN = 50;
	double ASPECT_MAX = 2;
	double VIEW_ANGLE = 64; // View angle fo camera, set to Axis m1011 (49.4) by
							// default, 64 for m1013, 51.7 for 206, 52 for
							// HD3000 square, 60 for HD3000 640x480
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0, 0, 1, 1);
	Scores scores = new Scores();
	public boolean isTote;

	public void run() {
		CameraServer.getInstance().startAutomaticCapture(camera);
		while (true) {
			//FIXME THIS!!!
			if (Robot.states.visionTrackingStateTracker == VisionTrackingState.ROBORIO) {
				
				System.out.println("Vision processing bruh");

				camera.getImage(frame);
				//CameraServer.getInstance().setImage(frame);

				//Used for quick changing of sorting values. Commented out for performance.
				/*TOTE_HUE_RANGE.minValue = (int) SmartDashboard.getNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
				TOTE_HUE_RANGE.maxValue = (int) SmartDashboard.getNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
				TOTE_SAT_RANGE.minValue = (int) SmartDashboard.getNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
				TOTE_SAT_RANGE.maxValue = (int) SmartDashboard.getNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
				TOTE_LUM_RANGE.minValue = (int) SmartDashboard.getNumber("Tote val min", TOTE_LUM_RANGE.minValue);
				TOTE_LUM_RANGE.maxValue = (int) SmartDashboard.getNumber("Tote val max", TOTE_LUM_RANGE.maxValue);*/

				// threshold image
				// FIXME: edit these vals
				// NIVision.imaqColorThreshold(binaryFrame, frame, 255,
				// NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE,
				// TOTE_VAL_RANGE);
				NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSL, TOTE_HUE_RANGE,
						TOTE_SAT_RANGE, TOTE_LUM_RANGE);
				
				
				//UNCOMMENT THIS TO DISPLAY THE PARTICLE REPORT
				//FIXME
				

				// Send particle count to dashboard
				int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Masked particles", numParticles);

				// filter out small particles
				// FIXME areamin value
				
				imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

				// Send particle count after filtering to dashboard
				numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
				SmartDashboard.putNumber("Filtered particles", numParticles);

				if (numParticles > 0) {
					// Measure particles and sort by particle size
					Vector<ParticleReport> particles = new Vector<ParticleReport>();
					for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
						ParticleReport par = new ParticleReport();
						par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
								NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
						
						
						/*
						par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
								NIVision.MeasurementType.MT_AREA);
						par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
								NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
						par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
								NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
						par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
								NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
						par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
								NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
								*/
						particles.add(par);
					}
					particles.sort(null);
					
					
					particles.elementAt(0).Area = NIVision.imaqMeasureParticle(binaryFrame, 0, 0,
							NIVision.MeasurementType.MT_AREA);
					particles.elementAt(0).BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, 0, 0,
							NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
					particles.elementAt(0).BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, 0, 0,
							NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
					particles.elementAt(0).BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, 0, 0,
							NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
					particles.elementAt(0).BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, 0, 0,
							NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);

					// This example only scores the largest particle. Extending
					// to score all particles and choosing the desired one is
					// left as an exercise
					// for the reader. Note that this scores and reports
					// information about a single particle (single L shaped
					// target). To get accurate information
					// about the location of the tote (not just the distance)
					// you will need to correlate two adjacent targets in order
					// to find the true center of the tote.
					scores.Aspect = AspectScore(particles.elementAt(0));
					SmartDashboard.putNumber("Aspect", scores.Aspect);
					scores.Area = AreaScore(particles.elementAt(0));
					SmartDashboard.putNumber("Area", scores.Area);
					isTote =  scores.Area > SCORE_MIN && scores.Area > ASPECT_MIN;
					
					if(isTote){
						//compute angle to target
						
					}

					// Send distance and tote status to dashboard. The bounding
					// rect, particularly the horizontal center (left - right)
					// may be useful for rotating/driving towards a tote
					SmartDashboard.putBoolean("TargetFound", isTote);
					
					deltaAngle = angleTo(binaryFrame, particles.elementAt(0));
					if(isTote){
						SmartDashboard.putNumber("angle to goal", deltaAngle);
					}else{
						
						deltaAngle = 0;
						SmartDashboard.putNumber("angle to goal", 0);

					}

					//double distance = computeDistance(binaryFrame, particles.elementAt(0));
					//SmartDashboard.putNumber("Distance from goal", distance);

					// TODO: calculate distance and angle
					// SmartDashboard.putNumber("Distance",
					// computeDistance(binaryFrame, particles.elementAt(0)));
				} else {
					SmartDashboard.putBoolean("TargetFound", false);
				}

				
				/*try {
					RioVisionThread.sleep(50);
				} catch (InterruptedException e) {
					System.out.println(e);
				}*/
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
	
	double angleTo(Image image, ParticleReport report){
		
		//FIXME clean this shit up
		
		NIVision.GetImageSizeResult size;
		size = NIVision.imaqGetImageSize(image);
		
		NIVision.Point actualTop = new NIVision.Point(size.width/2, size.height);
		NIVision.Point actualBottom = new NIVision.Point(size.width/2, 0);
		
		int targetX = (int) (report.BoundingRectLeft + report.BoundingRectRight) / 2;
		NIVision.Point targetTop = new NIVision.Point(targetX, size.height);
		NIVision.Point targetBottom = new NIVision.Point(targetX, 0);
		
		NIVision.Line targetMid = new NIVision.Line(targetTop, targetBottom);
		NIVision.Line actualMid = new NIVision.Line(actualTop, actualBottom);
		
		NIVision.imaqDrawLineOnImage(image, image, DrawMode.DRAW_VALUE, targetTop, targetBottom, 255);
		NIVision.imaqDrawLineOnImage(image, image, DrawMode.DRAW_VALUE, actualTop, actualBottom, 200);
		
		//CameraServer.getInstance().setImage(image);
		
		
		//returns angle to center of goal in degrees
		return (targetX - size.width/2) * kCameraHorizontalFOV/kCameraXResolution;
	}

	double computeDistance(Image image, ParticleReport report) {
		double normalizedWidth, targetWidth;
		NIVision.GetImageSizeResult size;

		size = NIVision.imaqGetImageSize(image);
		normalizedWidth = 2 * (report.BoundingRectRight - report.BoundingRectLeft) / size.width;
		targetWidth = 20;

		return targetWidth / (normalizedWidth * 12 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2)));
	}

}
