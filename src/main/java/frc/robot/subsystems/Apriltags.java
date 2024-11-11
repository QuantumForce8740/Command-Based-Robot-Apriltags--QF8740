// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class Apriltags extends SubsystemBase {
  /** Creates a new Apriltags. */
  private int currentID = 0;
  private Thread visionthread;

  public Apriltags() {
    new Thread(this::apriltagVision);
    visionthread.setDaemon(true);
  }

  public int getCurrentID(){
    return currentID;
  }

  public void startVision(){
    if (!visionthread.isAlive()){
      visionthread.start();
    }
  }

  public void apriltagVision(){
    var detector = new AprilTagDetector();
    detector.addFamily("tag36h11", 1);
    var poseEstimator = new AprilTagPoseEstimator.Config(0.1651, 699.3778103158814, 677.7161226393544,345.6059345433618, 207.12741326228522);
    var estimator = new AprilTagPoseEstimator(poseEstimator);
    System.out.println("Estimator: "+ estimator);
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640,480);
    CvSink cvsink = CameraServer.getVideo();
    CvSource cvsource = CameraServer.putVideo("Detector", 680,480);

    var mat = new Mat();
    var gray_mat = new Mat();

    while (!Thread.interrupted()){
      if (cvsink.grabFrame(mat) == 0){
        cvsource.notifyError(cvsink.getError());
        continue;
      }
      Imgproc.cvtColor(mat, gray_mat, Imgproc.COLOR_RGB2GRAY);
      AprilTagDetection[] detections = detector.detect(gray_mat);
      if (detections.length >0){
        AprilTagDetection detection = detections[0];
        int tagID = detection.getId();
        currentID = tagID;
      }
      cvsource.putFrame(mat);
    }
    detector.close();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
