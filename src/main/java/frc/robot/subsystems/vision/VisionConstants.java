package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import org.photonvision.PhotonPoseEstimator;

public class VisionConstants {
  public static class Limelight {
    public static final String version = "3G";
    public static final String streamIp = "http://10.7.51.11:5800";
    public static final String dashboardIp = "http://10.7.51.11:5801";
    public static final String name = "limelight";

    public static final Distance zOffset = Units.Inches.of(12.224 + 3.75); // inches
    public static final Distance yOffset = Units.Inches.of(13 - 6.01); // inches
  }

  public static class PhotonVision {
    public static final String name = "photonvision";
    public static final String streamIp = "http://";

    public static final String leftCameraName = "left";
    public static final String rightCameraName = "right";

    public static final PhotonPoseEstimator.PoseStrategy poseStrategy =
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static final AprilTagFieldLayout field =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Transform3d leftCameraToRobot =
        new Transform3d(
            new Translation3d(
                Units.Meter.convertFrom(2.979, Units.Inch),
                Units.Meter.convertFrom(9.41, Units.Inch),
                Units.Meter.convertFrom(37.418, Units.Inch)), /*Y, X, Z*/
            new Rotation3d(
                0,
                -Units.Radians.convertFrom(-15, Units.Degrees),
                Units.Radians.convertFrom(155, Units.Degrees)));
    public static final Transform3d rightCameraToRobot =
        new Transform3d(
            new Translation3d(
                Units.Meter.convertFrom(2.979, Units.Inch),
                -Units.Meter.convertFrom(9.41, Units.Inch),
                Units.Meter.convertFrom(37.418, Units.Inch)), /*Y, X, Z*/
            new Rotation3d(
                0,
                -Units.Radians.convertFrom(-15, Units.Degrees),
                Units.Radians.convertFrom(-155, Units.Degrees)));
  }
}
