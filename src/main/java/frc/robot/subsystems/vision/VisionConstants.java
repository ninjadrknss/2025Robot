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
        public final static String version = "3G";
        public final static String streamIp = "http://10.7.51.11:5800";
		public final static String dashboardIp = "http://10.7.51.11:5801";
        public final static String name = "limelight";

        public final static Distance zOffset = Units.Inches.of(12.224 + 3.75); // inches
        public final static Distance yOffset = Units.Inches.of(13-6.01); // inches
    }

    public static class PhotonVision {
        public final static String name = "photonvision";
        public final static String streamIp = "http://";

        public final static String leftCameraName = "left";
        public final static String rightCameraName = "right";

        public final static PhotonPoseEstimator.PoseStrategy poseStrategy = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        public final static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final Transform3d leftCameraToRobot = new Transform3d(
                new Translation3d(
                        Units.Meter.convertFrom(2.979, Units.Inch),
                        Units.Meter.convertFrom(9.41, Units.Inch),
                        Units.Meter.convertFrom(37.418, Units.Inch)
                ), /*Y, X, Z*/
                new Rotation3d(
                        0,
                        -Units.Radians.convertFrom(-15, Units.Degrees),
                        Units.Radians.convertFrom(155, Units.Degrees)
                )
        );
        public static final Transform3d rightCameraToRobot = new Transform3d(
                new Translation3d(
                        Units.Meter.convertFrom(2.979, Units.Inch),
                        -Units.Meter.convertFrom(9.41, Units.Inch),
                        Units.Meter.convertFrom(37.418, Units.Inch)
                ), /*Y, X, Z*/
                new Rotation3d(
                        0,
                        -Units.Radians.convertFrom(-15, Units.Degrees),
                        Units.Radians.convertFrom(-155, Units.Degrees)
                )
        );
    }
}
