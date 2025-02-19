package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonPoseEstimator;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Degrees;

public class VisionConstants {
    public static class Limelight {
        public final static String version = "3G";
        public final static String streamIp = "http://10.7.51.11:5800";
		public final static String dashboardIp = "http://10.7.51.11:5801";
        public final static String name = "limelight";

        public final static double height = 15.61 + 3.75; // inches
        public final static double angle = 35;
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
                        Meter.convertFrom(2.979, Inch),
                        Meter.convertFrom(9.41, Inch),
                        Meter.convertFrom(37.418, Inch)
                ), /*Y, X, Z*/
                new Rotation3d(
                        0,
                        -Radians.convertFrom(-15, Degrees),
                        Radians.convertFrom(155, Degrees)
                )
        );
        public static final Transform3d rightCameraToRobot = new Transform3d(
                new Translation3d(
                        Meter.convertFrom(2.979, Inch),
                        -Meter.convertFrom(9.41, Inch),
                        Meter.convertFrom(37.418, Inch)
                ), /*Y, X, Z*/
                new Rotation3d(
                        0,
                        -Radians.convertFrom(-15, Degrees),
                        Radians.convertFrom(-155, Degrees)
                )
        );
    }
}
