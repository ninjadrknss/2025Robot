package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
    public static class Limelight {
        public final static double version = 3.0;
        public final static String streamIp = "http://10.7.51.11:5800";
		public final static String dashboardIp = "http://10.7.51.11:5801";
        public final static String name = "limelight";

        public final static double height = 15.61 + 3.75; // inches
        public final static double angle = 35;
    }

    public static class PhotonVision {
        public final static String name = "photonvision";
        public final static String streamIp = "http://";
        
        public final static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }
}
