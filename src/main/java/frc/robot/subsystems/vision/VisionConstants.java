package frc.robot.subsystems.vision;

public class VisionConstants {
    public static class Limelight {
        public static double version = 3.0;
        public static String streamIp = "http://10.7.51.11:5800";
		public static String dashboardIp = "http://10.7.51.11:5801";
        public static String name = "limelight";

        public static double height = 15.61 + 3.75; // inches
        public static double angle = 35;
    }

    public static class PhotonVision {
        public static String name = "photonvision";
        public static String streamIp = "http://";

        public static double height = 15.61 + 3.75; // inches
        public static double angle = 35;
    }
}
