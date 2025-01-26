package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    /** Use {@see Robot.canivorebus #} instead */
    @SuppressWarnings("DeprecatedIsStillUsed")
    @Deprecated
    public static final String drivebus = "Drivebase";


    // Game elements, actual values from april tags. might be a bit off, as the locations are based off of april tag locations. 
    // the pattern is as follows: everything is ordered from right to left from blue origin. 
    // everything is in meters
    public enum GameElement {
        // Reefs (6 parts per reef)
        REEF_RED_1(new Pose2d(14.1528, 4.0386, Rotation2d.fromDegrees(0)), false),
        REEF_RED_2(new Pose2d(13.4620, 3.3020, Rotation2d.fromDegrees(300)), false),
        REEF_RED_3(new Pose2d(13.4620, 4.7498, Rotation2d.fromDegrees(60)), false),
        REEF_RED_4(new Pose2d(12.6492, 3.3020, Rotation2d.fromDegrees(240)), false),
        REEF_RED_5(new Pose2d(12.6492, 4.7498, Rotation2d.fromDegrees(120)), false),
        REEF_RED_6(new Pose2d(12.2174, 4.0386, Rotation2d.fromDegrees(180)), false),
    
        // Note: REEF_BLUE_1 is intentionally unchanged
        REEF_BLUE_1(new Pose2d(5.3086, 4.0386, Rotation2d.fromDegrees(0)), true),
        REEF_BLUE_2(new Pose2d(4.8922, 3.3020, Rotation2d.fromDegrees(300)), true),
        REEF_BLUE_3(new Pose2d(4.8922, 4.7498, Rotation2d.fromDegrees(60)), true),
        REEF_BLUE_4(new Pose2d(4.0640, 3.3020, Rotation2d.fromDegrees(240)), true),
        REEF_BLUE_5(new Pose2d(4.0640, 4.7498, Rotation2d.fromDegrees(120)), true),
        REEF_BLUE_6(new Pose2d(3.6576, 4.0386, Rotation2d.fromDegrees(180)), true),
    
        // Coral stations
        CORAL_STATION_RED_1(new Pose2d(16.6878, 0.6604, Rotation2d.fromDegrees(126)), false),
        CORAL_STATION_RED_2(new Pose2d(16.6878, 7.3914, Rotation2d.fromDegrees(234)), false),
        CORAL_STATION_BLUE_1(new Pose2d(0.8636, 0.6604, Rotation2d.fromDegrees(54)), true),
        CORAL_STATION_BLUE_2(new Pose2d(0.8636, 7.3914, Rotation2d.fromDegrees(306)), true),
    
        // Cages on the barge
        CAGE_RED_1(new Pose2d(8.7630, 0.0000, Rotation2d.fromDegrees(0)), false),
        CAGE_RED_2(new Pose2d(8.7630, 1.9050, Rotation2d.fromDegrees(0)), false),
        CAGE_RED_3(new Pose2d(8.7630, 0.0000, Rotation2d.fromDegrees(0)), false),
        CAGE_BLUE_1(new Pose2d(8.7630, 0.0000, Rotation2d.fromDegrees(0)), true),
        CAGE_BLUE_2(new Pose2d(8.7630, 6.1468, Rotation2d.fromDegrees(0)), true),
        CAGE_BLUE_3(new Pose2d(8.7630, 0.0000, Rotation2d.fromDegrees(0)), true),
    
        // Processors
        PROCESSOR_RED(new Pose2d(11.5670, 8.0518, Rotation2d.fromDegrees(270)), false),
        PROCESSOR_BLUE(new Pose2d(5.9944, 0.0000, Rotation2d.fromDegrees(90)), true),
    
        // Algae scoring areas
        ALGAE_RED(new Pose2d(9.2410, 1.9050, Rotation2d.fromDegrees(0)), false),
        ALGAE_BLUE(new Pose2d(8.2804, 6.1468, Rotation2d.fromDegrees(180)), true);    

        private final Pose2d location;
        private final boolean isBlue;

        GameElement(Pose2d location, boolean isBlue) {
            this.location = location;
            this.isBlue = isBlue;
        }

        public Pose2d getLocation() {
            return location;
        }

        public boolean isBlue() {
            return isBlue;
        }

        private double getXWithOffset(double offset) {
            return location.getX() + offset * Math.cos(location.getRotation().getRadians());
        }
        
        private double getYWithOffset(double offset) {
            return location.getY() + offset * Math.sin(location.getRotation().getRadians());
        }

        public Pose2d getPoseWithOffset(double offset) {
            return new Pose2d(getXWithOffset(offset), getYWithOffset(offset), location.getRotation());
        }


        /**
         * Finds the closest GameElement to a given robot pose.
         * If two elements are equidistant, the one with the smallest angle difference is chosen.
         *
         * @param robotPose The current Pose2d of the robot.
         * @return The closest GameElement.
         */
        public static GameElement closestElement(Pose2d robotPose) {
            GameElement closest = null;
            double minDistance = Double.MAX_VALUE;
            double minAngleDifference = Double.MAX_VALUE;

            for (GameElement element : GameElement.values()) {
                // Euclidean distance
                double distance = robotPose.getTranslation().getDistance(element.getLocation().getTranslation());

                if (distance < minDistance) {
                    closest = element;
                    minDistance = distance;
                    minAngleDifference = calculateAngleDifference(robotPose, element.getLocation());
                } else if (distance == minDistance) {
                    double angleDifference = calculateAngleDifference(robotPose, element.getLocation());
                    if (angleDifference < minAngleDifference) {
                        closest = element;
                        minAngleDifference = angleDifference;
                    }
                }
            }
            return closest;
        }

        private static double calculateAngleDifference(Pose2d robotPose, Pose2d elementPose) {
            Translation2d directionToElement = elementPose.getTranslation().minus(robotPose.getTranslation());
            Rotation2d elementDirection = new Rotation2d(directionToElement.getX(), directionToElement.getY());
            return Math.abs(robotPose.getRotation().minus(elementDirection).getDegrees());
        }


        @Override
        public String toString() {
            return String.format("%s [Pose=%s, isBlue=%b]", name(), location, isBlue);
        }
    }

}
