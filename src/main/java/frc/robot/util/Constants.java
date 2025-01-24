package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    /** Use {@see Robot.canivorebus #} instead */
    @SuppressWarnings("DeprecatedIsStillUsed")
    @Deprecated
    public static final String drivebus = "Drivebase";


    // TODO: These are completely arbitrary values, replace with actual values
    public enum GameElement {
        // Reefs (6 parts per reef)
        REEF_RED_1(new Pose2d(4.57, 3.25, Rotation2d.fromDegrees(240)), false),
        REEF_RED_2(new Pose2d(4.57, 3.91, Rotation2d.fromDegrees(180)), false),
        REEF_RED_3(new Pose2d(4.57, 4.57, Rotation2d.fromDegrees(120)), false),
        REEF_RED_4(new Pose2d(5.23, 4.57, Rotation2d.fromDegrees(60)), false),
        REEF_RED_5(new Pose2d(5.23, 3.91, Rotation2d.fromDegrees(0)), false),
        REEF_RED_6(new Pose2d(5.23, 3.25, Rotation2d.fromDegrees(300)), false),
    
        REEF_BLUE_1(new Pose2d(19.3, 13.07, Rotation2d.fromDegrees(300)), true),
        REEF_BLUE_2(new Pose2d(19.3, 13.71, Rotation2d.fromDegrees(0)), true),
        REEF_BLUE_3(new Pose2d(19.3, 14.35, Rotation2d.fromDegrees(60)), true),
        REEF_BLUE_4(new Pose2d(18.64, 14.35, Rotation2d.fromDegrees(120)), true),
        REEF_BLUE_5(new Pose2d(18.64, 13.71, Rotation2d.fromDegrees(180)), true),
        REEF_BLUE_6(new Pose2d(18.64, 13.07, Rotation2d.fromDegrees(240)), true),
    
        // Coral stations
        CORAL_STATION_RED_1(new Pose2d(0.66, 3.25, Rotation2d.fromDegrees(0)), false),
        CORAL_STATION_RED_2(new Pose2d(0.66, 7.39, Rotation2d.fromDegrees(0)), false),
        CORAL_STATION_BLUE_1(new Pose2d(16.7, 3.25, Rotation2d.fromDegrees(180)), true),
        CORAL_STATION_BLUE_2(new Pose2d(16.7, 7.39, Rotation2d.fromDegrees(180)), true),
    
        // Cages on the barge
        CAGE_RED_1(new Pose2d(9.28, 1.92, Rotation2d.fromDegrees(30)), false),
        CAGE_RED_2(new Pose2d(8.27, 6.14, Rotation2d.fromDegrees(150)), false),
        CAGE_RED_3(new Pose2d(9.28, 10.5, Rotation2d.fromDegrees(30)), false),
        CAGE_BLUE_1(new Pose2d(8.27, 16.0, Rotation2d.fromDegrees(210)), true),
        CAGE_BLUE_2(new Pose2d(9.28, 19.5, Rotation2d.fromDegrees(330)), true),
        CAGE_BLUE_3(new Pose2d(8.27, 23.0, Rotation2d.fromDegrees(210)), true),
    
        // Processors
        PROCESSOR_RED(new Pose2d(11.56, 8.06, Rotation2d.fromDegrees(270)), false),
        PROCESSOR_BLUE(new Pose2d(5.99, 7.99, Rotation2d.fromDegrees(90)), true),
    
        // Algae scoring areas
        ALGAE_RED(new Pose2d(9.28, 6.14, Rotation2d.fromDegrees(0)), false),
        ALGAE_BLUE(new Pose2d(14.4, 6.14, Rotation2d.fromDegrees(180)), true);
    
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
