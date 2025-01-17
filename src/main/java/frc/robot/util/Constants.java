package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    /** Use {@see Robot.canivorebus #} instead */
    @SuppressWarnings("DeprecatedIsStillUsed")
    @Deprecated
    public static final String canbus = "";




    public static enum GameElement {
        // Reefs
        REEF_RED(new Pose2d(12.0, 0.0, Rotation2d.fromDegrees(0)), false),
        REEF_BLUE(new Pose2d(12.0, 26.5, Rotation2d.fromDegrees(180)), true),

        // Reef levels
        REEF_L1_RED(new Pose2d(12.0, 1.0, Rotation2d.fromDegrees(0)), false),
        REEF_L2_RED(new Pose2d(12.0, 1.5, Rotation2d.fromDegrees(0)), false),
        REEF_L3_RED(new Pose2d(12.0, 2.0, Rotation2d.fromDegrees(0)), false),
        REEF_L4_RED(new Pose2d(12.0, 2.5, Rotation2d.fromDegrees(0)), false),
        REEF_L1_BLUE(new Pose2d(12.0, 25.5, Rotation2d.fromDegrees(180)), true),
        REEF_L2_BLUE(new Pose2d(12.0, 25.0, Rotation2d.fromDegrees(180)), true),
        REEF_L3_BLUE(new Pose2d(12.0, 24.5, Rotation2d.fromDegrees(180)), true),
        REEF_L4_BLUE(new Pose2d(12.0, 24.0, Rotation2d.fromDegrees(180)), true),

        // Coral stations
        CORAL_STATION_RED_1(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(90)), false),
        CORAL_STATION_RED_2(new Pose2d(1.0, 5.0, Rotation2d.fromDegrees(90)), false),
        CORAL_STATION_BLUE_1(new Pose2d(1.0, 26.5, Rotation2d.fromDegrees(270)), true),
        CORAL_STATION_BLUE_2(new Pose2d(1.0, 21.5, Rotation2d.fromDegrees(270)), true),

        // Cages on the barge
        CAGE_RED_1(new Pose2d(14.0, 3.5, Rotation2d.fromDegrees(0)), false),
        CAGE_RED_2(new Pose2d(14.0, 7.0, Rotation2d.fromDegrees(0)), false),
        CAGE_RED_3(new Pose2d(14.0, 10.5, Rotation2d.fromDegrees(0)), false),
        CAGE_BLUE_1(new Pose2d(14.0, 16.0, Rotation2d.fromDegrees(180)), true),
        CAGE_BLUE_2(new Pose2d(14.0, 19.5, Rotation2d.fromDegrees(180)), true),
        CAGE_BLUE_3(new Pose2d(14.0, 23.0, Rotation2d.fromDegrees(180)), true),

        // Processors
        PROCESSOR_RED(new Pose2d(20.0, 0.0, Rotation2d.fromDegrees(90)), false),
        PROCESSOR_BLUE(new Pose2d(20.0, 26.5, Rotation2d.fromDegrees(270)), true),

        // Nets
        NET_RED(new Pose2d(15.0, 0.0, Rotation2d.fromDegrees(90)), false),
        NET_BLUE(new Pose2d(15.0, 26.5, Rotation2d.fromDegrees(270)), true),

        // Algae scoring areas
        ALGAE_RED(new Pose2d(18.0, 0.0, Rotation2d.fromDegrees(90)), false),
        ALGAE_BLUE(new Pose2d(18.0, 26.5, Rotation2d.fromDegrees(270)), true),

        // Neutral elements
        BAR(new Pose2d(13.0, 13.25, Rotation2d.fromDegrees(0)), false),
        FIELD(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)), false); // Origin for reference

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
