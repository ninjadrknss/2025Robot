package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class Constants {
    /** Use {@see Robot.canivorebus #} instead */
    @SuppressWarnings("DeprecatedIsStillUsed")
    @Deprecated
    public static final String drivebus = "Drivebus";
    @SuppressWarnings("DeprecatedIsStillUsed")
    @Deprecated
    public static final String elevatorbus = "Elevatorbus";

    public static final double EPSILON = 1e-6;
    public static final double CORAL_OFFSET = 0.206; // meters

    // Game elements, actual values from april tags. might be a bit off, as the
    // locations are based off of april tag locations.
    // the pattern is as follows: everything is ordered from right to left from blue
    // origin.
    // everything is in meters

    public enum GameElement {
        // Reefs (6 parts per reef)
        REEF_RED_1(new Branches(
                offsetByAngle(new Pose2d(14.1528, 4.0386, Rotation2d.fromDegrees(0)), CORAL_OFFSET, -90), // left branch
                new Pose2d(14.1528, 4.0386, Rotation2d.fromDegrees(0)), // "mid"
                offsetByAngle(new Pose2d(14.1528, 4.0386, Rotation2d.fromDegrees(0)), CORAL_OFFSET, +90)), // right branch
                false),

        REEF_RED_2(new Branches(
                offsetByAngle(new Pose2d(13.4620, 3.3020, Rotation2d.fromDegrees(300)), CORAL_OFFSET, -90),
                new Pose2d(13.4620, 3.3020, Rotation2d.fromDegrees(300)),
                offsetByAngle(new Pose2d(13.4620, 3.3020, Rotation2d.fromDegrees(300)), CORAL_OFFSET, +90)),
                false),

        REEF_RED_3(new Branches(
                offsetByAngle(new Pose2d(13.4620, 4.7498, Rotation2d.fromDegrees(60)), CORAL_OFFSET, -90),
                new Pose2d(13.4620, 4.7498, Rotation2d.fromDegrees(60)),
                offsetByAngle(new Pose2d(13.4620, 4.7498, Rotation2d.fromDegrees(60)), CORAL_OFFSET, +90)),
                false),

        REEF_RED_4(new Branches(
                offsetByAngle(new Pose2d(12.6492, 3.3020, Rotation2d.fromDegrees(240)), CORAL_OFFSET, -90),
                new Pose2d(12.6492, 3.3020, Rotation2d.fromDegrees(240)),
                offsetByAngle(new Pose2d(12.6492, 3.3020, Rotation2d.fromDegrees(240)), CORAL_OFFSET, +90)),
                false),

        REEF_RED_5(new Branches(
                offsetByAngle(new Pose2d(12.6492, 4.7498, Rotation2d.fromDegrees(120)), CORAL_OFFSET, -90),
                new Pose2d(12.6492, 4.7498, Rotation2d.fromDegrees(120)),
                offsetByAngle(new Pose2d(12.6492, 4.7498, Rotation2d.fromDegrees(120)), CORAL_OFFSET, +90)),
                false),

        REEF_RED_6(new Branches(
                offsetByAngle(new Pose2d(12.2174, 4.0386, Rotation2d.fromDegrees(180)), CORAL_OFFSET, -90),
                new Pose2d(12.2174, 4.0386, Rotation2d.fromDegrees(180)),
                offsetByAngle(new Pose2d(12.2174, 4.0386, Rotation2d.fromDegrees(180)), CORAL_OFFSET, +90)),
                false),

        REEF_BLUE_1(new Branches(
                offsetByAngle(new Pose2d(5.3086, 4.0386, Rotation2d.fromDegrees(0)), CORAL_OFFSET, -90),
                new Pose2d(5.3086, 4.0386, Rotation2d.fromDegrees(0)),
                offsetByAngle(new Pose2d(5.3086, 4.0386, Rotation2d.fromDegrees(0)), CORAL_OFFSET, +90)),
                true),

        REEF_BLUE_2(new Branches(
                offsetByAngle(new Pose2d(4.8922, 3.3020, Rotation2d.fromDegrees(300)), CORAL_OFFSET, -90),
                new Pose2d(4.8922, 3.3020, Rotation2d.fromDegrees(300)),
                offsetByAngle(new Pose2d(4.8922, 3.3020, Rotation2d.fromDegrees(300)), CORAL_OFFSET, +90)),
                true),

        REEF_BLUE_3(new Branches(
                offsetByAngle(new Pose2d(4.8922, 4.7498, Rotation2d.fromDegrees(60)), CORAL_OFFSET, -90),
                new Pose2d(4.8922, 4.7498, Rotation2d.fromDegrees(60)),
                offsetByAngle(new Pose2d(4.8922, 4.7498, Rotation2d.fromDegrees(60)), CORAL_OFFSET, +90)),
                true),

        REEF_BLUE_4(new Branches(
                offsetByAngle(new Pose2d(4.0640, 3.3020, Rotation2d.fromDegrees(240)), CORAL_OFFSET, -90),
                new Pose2d(4.0640, 3.3020, Rotation2d.fromDegrees(240)),
                offsetByAngle(new Pose2d(4.0640, 3.3020, Rotation2d.fromDegrees(240)), CORAL_OFFSET, +90)),
                true),

        REEF_BLUE_5(new Branches(
                offsetByAngle(new Pose2d(4.0640, 4.7498, Rotation2d.fromDegrees(120)), CORAL_OFFSET, -90),
                new Pose2d(4.0640, 4.7498, Rotation2d.fromDegrees(120)),
                offsetByAngle(new Pose2d(4.0640, 4.7498, Rotation2d.fromDegrees(120)), CORAL_OFFSET, +90)),
                true),

        REEF_BLUE_6(new Branches(
                offsetByAngle(new Pose2d(3.6576, 4.0386, Rotation2d.fromDegrees(180)), CORAL_OFFSET, -90),
                new Pose2d(3.6576, 4.0386, Rotation2d.fromDegrees(180)),
                offsetByAngle(new Pose2d(3.6576, 4.0386, Rotation2d.fromDegrees(180)), CORAL_OFFSET, +90)),
                true),

        // Coral stations
        CORAL_STATION_RED_1(new Pose2d(16.6878, 0.6604, Rotation2d.fromDegrees(126)), false),
        CORAL_STATION_RED_2(new Pose2d(16.6878, 7.3914, Rotation2d.fromDegrees(234)), false),
        CORAL_STATION_BLUE_1(new Pose2d(0.8636, 0.6604, Rotation2d.fromDegrees(54)), true),
        CORAL_STATION_BLUE_2(new Pose2d(0.8636, 7.3914, Rotation2d.fromDegrees(306)), true),

        // Cages on the barge
        CAGE_RED_1(new Pose2d(8.7630, 0.0000, Rotation2d.fromDegrees(0)), false, true),
        CAGE_RED_2(new Pose2d(8.7630, 1.9050, Rotation2d.fromDegrees(0)), false, true),
        CAGE_RED_3(new Pose2d(8.7630, 0.0000, Rotation2d.fromDegrees(0)), false, true),
        CAGE_BLUE_1(new Pose2d(8.7630, 0.0000, Rotation2d.fromDegrees(0)), true, true),
        CAGE_BLUE_2(new Pose2d(8.7630, 6.1468, Rotation2d.fromDegrees(0)), true, true),
        CAGE_BLUE_3(new Pose2d(8.7630, 0.0000, Rotation2d.fromDegrees(0)), true, true),

        // Processors
        PROCESSOR_RED(new Pose2d(5.9944, 0.0000, Rotation2d.fromDegrees(90)), true, true),
        PROCESSOR_BLUE(new Pose2d(11.5670, 8.0518, Rotation2d.fromDegrees(270)), false, true);

        private final Pose2d center;
        private final Branches branches;
        private final boolean isBlue;
        private final boolean ignoreByTargetPredictor;

        GameElement(Branches branches, boolean isBlue) {
            if (branches == null) {
                throw new IllegalArgumentException("Branches cannot be null for reef elements.");
            }
            this.branches = branches;
            this.center = branches.mid; // center is taken from branches
            this.isBlue = isBlue;
            ignoreByTargetPredictor = false;
        }

        GameElement(Pose2d center, boolean isBlue, boolean ignoreByTargetPredictor) {
            if (center == null) {
                throw new IllegalArgumentException("Center pose cannot be null.");
            }
            this.center = center;
            this.branches = null;
            this.isBlue = isBlue;
            this.ignoreByTargetPredictor = ignoreByTargetPredictor;
        }

        GameElement(Pose2d center, boolean isBlue) {
            this(center, isBlue, false);
        }

        public Pose2d getLeftBranch() {
            if (!hasBranches()) {
                throw new UnsupportedOperationException("game element does not have branches");
            }
            return branches.left;
        }

        public Pose2d getCenter() {
            return center;
        }

        public boolean shouldIgnore(){
            return ignoreByTargetPredictor;
        }

        public Pose2d getRightBranch() {
            if (!hasBranches()) {
                throw new UnsupportedOperationException("game element does not have branches");
            }
            return branches.right;
        }

        public boolean hasBranches() {
            return branches != null;
        }

        public Pose2d getLocation() {
            return center;
        }

        public boolean isBlue() {
            return isBlue;
        }

        private static double getXWithOffset(Pose2d center, double offset) {
            return center.getX() + offset * Math.cos(center.getRotation().getRadians());
        }

        private static double getYWithOffset(Pose2d center, double offset) {
            return center.getY() + offset * Math.sin(center.getRotation().getRadians());
        }

        public static Pose2d getPoseWithOffset(GameElement element, double offset) {
            return new Pose2d(getXWithOffset(element.center, offset), getYWithOffset(element.center, offset),
                    element.center.getRotation());
        }

        public static Pose2d getPoseWithOffset(Pose2d center, double offset) {
            return new Pose2d(getXWithOffset(center, offset), getYWithOffset(center, offset), center.getRotation());
        }

        public static List<GameElement> getColor(boolean isBlue) {
            return Arrays.stream(GameElement.values())
                    .filter(element -> element.isBlue() == isBlue)
                    .collect(Collectors.toList());
        }

        @Override
        public String toString() {
            return String.format("%s [Pose=%s, isBlue=%b]", name(), center, isBlue);
        }

        public static class Branches {
            public final Pose2d left;
            public final Pose2d mid;
            public final Pose2d right;

            public Branches(Pose2d left, Pose2d mid, Pose2d right) {
                if (left == null || mid == null || right == null) {
                    throw new IllegalArgumentException("Branch poses cannot be null.");
                }
                this.left = left;
                this.mid = mid;
                this.right = right;
            }
        }

        public enum Branch {
            LEFT, CENTER, RIGHT
        }

        public enum ScoreLevel {
            L1, L2, L3, L4
        }

    }

    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI; // please dont crucify me
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public static Pose2d offsetByAngle(Pose2d center, double offsetMeters, double angleOffsetDegrees) {
        // "angleOffsetDegrees" is usually +-90 for this perpendicular offset
        // but idk maybe we gonna use this at some point for something else
        Rotation2d offsetRotation = center.getRotation().plus(Rotation2d.fromDegrees(angleOffsetDegrees));
        double newX = center.getX() + offsetMeters * Math.cos(offsetRotation.getRadians());
        double newY = center.getY() + offsetMeters * Math.sin(offsetRotation.getRadians());
        return new Pose2d(newX, newY, center.getRotation());
    }

    public static double calculateAngleDifference(Pose2d robotPose, Pose2d elementPose) {
        Translation2d directionToElement = elementPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d elementDirection = new Rotation2d(directionToElement.getX(), directionToElement.getY());
        return Math.abs(robotPose.getRotation().minus(elementDirection).getDegrees());
    }
}
