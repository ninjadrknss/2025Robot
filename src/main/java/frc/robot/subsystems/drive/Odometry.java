package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightSubsystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.lib.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.PhotonvisionSubsystem;
import frc.robot.util.ControlBoard;
import frc.robot.util.Constants.GameElement;
import frc.robot.util.Constants;
import org.photonvision.EstimatedRobotPose;

public class Odometry extends SubsystemBase {
    private static Odometry instance;
    private final SwerveSubsystem swerve;
    private final LimelightSubsystem limelight;
    private final PhotonvisionSubsystem photonvision;
    private final ControlBoard controlBoard;
    private final Pigeon2 gyro;
    /* Status Signals */
    private final StatusSignal<AngularVelocity> rollStatusSignal;
    private final StatusSignal<AngularVelocity> pitchStatusSignal;
    private final StatusSignal<AngularVelocity> yawStatusSignal;

    private boolean odometryResetRequested = false;
    private static final boolean limelightReset = true;

    // --- ADD THESE FIELDS FOR VELOCITY CALC ---
    private Pose2d previousPose = new Pose2d();
    private double previousTime = 0.0;

    Pose3d simulationPose = new Pose3d();

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose3d.struct).publish();

    StructPublisher<Pose3d> predictPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("PredictedElementPose", Pose3d.struct).publish();

    private RobotState.Velocity2D linearVelocity = new RobotState.Velocity2D(0, 0);

    public static class RobotState {
        public Pose2d pose;
        public Velocity2D velocity;
        public AngularVelocity3D angularVelocity;

        public RobotState(Pose2d pose, Velocity2D velocity, AngularVelocity3D angularVelocity) {
            this.pose = pose;
            this.velocity = velocity;
            this.angularVelocity = angularVelocity;
        }

        public Pose2d getPose() {
            return pose;
        }

        public Velocity2D getVelocity() {
            return velocity;
        }

        public AngularVelocity3D getAngularVelocity() {
            return angularVelocity;
        }

        public record AngularVelocity3D(double roll, double pitch, double yaw) {
        }

        public record Velocity2D(double x, double y) {
        }
    }

    public static class TargetPredictor {

        private static final boolean ALLIANCE_IS_BLUE = true;

        private static GameElement lastPredictedTarget = null;
        private static double targetConfidence = 0.0;

        private static final double CONFIDENCE_INCREMENT = 0.15; // Increase per consistent cycle
        private static final double CONFIDENCE_DECREMENT = 0.09; // Decrease per cycle when candidate changes
        private static final double CONFIDENCE_THRESHOLD = 0.3; // Below this threshold, target can be switched

        // Cone (forced-selection) parameters
        private static final double FORCE_SELECTION_RADIUS = 0.85;
        private static final double FORCE_SELECTION_CONE_HALF_ANGLE = Math.toRadians(34);

        // "Time to approach" weight in the cost function
        private static final double TIME_COST_WEIGHT = 0.29;

        // Kinetic-energy penalty weight
        private static final double ENERGY_COST_WEIGHT = 0.5;

        // Each reef (a GameElement with branches) has a wall extended on each side
        // from the central AprilTag
        private static final double WALL_EXTENSION = 0.825; // actual reef wall half-length

        public static Object[] predictTargetElement(RobotState state, ControlBoard cb) {

            // ----- Step 1: Forced Selection (element-faces-robot cone) -----
            GameElement forcedTarget = getForcedConeTarget(state);
            if (forcedTarget != null) {
                lastPredictedTarget = forcedTarget;
                targetConfidence = 1.0;
                return new Object[] { lastPredictedTarget, targetConfidence };
            }

            // ----- Step 2: Prediction & Raycasting -----
            Pose2d robotPose = state.getPose();
            Translation2d robotTranslation = robotPose.getTranslation();
            double currentX = robotTranslation.getX();
            double currentY = robotTranslation.getY();

            double vX = state.getVelocity().x();
            double vY = state.getVelocity().y();
            double vSquared = vX * vX + vY * vY;
            double currentSpeed = (vSquared > Constants.EPSILON) ? Math.sqrt(vSquared) : 0;

            GameElement candidateTarget = null;
            double bestCost = Double.MAX_VALUE;
            for (GameElement element : GameElement.values()) {

                if (element.isBlue() != ALLIANCE_IS_BLUE || element.shouldIgnore()) {
                    continue;
                }

                Pose2d elementPose = GameElement.getPoseWithOffset(element, 1.0);
                Translation2d elementTranslation = elementPose.getTranslation();
                double elementX = elementTranslation.getX();
                double elementY = elementTranslation.getY();

                // tOptimal: time in the future that (assuming constant velocity)
                // best aligns the robot with the target
                double tOptimal = 0;
                if (vSquared > Constants.EPSILON) {
                    double dotProduct = (currentX - elementX) * vX + (currentY - elementY) * vY;
                    tOptimal = -dotProduct / vSquared;
                    if (tOptimal < 0) {
                        tOptimal = 0;
                    }
                }

                // Predicted position if we keep current velocity for tOptimal
                double predictedX = currentX + vX * tOptimal;
                double predictedY = currentY + vY * tOptimal;
                double predictedDistance = Math.hypot(predictedX - elementX, predictedY - elementY);

                double cost = predictedDistance;
                if (currentSpeed > 0) {
                    cost += TIME_COST_WEIGHT * (tOptimal * currentSpeed);
                }

                if (vSquared > Constants.EPSILON) {
                    // Angle from current robot position to this element
                    double angleToTarget = Math.atan2(elementY - currentY, elementX - currentX);
                    // Angle of robot velocity
                    double angleVel = Math.atan2(vY, vX);
                    double angleDiff = normalizeAngle(angleToTarget - angleVel);
                    double absAngleDiff = Math.abs(angleDiff);

                    // Simple "reorientation" energy ~ v^2 * (1 - cos(angleDiff))
                    double reorientationEnergy = vSquared * (1 - Math.cos(absAngleDiff));
                    cost += ENERGY_COST_WEIGHT * reorientationEnergy;
                }

                // Raycast from current robot position to element
                Translation2d rayStart = new Translation2d(currentX, currentY);
                Translation2d rayEnd = new Translation2d(elementX, elementY);

                // If the ray is obstructed by any reef's wall (except this element's own), skip
                if (isRayObstructed(rayStart, rayEnd, element)) {
                    continue;
                }

                GameElement previousGoal = cb.previousConfirmedGoal;
                if (previousGoal != null) {
                    // If the previous confirmed goal was a coral station...
                    if (previousGoal.name().startsWith("CORAL_STATION_") && element.hasBranches()) {
                        // ...then bias in favor of reefs
                        cost *= 0.7;
                    }
                    // If the previous confirmed goal was a reef (hasBranches)...
                    if (previousGoal.hasBranches() && element.name().startsWith("CORAL_STATION_")) {
                        // ...then bias in favor of coral stations
                        cost *= 0.4;
                    }
                }

                if (cost < bestCost) {
                    bestCost = cost;
                    candidateTarget = element;
                }
            }

            // ----- Confidence (hysteresis) -----
            if (candidateTarget != null) {
                if (candidateTarget.equals(lastPredictedTarget)) {
                    targetConfidence = Math.min(targetConfidence + CONFIDENCE_INCREMENT, 1.0);
                } else {
                    targetConfidence = Math.max(targetConfidence - CONFIDENCE_DECREMENT, 0.0);
                    if (targetConfidence < CONFIDENCE_THRESHOLD) {
                        lastPredictedTarget = candidateTarget;
                        targetConfidence = CONFIDENCE_INCREMENT; // reset for new candidate
                    }
                }
            }

            return new Object[] { lastPredictedTarget, targetConfidence };
        }

        private static GameElement getForcedConeTarget(RobotState state) {
            Pose2d robotPose = state.getPose();
            Translation2d robotTranslation = robotPose.getTranslation();
            double robotX = robotTranslation.getX();
            double robotY = robotTranslation.getY();

            GameElement forcedTarget = null;
            double forcedTargetDistance = Double.MAX_VALUE;

            for (GameElement element : GameElement.values()) {

                // Skip if alliance color doesn't match
                if (element.isBlue() != ALLIANCE_IS_BLUE || element.shouldIgnore()) {
                    continue;
                }

                Pose2d elementPose = GameElement.getPoseWithOffset(element, 1.0);
                Translation2d elementTranslation = elementPose.getTranslation();
                double elementX = elementTranslation.getX();
                double elementY = elementTranslation.getY();
                double elementFacing = elementPose.getRotation().getRadians(); // element's facing

                double distance = Math.hypot(robotX - elementX, robotY - elementY);
                if (distance <= FORCE_SELECTION_RADIUS) {
                    // Angle from element to robot
                    double angleElementToRobot = Math.atan2(robotY - elementY, robotX - elementX);
                    double angleDiff = Math.abs(normalizeAngle(angleElementToRobot - elementFacing));
                    if (angleDiff <= FORCE_SELECTION_CONE_HALF_ANGLE) {
                        // If multiple qualify, choose the closest
                        if (distance < forcedTargetDistance) {
                            forcedTargetDistance = distance;
                            forcedTarget = element;
                        }
                    }
                }
            }

            return forcedTarget;
        }

        private static boolean isRayObstructed(Translation2d rayStart, Translation2d rayEnd, GameElement candidate) {
            for (GameElement element : GameElement.values()) {
                if (element.hasBranches() && !element.equals(candidate)) {
                    Pose2d reefPose = element.getLocation();
                    Translation2d reefTranslation = reefPose.getTranslation();
                    double reefX = reefTranslation.getX();
                    double reefY = reefTranslation.getY();
                    double reefFacing = reefPose.getRotation().getRadians();
                    Translation2d wallLeft = new Translation2d(
                            reefX + WALL_EXTENSION * Math.cos(reefFacing + Math.PI / 2),
                            reefY + WALL_EXTENSION * Math.sin(reefFacing + Math.PI / 2));
                    Translation2d wallRight = new Translation2d(
                            reefX + WALL_EXTENSION * Math.cos(reefFacing - Math.PI / 2),
                            reefY + WALL_EXTENSION * Math.sin(reefFacing - Math.PI / 2));

                    if (rayIntersectsSegment(rayStart, rayEnd, wallLeft, wallRight)) {
                        return true;
                    }
                }
            }
            return false;
        }

        private static boolean rayIntersectsSegment(Translation2d p, Translation2d q,
                                                    Translation2d a, Translation2d b) {
            double o1 = orientation(p, q, a);
            double o2 = orientation(p, q, b);
            double o3 = orientation(a, b, p);
            double o4 = orientation(a, b, q);

            if (o1 * o2 < 0 && o3 * o4 < 0) {
                return true;
            }

            if (Math.abs(o1) < Constants.EPSILON && onSegment(p, q, a))
                return true;
            if (Math.abs(o2) < Constants.EPSILON && onSegment(p, q, b))
                return true;
            if (Math.abs(o3) < Constants.EPSILON && onSegment(a, b, p))
                return true;
            if (Math.abs(o4) < Constants.EPSILON && onSegment(a, b, q))
                return true;

            return false;
        }

        private static double orientation(Translation2d p, Translation2d q, Translation2d r) {
            return (q.getX() - p.getX()) * (r.getY() - p.getY())
                    - (q.getY() - p.getY()) * (r.getX() - p.getX());
        }

        private static boolean onSegment(Translation2d p, Translation2d q, Translation2d r) {
            return r.getX() <= Math.max(p.getX(), q.getX()) + Constants.EPSILON &&
                    r.getX() >= Math.min(p.getX(), q.getX()) - Constants.EPSILON &&
                    r.getY() <= Math.max(p.getY(), q.getY()) + Constants.EPSILON &&
                    r.getY() >= Math.min(p.getY(), q.getY()) - Constants.EPSILON;
        }

        private static double normalizeAngle(double angle) {
            while (angle > Math.PI) {
                angle -= 2 * Math.PI;
            }
            while (angle < -Math.PI) {
                angle += 2 * Math.PI;
            }
            return angle;
        }
    }

    private Odometry() {
        this.swerve = SwerveSubsystem.getInstance();
        this.limelight = LimelightSubsystem.getInstance();
        this.photonvision = PhotonvisionSubsystem.getInstance();
        this.gyro = swerve.getPigeon2();

        rollStatusSignal = gyro.getAngularVelocityXWorld();
        pitchStatusSignal = gyro.getAngularVelocityYWorld();
        yawStatusSignal = gyro.getAngularVelocityZWorld();

        // Initialize previousPose and previousTime:
        previousPose = swerve.getPose();
        previousTime = Timer.getFPGATimestamp();

        controlBoard = ControlBoard.getInstance();

    }

    public static Odometry getInstance() {
        if (instance == null)
            instance = new Odometry();
        return instance;
    }

    /**
     * Finds the closest GameElement to a given robot pose.
     * If two elements are equidistant, the one with the smallest angle difference
     * is chosen.
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
                minAngleDifference = Constants.calculateAngleDifference(robotPose, element.getLocation());
            } else if (distance == minDistance) {
                double angleDifference = Constants.calculateAngleDifference(robotPose, element.getLocation());
                if (angleDifference < minAngleDifference) {
                    closest = element;
                    minAngleDifference = angleDifference;
                }
            }
        }
        return closest;
    }

    public void addVisionMeasurement() {
        RobotState previousRobotState = getRobotState();
        PoseEstimate limelightPose = limelight.getPoseEstimate(previousRobotState);
        EstimatedRobotPose photonVisionPose = photonvision.update(previousRobotState.pose);

        if (limelightPose != null && limelightPose.pose.getTranslation().getDistance(previousRobotState.getPose().getTranslation()) < 1) {
            swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            swerve.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
        }

        if (photonVisionPose != null && photonVisionPose.estimatedPose.getTranslation().toTranslation2d().getDistance(previousRobotState.getPose().getTranslation()) < 1) {
            swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            swerve.addVisionMeasurement(photonVisionPose.estimatedPose.toPose2d(), photonVisionPose.timestampSeconds);
        }
    }

    public RobotState getRobotState() {
        // Return the current Pose2d from the swerve, the *computed* velocity, and the measured angular velocity
        return new RobotState(
                swerve.getPose(),
                linearVelocity,
                new RobotState.AngularVelocity3D(
                        getFieldRollRate(),
                        getFieldPitchRate(),
                        getFieldYawRate()
                )
        );
    }

    public double getFieldPitchRate() {
        return pitchStatusSignal.getValueAsDouble();
    }

    public double getFieldYawRate() {
        return yawStatusSignal.getValueAsDouble();
    }

    public double getFieldRollRate() {
        return rollStatusSignal.getValueAsDouble();
    }

    public double getVelocityX() {
        return linearVelocity.x;
    }

    public double getVelocityY() {
        return linearVelocity.y;
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public Pose3d getPose3d(){
        Pose2d pose = getPose();
        return new Pose3d(pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians()));
    }

    public void resetGyro() {
        gyro.reset();
        swerve.resetRotation(new Rotation2d(0));
    }

    public void resetOdometry() {
        resetGyro();
        odometryResetRequested = true;
    }

    public void testResetOdo(){
        swerve.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
    }

    public void displayValues(){
        SmartDashboard.putNumber("Odometry Position x", swerve.getPose().getX());
        SmartDashboard.putNumber("Odometry Position y", swerve.getPose().getY());
        SmartDashboard.putNumber("Odometry Rotation", swerve.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Odometry Velocity x", linearVelocity.x);
        SmartDashboard.putNumber("Odometry Velocity y", linearVelocity.y);
        SmartDashboard.putNumber("Odometry Angular Velocity", getFieldYawRate());
        SmartDashboard.putString("P Target Element", controlBoard.desiredGoal.name());
        SmartDashboard.putString("P Target Confidence", controlBoard.goalConfidence());
        SmartDashboard.putString("Last Confirmed Target", (controlBoard.previousConfirmedGoal != null ? controlBoard.previousConfirmedGoal.name() : ""));
        SmartDashboard.putString("Selected Branch", controlBoard.selectedBranch.name());
        SmartDashboard.putString("Score Level", controlBoard.scoreLevel.name());
    }

    @Override
    public void periodic() {

        publisher.set(getPose3d());
        Pose2d gePose = GameElement.getPoseWithOffset(controlBoard.desiredGoal, 1.0);
        predictPublisher.set(new Pose3d(gePose.getX(), gePose.getY(), 0, new Rotation3d(0, 0, gePose.getRotation().getRadians())));

        if (odometryResetRequested) {
            PoseEstimate limelightPose = limelight.getPoseEstimate(getRobotState());
            EstimatedRobotPose photonVisionPose = photonvision.update(getRobotState().pose);
            if (limelightReset && limelightPose != null) {
                swerve.resetPose(limelightPose.pose);
            }
            if (!limelightReset && photonVisionPose != null) {
                swerve.resetPose(photonVisionPose.estimatedPose.toPose2d());
            }
            odometryResetRequested = false;
        } else {
            addVisionMeasurement();
        }

        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - previousTime;
        if (dt > 0) {
            Pose2d currentPose = swerve.getPose();
            double dx = currentPose.getX() - previousPose.getX();
            double dy = currentPose.getY() - previousPose.getY();

            double vx = dx / dt;
            double vy = dy / dt;

            // Store the newly computed velocity
            linearVelocity = new RobotState.Velocity2D(vx, vy);
        }

        // Update for next iteration
        previousTime = currentTime;
        previousPose = swerve.getPose();

        controlBoard.desiredGoal = (GameElement) TargetPredictor.predictTargetElement(getRobotState(), controlBoard)[0];
        controlBoard.goalConfidence = (double) TargetPredictor.predictTargetElement(getRobotState(), controlBoard)[1];
        displayValues();
    }
}