package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.PhotonvisionSubsystem;
import frc.robot.util.ControlBoard;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.GameElement;

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

  private final Field2d m_field = new Field2d();

  Pose3d simulationPose = new Pose3d();
  private static Pose2d globalPose = new Pose2d(0, 0, new Rotation2d(0));

  StructPublisher<Pose3d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();

  /// StructPublisher<Pose3d> predictPublisher = NetworkTableInstance.getDefault()
  // .getStructTopic("PredictedElementPose", Pose3d.struct).publish();

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

    public record AngularVelocity3D(double roll, double pitch, double yaw) {}

    public record Velocity2D(double x, double y) {}
  }

  public static class TargetPredictor {

    public static boolean ALLIANCE_IS_BLUE =
        (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue);

    private static GameElement lastPredictedTarget = null;
    private static double targetConfidence = 0.0;

    private static final double CONFIDENCE_INCREMENT = 0.15; // Increase per consistent cycle
    private static final double CONFIDENCE_DECREMENT =
        0.09; // Decrease per cycle when candidate changes
    private static final double CONFIDENCE_THRESHOLD =
        0.3; // Below this threshold, target can be switched

    private static final double REEF_BIAS_MULTIPLIER = 0.8; // Higher = less bias (OLD = 0.7)
    private static final double CORAL_STATION_BIAS_MULTIPLIER = 0.57; // (OLD = 0.4)

    // Cone (forced-selection) parameters
    private static final double FORCE_SELECTION_RADIUS = 0.50;
    private static final double FORCE_SELECTION_CONE_HALF_ANGLE = Math.toRadians(42); // (OLD = 34)

    // "Time to approach" weight in the cost function
    private static final double TIME_COST_WEIGHT = 0.29;

    // Kinetic-energy penalty weight
    private static final double ENERGY_COST_WEIGHT = 0.4;

    // Each reef (a GameElement with branches) has a wall extended on each side
    // from the central AprilTag
    private static final double WALL_EXTENSION =
        0.875; // actual reef wall half-length // old = 0.825 length

    public static class PredictionResult {
      private final GameElement target;
      private final Pose2d targetPose;
      private final double confidence;
      private final double cost;

      public PredictionResult(GameElement target, double confidence, double cost) {
        this.target = target;
        this.confidence = confidence;
        this.cost = cost;
        this.targetPose = (target != null) ? GameElement.getPoseWithOffset(target, 1.0) : null;
      }

      public GameElement getTarget() {
        return target;
      }

      public Pose2d getTargetPose() {
        return targetPose;
      }

      public double getConfidence() {
        return confidence;
      }

      public double getCost() {
        return cost;
      }
    }

    public static PredictionResult predictTargetElement(RobotState state, ControlBoard cb) {
      if (cb.isAssisting) {
        return new PredictionResult(cb.previousConfirmedGoal, 1.0, 0.0);
      }

      GameElement forcedTarget = getForcedConeTarget(globalPose);
      if (forcedTarget != null) {
        lastPredictedTarget = forcedTarget;
        targetConfidence = 1.0;
        return new PredictionResult(lastPredictedTarget, targetConfidence, 0.0);
      }

      Translation2d robotTranslation = globalPose.getTranslation();
      double currentX = robotTranslation.getX();
      double currentY = robotTranslation.getY();

      double vX = state.getVelocity().x();
      double vY = state.getVelocity().y();
      double vSquared = vX * vX + vY * vY;
      double currentSpeed = (vSquared > FieldConstants.EPSILON) ? Math.sqrt(vSquared) : 0;

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

        // vector from robot to element
        double dx = elementX - currentX;
        double dy = elementY - currentY;

        // dot product (for projection and angle difference)
        double dot = dx * vX + dy * vY;

        // cost based on distance from element to the line of motion
        double cost =
            (currentSpeed > FieldConstants.EPSILON)
                ? Math.abs(vX * dy - vY * dx) / currentSpeed
                : Math.hypot(dx, dy);
        if (currentSpeed > FieldConstants.EPSILON) {
          cost += TIME_COST_WEIGHT * (dot / currentSpeed);
        }

        // Reorientation energy computed via dot product
        double distanceToElement = Math.hypot(dx, dy);
        if (currentSpeed > FieldConstants.EPSILON && distanceToElement > FieldConstants.EPSILON) {
          double cosAngleDiff = dot / (currentSpeed * distanceToElement);
          cost += ENERGY_COST_WEIGHT * (vSquared * (1 - cosAngleDiff));
        }

        // Ray-casting
        Translation2d rayStart = new Translation2d(currentX, currentY);
        Translation2d rayEnd = new Translation2d(elementX, elementY);
        if (isRayObstructed(rayStart, rayEnd, element)) {
          continue;
        }

        // (game cycle knowledge)
        GameElement previousGoal = cb.previousConfirmedGoal;
        if (previousGoal != null) {
          if (IntakeSubsystem.getInstance().coralDetected()) {
            cost *= REEF_BIAS_MULTIPLIER;
          } else {
            cost *= CORAL_STATION_BIAS_MULTIPLIER;
          }
        }

        if (cost < bestCost) {
          bestCost = cost;
          candidateTarget = element;
        }
      }

      // Confidence (hysteresis) to prevent flickering
      if (candidateTarget != null) {
        if (candidateTarget.equals(lastPredictedTarget)) {
          targetConfidence = Math.min(targetConfidence + CONFIDENCE_INCREMENT, 1.0);
        } else {
          targetConfidence = Math.max(targetConfidence - CONFIDENCE_DECREMENT, 0.0);
          if (targetConfidence < CONFIDENCE_THRESHOLD) {
            lastPredictedTarget = candidateTarget;
            targetConfidence = CONFIDENCE_INCREMENT;
          }
        }
      }
      return new PredictionResult(lastPredictedTarget, targetConfidence, bestCost);
    }

    private static GameElement getForcedConeTarget(Pose2d pose) {
      Translation2d robotTranslation = pose.getTranslation();
      double robotX = robotTranslation.getX();
      double robotY = robotTranslation.getY();

      GameElement forcedTarget = null;
      double bestDistanceSq = Double.MAX_VALUE;
      double radiusSq = FORCE_SELECTION_RADIUS * FORCE_SELECTION_RADIUS;

      for (GameElement element : GameElement.values()) {
        if (element.isBlue() != ALLIANCE_IS_BLUE || element.shouldIgnore()) {
          continue;
        }
        Pose2d elementPose = GameElement.getPoseWithOffset(element, 1.0);
        Translation2d elementTranslation = elementPose.getTranslation();
        double dx = robotX - elementTranslation.getX();
        double dy = robotY - elementTranslation.getY();
        double distanceSq = dx * dx + dy * dy;

        if (distanceSq <= radiusSq) {
          double angleElementToRobot =
              Math.atan2(robotY - elementTranslation.getY(), robotX - elementTranslation.getX());
          double angleDiff =
              Math.abs(
                  normalizeAngle(angleElementToRobot - elementPose.getRotation().getRadians()));
          if (angleDiff <= FORCE_SELECTION_CONE_HALF_ANGLE) {
            if (distanceSq < bestDistanceSq) {
              bestDistanceSq = distanceSq;
              forcedTarget = element;
            }
          }
        }
      }
      return forcedTarget;
    }

    private static boolean isRayObstructed(
        Translation2d rayStart, Translation2d rayEnd, GameElement candidate) {
      for (GameElement element : GameElement.values()) {
        if (element.hasBranches() && !element.equals(candidate)) {
          Pose2d reefPose = element.getLocation();
          Translation2d reefTranslation = reefPose.getTranslation();
          double reefX = reefTranslation.getX();
          double reefY = reefTranslation.getY();
          double reefFacing = reefPose.getRotation().getRadians();
          double cosLeft = Math.cos(reefFacing + Math.PI / 2);
          double sinLeft = Math.sin(reefFacing + Math.PI / 2);
          double cosRight = Math.cos(reefFacing - Math.PI / 2);
          double sinRight = Math.sin(reefFacing - Math.PI / 2);

          Translation2d wallLeft =
              new Translation2d(reefX + WALL_EXTENSION * cosLeft, reefY + WALL_EXTENSION * sinLeft);
          Translation2d wallRight =
              new Translation2d(
                  reefX + WALL_EXTENSION * cosRight, reefY + WALL_EXTENSION * sinRight);

          if (rayIntersectsSegment(rayStart, rayEnd, wallLeft, wallRight)) {
            return true;
          }
        }
      }
      return false;
    }

    private static boolean rayIntersectsSegment(
        Translation2d p, Translation2d q, Translation2d a, Translation2d b) {
      double o1 = orientation(p, q, a);
      double o2 = orientation(p, q, b);
      double o3 = orientation(a, b, p);
      double o4 = orientation(a, b, q);

      return o1 * o2 < 0 && o3 * o4 < 0
          || Math.abs(o1) < FieldConstants.EPSILON && onSegment(p, q, a)
          || Math.abs(o2) < FieldConstants.EPSILON && onSegment(p, q, b)
          || Math.abs(o3) < FieldConstants.EPSILON && onSegment(a, b, p)
          || Math.abs(o4) < FieldConstants.EPSILON && onSegment(a, b, q);
    }

    private static double orientation(Translation2d p, Translation2d q, Translation2d r) {
      return (q.getX() - p.getX()) * (r.getY() - p.getY())
          - (q.getY() - p.getY()) * (r.getX() - p.getX());
    }

    private static boolean onSegment(Translation2d p, Translation2d q, Translation2d r) {
      return r.getX() <= Math.max(p.getX(), q.getX()) + FieldConstants.EPSILON
          && r.getX() >= Math.min(p.getX(), q.getX()) - FieldConstants.EPSILON
          && r.getY() <= Math.max(p.getY(), q.getY()) + FieldConstants.EPSILON
          && r.getY() >= Math.min(p.getY(), q.getY()) - FieldConstants.EPSILON;
    }

    private static double normalizeAngle(double angle) {
      return Math.IEEEremainder(angle, 2 * Math.PI); // My brain is exploding with intellect
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
    previousPose = globalPose;
    previousTime = Timer.getFPGATimestamp();

    controlBoard = ControlBoard.getInstance();

    SmartDashboard.putBoolean("Odometry/Odometry Reset Requested", odometryResetRequested);
    SmartDashboard.putData("Field", m_field);
    // swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0, 0, 9999999));
  }

  public static Odometry getInstance() {
    if (instance == null) instance = new Odometry();
    return instance;
  }

  /**
   * Finds the closest GameElement to a given robot pose. If two elements are equidistant, the one
   * with the smallest angle difference is chosen.
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
      double distance =
          robotPose.getTranslation().getDistance(element.getLocation().getTranslation());

      if (distance < minDistance) {
        closest = element;
        minDistance = distance;
        minAngleDifference =
            FieldConstants.calculateAngleDifference(robotPose, element.getLocation());
      } else if (distance == minDistance) {
        double angleDifference =
            FieldConstants.calculateAngleDifference(robotPose, element.getLocation());
        if (angleDifference < minAngleDifference) {
          closest = element;
          minAngleDifference = angleDifference;
        }
      }
    }
    return closest;
  }

  public RobotState getRobotState() {
    // Return the current Pose2d from the swerve, the *computed* velocity, and the measured angular
    // velocity
    return new RobotState(
        globalPose,
        linearVelocity,
        new RobotState.AngularVelocity3D(
            getFieldRollRate(), getFieldPitchRate(), getFieldYawRate()));
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
    return globalPose;
  }

  public Pose3d getPose3d() {
    Pose2d pose = getPose();
    return new Pose3d(
        pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians()));
  }

  public void resetGyro() {
    gyro.reset();
    swerve.resetRotation(new Rotation2d(0));
  }

  public void resetOdometry() {
    // resetGyro();
    odometryResetRequested = true;
    SmartDashboard.putBoolean("Odometry/Odometry Reset Requested", odometryResetRequested);
  }

  public Command resetOdometryCommand() {
    return new InstantCommand(
            () -> {
              resetOdometry();
            },
            this)
        .andThen(new WaitUntilCommand(() -> !odometryResetRequested));
  }

  public void testResetOdo() {
    swerve.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public Pose2d predictFuturePose(double secondsAhead) {
    Pose2d currentPose = getPose();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();
    double currentTheta = currentPose.getRotation().getRadians();

    double vx = getVelocityX();
    double vy = getVelocityY();
    double angularVelocity = getFieldYawRate();

    double predictedX = currentX + vx * secondsAhead;
    double predictedY = currentY + vy * secondsAhead;
    double predictedTheta = currentTheta + angularVelocity * secondsAhead;

    return new Pose2d(predictedX, predictedY, new Rotation2d(predictedTheta));
  }

  @Override
  public void periodic() {
    globalPose = swerve.getPose();
    m_field.setRobotPose(globalPose);
    publisher.set(getPose3d());
    // Pose2d gePose = GameElement.getPoseWithOffset(controlBoard.desiredGoal, 1.0);
    // predictPublisher.set(new Pose3d(gePose.getX(), gePose.getY(), 0, new Rotation3d(0, 0,
    // gePose.getRotation().getRadians())));
    if (SmartDashboard.getBoolean("Odometry/Odometry Reset Requested", false)
        != odometryResetRequested) {
      System.out.println("Odometry reset requested");
      odometryResetRequested = !odometryResetRequested;
    }

    PoseEstimate limelightPose = limelight.getPoseEstimate(getRobotState(), false);
    if (odometryResetRequested) {
      // not using photonvision yet
      // EstimatedRobotPose photonVisionPose = photonvision.update(getRobotState().pose);
      PoseEstimate mt1PoseWithRot = limelight.getPoseEstimate(getRobotState(), false);
      if (limelightReset
          && mt1PoseWithRot != null
          && !mt1PoseWithRot.pose.equals(new Pose2d(0, 0, new Rotation2d(0)))) {
        swerve.resetPose(mt1PoseWithRot.pose);

        odometryResetRequested = false;
        SmartDashboard.putBoolean("Odometry/Odometry Reset Requested", odometryResetRequested);
      }
      // not using photonvision yet
      /*if (!limelightReset && photonVisionPose != null) {
          swerve.resetPose(photonVisionPose.estimatedPose.toPose2d());
      }*/
    } else {
    }

    if (limelightPose
        != null /*&& limelightPose.pose.getTranslation().getDistance(globalPose.getTranslation()) < 2*/) { // && limelightPose.pose.getTranslation().getDistance(previousRobotState.getPose().getTranslation()) < 1) {
      // TODO: tune
      swerve.resetPose(new Pose2d(limelightPose.pose.getTranslation(), globalPose.getRotation()));
      // swerve.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds,
      // VecBuilder.fill(0.1, 0.1, 9999999));
      // swerve.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
      // swerve.resetPose(new Pose2d(limelightPose.pose.getTranslation(),
      // globalPose.getRotation()));
    }

    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - previousTime;
    if (dt > 0) {
      Pose2d currentPose = globalPose;
      double dx = currentPose.getX() - previousPose.getX();
      double dy = currentPose.getY() - previousPose.getY();

      double vx = dx / dt;
      double vy = dy / dt;

      // Store the newly computed velocity
      linearVelocity = new RobotState.Velocity2D(vx, vy);
    }

    // Update for next iteration
    previousTime = currentTime;
    previousPose = globalPose;

    TargetPredictor.PredictionResult prediction =
        TargetPredictor.predictTargetElement(getRobotState(), controlBoard);

    controlBoard.desiredGoal = prediction.getTarget();

    controlBoard.goalConfidence = prediction.getConfidence();
    SmartDashboard.putString("Odometry/Current Goal", controlBoard.desiredGoal.name());
  }

  public void updateAllianceColor(boolean isBlue) {
    TargetPredictor.ALLIANCE_IS_BLUE = isBlue;
  }
}
