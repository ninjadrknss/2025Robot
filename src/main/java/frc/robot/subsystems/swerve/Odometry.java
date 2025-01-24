package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightSubsystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class Odometry extends SubsystemBase {
    private static Odometry instance;
    private final SwerveSubsystem swerve;
    private final LimelightSubsystem limelight;
    private final Pigeon2 gyro;
    /* Status Signals */
    private final StatusSignal<AngularVelocity> rollStatusSignal;
    private final StatusSignal<AngularVelocity> pitchStatusSignal;
    private final StatusSignal<AngularVelocity> yawStatusSignal;

    private boolean odometryResetRequested = false;

    // --- ADD THESE FIELDS FOR VELOCITY CALC ---
    private Pose2d previousPose = new Pose2d();
    private double previousTime = 0.0;

    // We'll store the linear velocity here so we can provide it in getRobotState()
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

    private Odometry() {
        this.swerve = SwerveSubsystem.getInstance();
        this.limelight = LimelightSubsystem.getInstance();
        this.gyro = swerve.getPigeon2();

        rollStatusSignal = gyro.getAngularVelocityXWorld();
        pitchStatusSignal = gyro.getAngularVelocityYWorld();
        yawStatusSignal = gyro.getAngularVelocityZWorld();
        
        // Initialize previousPose and previousTime:
        previousPose = swerve.getPose();
        previousTime = Timer.getFPGATimestamp();
    }

    public static Odometry getInstance() {
        if (instance == null) instance = new Odometry();
        return instance;
    }

    public void addVisionMeasurement() {
        RobotState previousRobotState = getRobotState();
        PoseEstimate limelightPose = limelight.getPoseEstimate(previousRobotState);
        if (limelightPose == null) return;
        if (limelightPose.pose.getTranslation().getDistance(previousRobotState.getPose().getTranslation()) < 1) {
            swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            swerve.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
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

    public void resetGyro() {
        gyro.reset();
        swerve.resetRotation(new Rotation2d(0)); 
    }

    public void resetOdometry() {
        resetGyro();
        odometryResetRequested = true;
    }

    public void displayValues(){
        SmartDashboard.putNumber("Odometry Position x", swerve.getPose().getX());
        SmartDashboard.putNumber("Odometry Position y", swerve.getPose().getY());
        SmartDashboard.putNumber("Odometry Rotation", swerve.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Odometry Velocity x", linearVelocity.x);
        SmartDashboard.putNumber("Odometry Velocity y", linearVelocity.y);
        SmartDashboard.putNumber("Odometry Angular Velocity", getFieldYawRate());
    }

    @Override
    public void periodic() {
        if (odometryResetRequested) {
            PoseEstimate limelightPose = limelight.getPoseEstimate(getRobotState());
            if (limelightPose != null) {
                swerve.resetPose(limelightPose.pose);
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
        displayValues();
    }
}
