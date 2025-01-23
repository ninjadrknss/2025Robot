package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightSubsystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.util.LimelightHelpers.*;

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

        /**
         * @param roll  Roll, degrees/second
         * @param pitch Pitch, degrees/second
         * @param yaw   Yaw, degrees/second
         */
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
            swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999)); // I have no clue what this is but its int eh instructions
            swerve.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
        }
    }

    public RobotState getRobotState() {
        return new RobotState(
                getRobotPose(),
                new RobotState.Velocity2D(0, 0),
                new RobotState.AngularVelocity3D(
                        getFieldPitchRate(),
                        getFieldRawRate(),
                        getFieldRollRate()
                )
        );
    }

    public Pose2d getRobotPose(){
        return swerve.getPose();
        //swerve.getPose();
    }

    public double getFieldPitchRate() {
        return pitchStatusSignal.getValueAsDouble();
    }

    public double getFieldRawRate() {
        return yawStatusSignal.getValueAsDouble();
    }

    public double getFieldRollRate() {
        return rollStatusSignal.getValueAsDouble();
    }

    public void resetGyro() {
        gyro.reset();
        swerve.resetRotation(new Rotation2d(0)); // TODO: needs to be based off of WPI_Blue
    }

    public void resetOdometry() {
        resetGyro();
        odometryResetRequested = true;
    }


    @Override
    public void periodic() {
        if (odometryResetRequested) {
            //this needs to be in perodic because limelight might not initially see the target, so we need to wait until it does
            PoseEstimate limelightPose = limelight.getPoseEstimate(getRobotState());
            if (limelightPose != null) swerve.resetPose(limelightPose.pose);
            odometryResetRequested = false;
        }else{
            addVisionMeasurement();
        }


    }
}
