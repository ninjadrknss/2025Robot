package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.util.LimelightHelpers.*;

public class Odometry extends SubsystemBase {
    private static Odometry instance;
    private SwerveSubsystem swerve;
    private LimelightSubsystem limelight;
    private Pigeon2 gyro;

    private boolean odometryResetRequested = false;

    public static class RobotState {
        public static class AngularVelocity3D {
            private final double x; // Roll, degrees/second
            private final double y; // Pitch, degrees/second
            private final double z; // Yaw, degrees/second
        
            public AngularVelocity3D(double x, double y, double z) {
                this.x = x;
                this.y = y;
                this.z = z;
            }
        
            public double getRoll() {
                return x;
            }
        
            public double getPitch() {
                return y;
            }
        
            public double getYaw() {
                return z;
            }
        }

        public Pose2d pose;
        public AngularVelocity3D angularVelocity;

        public RobotState(Pose2d pose, AngularVelocity3D angularVelocity) {
            this.pose = pose;
            this.angularVelocity = angularVelocity;
        }
        
        public Pose2d getPose() {
            return pose;
        }
        public AngularVelocity3D getAngularVelocity() {
            return angularVelocity;
        }
    }

    private Odometry() {
        this.swerve = SwerveSubsystem.getInstance();
        this.limelight = LimelightSubsystem.getInstance();
        this.gyro = swerve.getPigeon2();
    }

    public static Odometry getInstance() {
        if (instance == null) instance = new Odometry();
        return instance;
    }

    public void addVisionMeasurement() {
        RobotState previousRobotState = getRobotState();
        PoseEstimate limelightPose = limelight.getPoseEstimate(previousRobotState);
        if (limelightPose != null && limelightPose.pose.getTranslation().getDistance(previousRobotState.getPose().getTranslation()) < 1) {
            swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); // I have no clue what this is but its int eh instructions
            swerve.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
        }
    }

    public RobotState getRobotState(){
        return new RobotState(getRobotPose(), new RobotState.AngularVelocity3D(getFieldPitchRate(), getFieldRawRate(), getFieldRollRate()));
    }

    public Pose2d getRobotPose(){
        return swerve.getState().Pose;
    }

    public double getFieldPitchRate(){
        return gyro.getAngularVelocityYWorld().getValueAsDouble();
    }
    public double getFieldRawRate(){
        return gyro.getAngularVelocityZWorld().getValueAsDouble();
    }

    public double getFieldRollRate(){
        return gyro.getAngularVelocityXWorld().getValueAsDouble();
    }

    public void resetOdometry(){
        odometryResetRequested = true;
    }

    @Override
    public void periodic() {
        if (odometryResetRequested) {
            //this needs to be in perodic because limelight might not initially see the target, so we need to wait until it does
            PoseEstimate limelightPose = limelight.getPoseEstimate(getRobotState());
            if ( limelightPose != null)
            swerve.resetPose(limelightPose.pose);
            odometryResetRequested = false;
        }
    }
    
}
