package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.util.LimelightHelpers.*;
import edu.wpi.first.math.geometry.Rotation3d;

public class Odometry extends SubsystemBase {
    private static Odometry instance;
    private SwerveSubsystem swerve;
    private LimelightSubsystem limelight;
    private Pigeon2 gyro;

    private Odometry() {
        this.swerve = SwerveSubsystem.getInstance();
    }

    public static Odometry getInstance() {
        if (instance == null) instance = new Odometry();
        return instance;
    }

    public void addVisionMeasurement() {

        PoseEstimate limelightPose = limelight.getPoseEstimate(getRobotPose(), getRotationRate());
        if (limelightPose != null) {
            swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); // TODO: I have no clue what this is https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
            swerve.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
        }
    }

    public Pose2d getRobotPose(){
        return swerve.getState().Pose;
    }

    public Rotation3d getRobotRotationPose(){
        return gyro.getRotation3d();
    }

    public Rotation3d getRobotRotationVelocity(){
        double pitchRate = gyro.getAngularVelocityXWorld().getValueAsDouble();
        double yawRate = gyro.getAngularVelocityZWorld().getValueAsDouble();
        double rollRate = gyro.getAngularVelocityZWorld().getValueAsDouble();
        return new Rotation3d(pitchRate, yawRate, rollRate);
    }

    public double getRotationRate(){
        return gyro.getAngularVelocityZWorld().getValueAsDouble();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
}
