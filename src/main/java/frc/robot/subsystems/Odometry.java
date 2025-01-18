package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
//import com.ctre.phoenix.sensors.Pigeon2;

public class Odometry extends SubsystemBase {
    private static Odometry instance;
    private SwerveSubsystem swerve;
    private LimelightSubsystem limelight;
    //private Pigeon2 gyro;

    private Odometry() {
        this.swerve = SwerveSubsystem.getInstance();
    }

    public static Odometry getInstance() {
        if (instance == null) instance = new Odometry();
        return instance;
    }

    public void addVisionMeasurement() {

    }

    public Pose2d getRobotPose(){
        return swerve.getState().Pose;
    }

    // public Pose2d getRobotVel(){
    //     return gyro.get
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
}
