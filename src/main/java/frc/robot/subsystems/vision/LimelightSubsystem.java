package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.Odometry.RobotState;

public class LimelightSubsystem extends SubsystemBase {
    public enum LEDMode {
		PIPELINE,
		OFF,
		BLINK,
		ON
    }

    private static LimelightSubsystem instance;

    private final String name;

    public static LimelightSubsystem getInstance() {
        if (instance == null) instance = new LimelightSubsystem();
        return instance;
    }

	private LimelightSubsystem() {
        name = VisionConstants.Limelight.name;
        LimelightHelpers.setCameraPose_RobotSpace(name, VisionConstants.Limelight.yOffset, 0, VisionConstants.Limelight.zOffset, 0, 0, 0);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(this.name);
    }

    public PoseEstimate getPoseEstimate(RobotState previousRobotState) {
        Pose2d previousRobotPose = previousRobotState.getPose();

        double rotationRate = previousRobotState.getAngularVelocity().yaw();
        LimelightHelpers.SetRobotOrientation(
                VisionConstants.Limelight.name,
                previousRobotPose.getRotation().getDegrees(),
                rotationRate, 0, 0, 0, 0
        );

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.Limelight.name);
        if (mt2 == null) return null; // Pose not found
        if (!(Math.abs(rotationRate) < 360) || mt2.tagCount <= 0) return null;

        SmartDashboard.putNumber("Limelight X", mt2.pose.getX());
        SmartDashboard.putNumber("Limelight Y", mt2.pose.getY());
        SmartDashboard.putNumber("Limelight Rotation", mt2.pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Limelight latency", mt2.latency);
        SmartDashboard.putNumber("Limelight tag count", mt2.tagCount);
        SmartDashboard.putBoolean("Limelight has target", hasTarget());
        return mt2;
    }

	public void setLEDMode(LEDMode mode) {
        switch (mode) {
            case PIPELINE -> LimelightHelpers.setLEDMode_PipelineControl(this.name);
            case OFF -> LimelightHelpers.setLEDMode_ForceOff(this.name);
            case BLINK -> LimelightHelpers.setLEDMode_ForceBlink(this.name);
            case ON -> LimelightHelpers.setLEDMode_ForceOn(this.name);
        }
    }

    @Override
    public void periodic() {
        
    }
}
