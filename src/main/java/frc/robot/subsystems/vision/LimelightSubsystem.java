package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.*;
import frc.robot.subsystems.drive.Odometry.RobotState;
import frc.robot.subsystems.vision.VisionConstants.Limelight;

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
        name = Limelight.name;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(this.name);
    }

	public PoseEstimate getPoseEstimate(RobotState previousRobotState, boolean resetRobotRotation) {
        Pose2d previousRobotPose = previousRobotState.getPose();

        double rotationRate = previousRobotState.getAngularVelocity().yaw();
        if (resetRobotRotation) LimelightHelpers.SetRobotOrientation(
                Limelight.name,
                previousRobotPose.getRotation().getDegrees(),
                rotationRate, 0, 0, 0, 0
        );

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Limelight.name);
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

    public PoseEstimate getPoseEstimate(RobotState previousRobotState) {
        return getPoseEstimate(previousRobotState, false);
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
