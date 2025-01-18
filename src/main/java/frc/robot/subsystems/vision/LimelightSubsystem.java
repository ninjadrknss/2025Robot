package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.*;
import frc.robot.subsystems.vision.VisionConstants;

import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
    public enum LEDMode {
		PIPELINE,
		OFF,
		BLINK,
		ON
    }

    private static LimelightSubsystem instance;

    private final String name;

    public boolean forceLedOff = false;

    public static LimelightSubsystem getInstance() {
        if (instance == null) instance = new LimelightSubsystem();
        return instance;
    }

	private LimelightSubsystem() {
        name = VisionConstants.Limelight.name;
    }

    public void toggleLeds() {
        forceLedOff = !forceLedOff;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(this.name);
    }

    public double[] getValues() {
        return hasTarget() ? LimelightHelpers.getBotPose_wpiBlue(this.name) : null;
    }

    public double getYaw() {
        return LimelightHelpers.getBotPose2d(this.name).getRotation().getDegrees();
    }

	public PoseEstimate getPoseEstimate(Pose2d previousRobotPose, double rotationRate, boolean resetRobotRotation) {

        if (resetRobotRotation) LimelightHelpers.SetRobotOrientation("limelight", 0, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(rotationRate) < 360 || mt2.tagCount > 0 || mt2.pose.getTranslation().getDistance(previousRobotPose.getTranslation()) < 1) { // TODO: rotation rate has to be less than 1 rotation per second and the pose has to be less than 1 meter off
            SmartDashboard.putNumber("Limelight X", mt2.pose.getX());
            SmartDashboard.putNumber("Limelight Y", mt2.pose.getY());
            return mt2;
        }
        return null;
	}

    public PoseEstimate getPoseEstimate(Pose2d previousRobotPose, double rotationRate) {
        return getPoseEstimate(previousRobotPose, rotationRate, false);
    }

    /**
     * 0 is x, 1 is y, 2 z, 3 rotation x, 4 rotation y, 5 rotation z
     * @return Pose2d, null if no target
     */
	public Pose2d getPose() {
		double[] values = getValues();
        return values != null ? new Pose2d(values[0], values[1], new Rotation2d(getYaw())) : null;
    }

	public void setLEDMode(LEDMode mode) {
        if (forceLedOff) mode = LEDMode.OFF;
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
