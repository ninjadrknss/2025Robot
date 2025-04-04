package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        LimelightHelpers.setCameraPose_RobotSpace(name, VisionConstants.Limelight.yOffset.in(Units.Meters), 0, VisionConstants.Limelight.zOffset.in(Units.Meters), 0, 0, 0);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(this.name);
    }
    //TODO: Maybe?
    //MegaTag Standard Deviations [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
    public double[] getSTD(){
        //return NetworkTableInstance.getDefault().getTable(name).getEntry("stddevs").getDoubleArray(new double[12]);
        return LimelightHelpers.getSTD(name);
    }

    public PoseEstimate getPoseEstimate(RobotState previousRobotState, boolean useMegaTag2) {
        Pose2d previousRobotPose = previousRobotState.getPose();

        double rotationRate = previousRobotState.getAngularVelocity().yaw();
        LimelightHelpers.SetRobotOrientation(
                VisionConstants.Limelight.name,
                previousRobotPose.getRotation().getDegrees(),
                0, 0, 0, 0, 0
        );
        
        LimelightHelpers.PoseEstimate mt2 = null;
        if (useMegaTag2) mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.Limelight.name);
        else mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.Limelight.name);

        if (mt2 == null) return null; // Pose not found
        //if (!(Math.abs(rotationRate) < 360) || mt2.tagCount <= 0) return null;

        SmartDashboard.putNumber("Limelight/Limelight X", mt2.pose.getX());
        SmartDashboard.putNumber("Limelight/Limelight Y", mt2.pose.getY());
        SmartDashboard.putNumber("Limelight/Limelight Rotation", mt2.pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Limelight/Limelight latency", mt2.latency);
        SmartDashboard.putNumber("Limelight/Limelight tag count", mt2.tagCount);
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
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.Limelight.name, new int[]{ 6,  7,  8,  9, 10, 11,
                                                                                                                        17, 18, 19, 20, 21, 22});
        else if (alliance.get().equals(DriverStation.Alliance.Blue)) LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.Limelight.name, new int[]{17, 18, 19, 20, 21, 22});
        else if (alliance.get().equals(DriverStation.Alliance.Red)) LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.Limelight.name, new int[]{6, 7, 8, 9, 10, 11});
    }

    public void setAprilTagFilters() {
    }
}
