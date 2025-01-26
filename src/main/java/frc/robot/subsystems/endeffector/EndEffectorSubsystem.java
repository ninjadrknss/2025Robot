package frc.robot.subsystems.endeffector;


import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
    private static EndEffectorSubsystem instance;

    private final LaserCan laserCan;

    public static EndEffectorSubsystem getInstance() {
        if (instance == null) instance = new EndEffectorSubsystem();
        return instance;
    }

    private EndEffectorSubsystem() {
        // TODO: implement
        // Possible states? Spitting, Intaking, Idle

        laserCan = new LaserCan(EndEffectorConstants.LaserCanID);

        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(EndEffectorConstants.roi);
            laserCan.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_50MS);
        } catch (ConfigurationFailedException e) {
            System.err.println("Failed to configure laserCan");
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("EndEffector/Distance", laserCan.getMeasurement().distance_mm);
        SmartDashboard.putBoolean("EndEffector/Beam Broken", isBeamBroken());
    }

    public boolean isBeamBroken() {
        return laserCan.getMeasurement().distance_mm < 100;
    }
}
