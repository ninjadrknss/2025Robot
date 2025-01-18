package frc.robot.subsystems.auton;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutonSubsystem {
    private final SendableChooser<Command> autonSelector = new SendableChooser<>();

    private final AutoFactory autoFactory;

    private final Superstructure superstructure;
    private final SwerveSubsystem swerveSubsystem;

    private static AutonSubsystem instance;

    private AutonSubsystem() {
        superstructure = Superstructure.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();

        autoFactory = new AutoFactory(
            swerveSubsystem::getPose,
            swerveSubsystem::resetPose,
            swerveSubsystem::followPath,
            true,
            swerveSubsystem
        );

        String testAutonName = "TestAuton";
        autonSelector.setDefaultOption(
            testAutonName,
            getTestAuton(testAutonName)
        );
    }

    public static AutonSubsystem getInstance() {
        if (instance == null) instance = new AutonSubsystem();
        return instance;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }

    private Command getTestAuton(String name) {
        return Commands.sequence(
            autoFactory.resetOdometry(name),
            autoFactory.trajectoryCmd(name)
        );
    }

    public Command getSelectedAuton() {
        return autonSelector.getSelected();
    }
}
