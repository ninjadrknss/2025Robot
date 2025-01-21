package frc.robot.subsystems.auton;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutonSubsystem {
    private final AutoChooser autoChooser = new AutoChooser();

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

        autoChooser.addRoutine("TestAuton", this::getTestAuton);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static AutonSubsystem getInstance() {
        if (instance == null) instance = new AutonSubsystem();
        return instance;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }

    private AutoRoutine getTestAuton() {
        AutoRoutine routine = autoFactory.newRoutine("TestAuton");
        AutoTrajectory trajectory = routine.trajectory("TestAuton");

        routine.active().onTrue(
            trajectory.resetOdometry().andThen(
                trajectory.cmd()
            )
        );
        return routine;
    }

    public Command getSelectedAuton() {
        return autoChooser.selectedCommand();
    }
}
