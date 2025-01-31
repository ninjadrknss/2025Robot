package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class AssistCommand extends Command {
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private final Superstructure superstructure;

    public AssistCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(this.elevatorWristSubsystem, this.swerve);
    }

    @Override
    public void initialize() {

        swerve.goToPositionCommand(new Pose2d(0, 0, new Rotation2d(0))).initialize();
        //superstructure.requestChuteIntake();
    }

    @Override
    public void execute() {
        swerve.goToPositionCommand(new Pose2d(0, 0, new Rotation2d(0))).execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
