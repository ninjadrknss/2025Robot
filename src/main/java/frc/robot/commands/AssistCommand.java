package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorarm.ElevatorArmSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AssistCommand extends Command {
    private final ElevatorArmSubsystem elevatorArmSubsystem = ElevatorArmSubsystem.getInstance();
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private final Superstructure superstructure;

    public AssistCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(this.elevatorArmSubsystem, this.swerve);
    }

    @Override
    public void initialize() {
        superstructure.requestChuteIntake();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
