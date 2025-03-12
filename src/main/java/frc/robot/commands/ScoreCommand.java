package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class ScoreCommand extends Command {
    public final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public ScoreCommand() {
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.requestSpit();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.requestIdle();
    }
}
