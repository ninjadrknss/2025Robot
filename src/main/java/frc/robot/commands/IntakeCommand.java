package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class IntakeCommand extends Command {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public IntakeCommand() {
        addRequirements(ElevatorWristSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        // elevatorWristSubsystem.requestChuteIntake();
        intakeSubsystem.requestIntake();
    }

    @Override
    public boolean isFinished() {
        // return intakeSubsystem.coralBeamBroken(); // Maybe
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.requestIdle();
    }
}
