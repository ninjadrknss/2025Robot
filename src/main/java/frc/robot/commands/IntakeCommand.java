package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class IntakeCommand extends Command {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    private final Debouncer coralBeamBreakDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);

    public IntakeCommand() {
        addRequirements(ElevatorWristSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        superstructure.requestChuteIntake();
        intakeSubsystem.requestIntake();
    }

    @Override
    public boolean isFinished() {
        return coralBeamBreakDebouncer.calculate(intakeSubsystem.algaeDetected()); // Maybe
//        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        intakeSubsystem.requestIdle();
        superstructure.requestIdle();
    }
}
