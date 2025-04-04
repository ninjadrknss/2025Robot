package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class ScoreCommand extends Command {
    public enum Level {
//        L1,
        L2,
        L3,
        L4
    }

    private final Superstructure superstructure = Superstructure.getInstance();
    private final Level level;
    private final Debouncer atPositionDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);
    private final Debouncer coralDebouncer = new Debouncer(0.75, Debouncer.DebounceType.kFalling);
    private boolean hasStartedSpitting;

    public final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public ScoreCommand(Level level) {
        this.level = level;

        addRequirements(this.superstructure, this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        switch (level) {
            case L2 -> superstructure.requestL2Score();
            case L3 -> superstructure.requestL3Score();
            case L4 -> superstructure.requestL4Score();
        }
        atPositionDebouncer.calculate(false);
        coralDebouncer.calculate(true);
        hasStartedSpitting = false;
    }

    @Override
    public void execute() {
        if (atPositionDebouncer.calculate(superstructure.isAtPosition()) && !hasStartedSpitting) {
            System.out.println("at position, starting spit");
            hasStartedSpitting = true;
            intakeSubsystem.requestSpit();
        }
    }

    @Override
    public boolean isFinished() {
        return !coralDebouncer.calculate(IntakeSubsystem.getInstance().coralDetected());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("score command done");
        intakeSubsystem.requestIdle();
        superstructure.requestIdle();
    }
}
