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
    private boolean atPosition = false;
    private final Debouncer coralDebouncer = new Debouncer(0.75, Debouncer.DebounceType.kFalling);
    private boolean coralDetected = false;

    private static final SpitCommand spitCommand = new SpitCommand();

    public ScoreCommand(Level level) {
        this.level = level;

        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.superstructure);
    }

    @Override
    public void initialize() {
        switch (level) {
            case L2 -> superstructure.requestL2Score();
            case L3 -> superstructure.requestL3Score();
            case L4 -> superstructure.requestL4Score();
        }
        atPositionDebouncer.calculate(false);
    }

    @Override
    public void execute() {
        atPosition = atPositionDebouncer.calculate(superstructure.isAtPosition());
        coralDetected = coralDebouncer.calculate(IntakeSubsystem.getInstance().coralDetected());

        if (atPosition && !spitCommand.isScheduled()) spitCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return atPosition && !coralDetected;
    }

    @Override
    public void end(boolean interrupted) {
        spitCommand.cancel();
        superstructure.requestIdle();
    }
}
