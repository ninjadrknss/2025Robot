package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class ScoreCommand extends Command {
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final Action action;
    private boolean hasPerformedAction = false;

    public enum Action {
        HOME,
        INTAKE,
        SPIT,
        SCOREL1,
        SCOREL2,
        SCOREL3,
        SCOREL4,
        SCOREBARGE,
        PREPAREL1,
        PREPAREL2,
        PREPAREL3,
        PREPAREL4,
        PREPAREBARGE
    }

    public ScoreCommand(Action action) {
        this.action = action;
        addRequirements(this.elevatorWristSubsystem);
    }

    @Override
    public void initialize() {
        hasPerformedAction = false;
        switch (action) {
            case SCOREL1 -> elevatorWristSubsystem.requestL1Score();
            case SCOREL2 -> elevatorWristSubsystem.requestL2Score();
            case SCOREL3 -> elevatorWristSubsystem.requestL3Score();
            case SCOREL4 -> elevatorWristSubsystem.requestL4Score();
            case SCOREBARGE -> elevatorWristSubsystem.requestBargeScore();
            case PREPAREL1 -> elevatorWristSubsystem.requestL1Score();
            case PREPAREL2 -> elevatorWristSubsystem.requestL2Score();
            case PREPAREL3 -> elevatorWristSubsystem.requestL3Score();
            case PREPAREL4 -> elevatorWristSubsystem.requestL4Score();
            case PREPAREBARGE -> elevatorWristSubsystem.requestBargeScore();
            case INTAKE -> elevatorWristSubsystem.requestHome();
            case SPIT -> intakeSubsystem.requestSpit();
            case HOME -> elevatorWristSubsystem.requestHome();
        }
    }  

    @Override
    public void execute() {
        if (elevatorWristSubsystem.isAtPosition() && !hasPerformedAction){
            switch (action) {
                case INTAKE -> intakeSubsystem.requestIntake();
                case SCOREL1 -> intakeSubsystem.requestSpit();
                case SCOREL2 -> intakeSubsystem.requestSpit();
                case SCOREL3 -> intakeSubsystem.requestSpit();
                case SCOREL4 -> intakeSubsystem.requestSpit();
                case SCOREBARGE -> intakeSubsystem.requestSpit();
                default -> end(true);
            }
            hasPerformedAction = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (hasPerformedAction && intakeSubsystem.isDone()) return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
