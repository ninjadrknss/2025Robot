package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.ControlBoard;

public class ScoreCommand extends Command {

    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final Action action;
    private boolean actionPerformed = false;

    public enum Action {
        HOME, INTAKE, SPIT,
        SCOREL1, SCOREL2, SCOREL3, SCOREL4, SCOREBARGE,
        PREPAREL1, PREPAREL2, PREPAREL3, PREPAREL4, PREPAREBARGE,
        PREPARESELECTED
    }

    public ScoreCommand(Action action) {
        this.action = action;
        addRequirements(elevatorWristSubsystem);
    }

    @Override
    public void initialize() {
        actionPerformed = false;

        if (action == Action.PREPARESELECTED) {
            performElevatorAction(switch (ControlBoard.getInstance().scoreLevel) {
                case L1 -> Action.PREPAREL1;
                case L2 -> Action.PREPAREL2;
                case L3 -> Action.PREPAREL3;
                case L4 -> Action.PREPAREL4;
                case BARGE -> Action.PREPAREBARGE;
            });
        } else {
            performElevatorAction(action);
        }

        if (action == Action.SPIT) {
            intakeSubsystem.requestSpit();
        }
    }

    private void performElevatorAction(Action action) {
        switch (action) {
            case SCOREL1, PREPAREL1 -> elevatorWristSubsystem.requestL1Score();
            case SCOREL2, PREPAREL2 -> elevatorWristSubsystem.requestL2Score();
            case SCOREL3, PREPAREL3 -> elevatorWristSubsystem.requestL3Score();
            case SCOREL4, PREPAREL4 -> elevatorWristSubsystem.requestL4Score();
            case SCOREBARGE, PREPAREBARGE -> elevatorWristSubsystem.requestBargeScore();
            case INTAKE, HOME -> elevatorWristSubsystem.requestHome();
            default -> {}
        }
    }

    @Override
    public void execute() {
        if (!actionPerformed && elevatorWristSubsystem.isAtPosition()) {
            switch (action) {
                case INTAKE -> intakeSubsystem.requestIntake();
                case SCOREL1, SCOREL2, SCOREL3, SCOREL4, SCOREBARGE -> intakeSubsystem.requestSpit();
                default -> {}
            }
            actionPerformed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return actionPerformed && intakeSubsystem.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ActionCommand ended: " + action);
    }
}
