package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.ControlBoard;

public class ActionCommand extends Command {
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final Action action;
    private boolean actionPerformed = false;

    public enum Action {
        HOME, INTAKE, SPIT, // TODO: I think intake can be handled fully by driver, no point for spit as that is just handled by smt else -Alex
        SCOREL1, SCOREL2, SCOREL3, SCOREL4, SCORE_BARGE, // TODO: add algae in between levels HAHAHA
        PREPAREL1, PREPAREL2, PREPAREL3, PREPAREL4, PREPARE_BARGE, // TODO: prob nuke these as all action commands are prepare only -Alex
        PREPARE_SELECTED
    }

    public ActionCommand(Action action) {
        this.action = action; // TODO: Probably should decide if this selection is handled by ControlBoard or by object creation as it just feels sloppy atm - Alex
        addRequirements(elevatorWristSubsystem);
    }

    @Override
    public void initialize() {
        actionPerformed = false;

        if (action == Action.PREPARE_SELECTED) {
            performElevatorAction(
                switch (ControlBoard.getInstance().scoreLevel) {
                    case L1 -> Action.PREPAREL1;
                    case L2 -> Action.PREPAREL2;
                    case L3 -> Action.PREPAREL3;
                    case L4 -> Action.PREPAREL4;
                    case BARGE -> Action.PREPARE_BARGE;
                }
            );
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
            case SCORE_BARGE, PREPARE_BARGE -> elevatorWristSubsystem.requestBargeScore();
            case INTAKE, HOME -> elevatorWristSubsystem.requestHome();
            default -> {}
        }
    }

    @Override
    public void execute() {
        if (!actionPerformed && elevatorWristSubsystem.isAtPosition()) { // TODO: i kinda want this handled by driver manually instead of this automated stuff - Alex
            switch (action) {
                case INTAKE -> intakeSubsystem.requestIntake();
                case SCOREL1, SCOREL2, SCOREL3, SCOREL4, SCORE_BARGE -> intakeSubsystem.requestSpit();
                default -> {}
            }
            actionPerformed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return actionPerformed && intakeSubsystem.isDone(); // TODO: prob should always just be false -Alex
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ActionCommand ended: " + action);
        // TODO: maybe request EWS home? -Alex
    }
}
