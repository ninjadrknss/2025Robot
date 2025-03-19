package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.ControlBoard;

public class ActionCommand extends Command {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final Action action;
    private boolean actionPerformed = false;

    public enum Action {
        HOME,
        /*SCOREL1, */SCOREL2, SCOREL3, /*SCOREL4,*/
        PREPARE_SELECTED
    }

    public ActionCommand(Action action) {
        this.action = action; // TODO: Probably should decide if this selection is handled by ControlBoard or by object creation as it just feels sloppy atm - Alex
        addRequirements(ElevatorWristSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        actionPerformed = false;

        if (action == Action.PREPARE_SELECTED) {
            performElevatorAction(
                switch (ControlBoard.getInstance().scoreLevel) {
//                    case L1 -> Action.SCOREL1;
                    case L2 -> Action.SCOREL2;
                    case L3 -> Action.SCOREL3;
//                    case L4 -> Action.SCOREL4;
                }
            );
        } else {
            performElevatorAction(action);
        }
    }

    private void performElevatorAction(Action action) {
        switch (action) {
//            case SCOREL1, PREPAREL1 -> superstructure.requestL1Score();
            case SCOREL2 -> superstructure.requestL2Score();
            case SCOREL3 -> superstructure.requestL3Score();
//            case SCOREL4, PREPAREL4 -> elevatorWristSubsystem.requestL4Score();
            case HOME -> superstructure.requestIdle();
            default -> {}
        }
    }

    @Override
    public void execute() {
        if (!actionPerformed && superstructure.isAtPosition()) { // TODO: i kinda want this handled by driver manually instead of this automated stuff - Alex
            switch (action) {
                case /*SCOREL1,*/ SCOREL2, SCOREL3/*, SCOREL4*/ -> intakeSubsystem.requestSpit();
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
        switch (action) {
            case /*SCOREL1,*/ SCOREL2, SCOREL3/*, SCOREL4,*/ -> superstructure.requestIdle();
            default -> {}
        }
    }
}
