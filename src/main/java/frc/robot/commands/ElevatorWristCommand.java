package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.util.ControlBoard;
import frc.robot.util.FieldConstants.GameElement.ScoreLevel;

public class ElevatorWristCommand extends Command {
    private final Superstructure superstructure = Superstructure.getInstance();

    private final boolean reinitialize;

    private ScoreLevel scoreLevel;

    public ElevatorWristCommand() {
        this.reinitialize = true;
        addRequirements(ElevatorWristSubsystem.getInstance());
    }

    public ElevatorWristCommand(boolean reinitialize) {
        this.reinitialize = reinitialize;
        addRequirements(ElevatorWristSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        scoreLevel = ControlBoard.getInstance().scoreLevel;
        switch (scoreLevel) {
//            case L1 -> superstructure.requestL1Score();
            case L2 -> superstructure.requestL2Score();
            case L3 -> superstructure.requestL3Score();
//            case L4 -> elevatorWristSubsystem.requestL4Score();
        }
    }

    @Override
    public void execute() {
        if (reinitialize && ControlBoard.getInstance().scoreLevel != scoreLevel) {
            initialize(); // Reinitialize if the score level changes mid-command
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ElevatorWristCommand ended: " + scoreLevel);
//        Commands.sequence(new WaitCommand(0.75), new InstantCommand(superstructure::requestIdle, superstructure)).schedule();
        superstructure.requestIdle();
    }
}
