package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Superstructure;


public class ScoreCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private final Superstructure superstructure;
    private final int level;

    public ScoreCommand(Superstructure superstructure, int level) {
        this.superstructure = superstructure;
        this.level = level;
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {
        switch (level) {
            case 1 -> superstructure.requestL1Score();
            case 2 -> superstructure.requestL2Score();
            case 3 -> superstructure.requestL3Score();
            case 4 -> superstructure.requestL4Score();
            case 5 -> superstructure.requestBargeScore();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
