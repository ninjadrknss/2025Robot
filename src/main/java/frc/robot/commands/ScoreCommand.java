package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;


public class ScoreCommand extends Command {
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final int level;

    public ScoreCommand(int level) {
        this.level = level;
        addRequirements(this.elevatorWristSubsystem);
    }

    @Override
    public void initialize() {
        switch (level) {
            case 1 -> elevatorWristSubsystem.requestL1Score();
            case 2 -> elevatorWristSubsystem.requestL2Score();
            case 3 -> elevatorWristSubsystem.requestL3Score();
            case 4 -> elevatorWristSubsystem.requestL4Score();
            case 5 -> elevatorWristSubsystem.requestBargeScore();
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
