package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.Superstructure;


public class GroundIntakeCommand extends Command {
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final Superstructure superstructure;

    public GroundIntakeCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(this.elevatorWristSubsystem);
    }

    @Override
    public void initialize() {
        superstructure.requestGroundIntake();
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
