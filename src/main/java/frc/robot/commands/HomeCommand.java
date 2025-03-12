package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.Superstructure;


public class HomeCommand extends Command {
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();

    public HomeCommand() {
        addRequirements(this.elevatorWristSubsystem);
    }

    @Override
    public void initialize() {
        elevatorWristSubsystem.requestHome();
    }
}
