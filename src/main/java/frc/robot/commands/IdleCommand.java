package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;


public class IdleCommand extends Command {
    private final Superstructure superstructure = Superstructure.getInstance();

    public IdleCommand() {
        addRequirements(ElevatorWristSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        superstructure.requestIdle();
    }
}
