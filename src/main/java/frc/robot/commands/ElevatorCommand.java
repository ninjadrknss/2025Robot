package frc.robot.commands;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final Superstructure superstructure;
    
    public ElevatorCommand(){
        this.superstructure = Superstructure.getInstance();
        addRequirements(this.elevatorWristSubsystem);
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
