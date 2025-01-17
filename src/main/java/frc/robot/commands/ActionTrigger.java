package frc.robot.commands;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.elevatorarm.ElevatorArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ActionTrigger extends Command {
    private final ElevatorArmSubsystem elevatorArmSubsystem = ElevatorArmSubsystem.getInstance();
    private final Superstructure superstructure;
    
    public ActionTrigger(){
        this.superstructure = Superstructure.getInstance();
        addRequirements(this.elevatorArmSubsystem);
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
