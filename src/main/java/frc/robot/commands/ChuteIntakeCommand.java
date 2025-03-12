package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class ChuteIntakeCommand extends Command {
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public ChuteIntakeCommand() {
        addRequirements(this.elevatorWristSubsystem);
    }

    @Override
    public void initialize() {
        elevatorWristSubsystem.requestChuteIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.coralBeamBroken(); // Maybe
    }
}
