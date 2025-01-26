package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorarm.ElevatorArmSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.Constants;

public class AssistCommand extends Command {
    private final ElevatorArmSubsystem elevatorArmSubsystem = ElevatorArmSubsystem.getInstance();
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private final Superstructure superstructure;
    private Command goToPositionCommand;

    public AssistCommand(Superstructure superstructure) {
        this.superstructure = superstructure;
        Constants.GameElement gameElement = Constants.GameElement.CORAL_STATION_BLUE_1;
        this.goToPositionCommand = swerve.goToPositionCommand(new Pose2d(gameElement.getPoseWithOffset(0.5).getX(), gameElement.getPoseWithOffset(0.5).getY(), gameElement.getLocation().getRotation().minus(Rotation2d.fromDegrees(180))));
        addRequirements(this.elevatorArmSubsystem, this.swerve);
    }

    @Override
    public void initialize() {

        goToPositionCommand.initialize();
        //superstructure.requestChuteIntake();
    }

    @Override
    public void execute() {
        goToPositionCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return goToPositionCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
