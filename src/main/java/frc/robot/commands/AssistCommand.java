package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.ControlBoard;
import frc.robot.util.Constants.GameElement;
import frc.robot.util.Constants.GameElement.*;

public class AssistCommand extends Command {
    private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private final Superstructure superstructure;
    private Command goToPositionCommand;

    Constants.GameElement gameElement;

    private final Branch selectedBranch;


    public AssistCommand(Superstructure superstructure, Branch selectedBranch) {
        this.superstructure = superstructure;
        this.selectedBranch = selectedBranch;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {

        gameElement = ControlBoard.getInstance().desiredGoal;
        Pose2d elementPose = gameElement.getCenter();

        if (gameElement.hasBranches() && selectedBranch != Branch.CENTER){
            elementPose = (selectedBranch == Branch.LEFT ? gameElement.getLeftBranch() : gameElement.getRightBranch());
        }
        Rotation2d targetRotation = gameElement.getLocation()
            .getRotation()
            .minus(Rotation2d.fromDegrees(180));


        // Get the poses with offset for the two positions
        Pose2d offsetPose1 = GameElement.getPoseWithOffset(elementPose, 0.38);
        Pose2d offsetPose2 = GameElement.getPoseWithOffset(elementPose, 1.5);
        Pose2d offsetPose3 = GameElement.getPoseWithOffset(elementPose, 0.751);

        // Create new Pose2d objects using the x/y from the offset poses and the adjusted rotation
        Pose2d intermediatePose1 = new Pose2d(offsetPose2.getX(), offsetPose2.getY(), targetRotation);
        Pose2d intermediatePose2 = new Pose2d(offsetPose3.getX(), offsetPose3.getY(), targetRotation);
        Pose2d targetPose = new Pose2d(offsetPose1.getX(), offsetPose1.getY(), targetRotation);

        this.goToPositionCommand = swerve.goToPositionCommand(targetPose, List.of(intermediatePose1, intermediatePose2));
        goToPositionCommand.initialize();
        //superstructure.requestChuteIntake();
    }

    @Override
    public void execute() {
		goToPositionCommand.execute();
		ControlBoard.getInstance().previousConfirmedGoal = gameElement;
    }

    @Override
    public boolean isFinished() {
        return goToPositionCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Assist Command Ended");
    }
}
