package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.ControlBoard;
import frc.robot.util.Constants.GameElement;
import frc.robot.util.Constants.GameElement.*;

public class AssistCommand extends Command {
    //private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private Command goToPositionCommand;

    private Constants.GameElement gameElement;

    private final Branch selectedBranch;

    private boolean firstWaypoint = true;
    private boolean secondWaypoint = true;

    private final StructPublisher<Pose2d> desiredPosePublisher = NetworkTableInstance.getDefault().getTable("Auton")
        .getStructTopic("Desired Pose", Pose2d.struct)
        .publish();

    public AssistCommand(Superstructure superstructure, Branch selectedBranch) {
        this.selectedBranch = selectedBranch;
        addRequirements(this.swerve);
    }

    public AssistCommand(Superstructure superstructure) {
        this.selectedBranch = Branch.CENTER;
        addRequirements(this.swerve);
    }


    @Override
    public void initialize() {

        gameElement = ControlBoard.getInstance().desiredGoal;
        Pose2d elementPose = gameElement.getCenter();
        //elementPose = gameElement.getRightBranch();
        if (gameElement.hasBranches() && selectedBranch != Branch.CENTER) {
            elementPose = selectedBranch == Branch.LEFT ? gameElement.getLeftBranch() : gameElement.getRightBranch();
        }

        Rotation2d targetRotation = gameElement.getLocation()
            .getRotation()
            .minus(Rotation2d.fromDegrees(180));

        Pose2d offsetPose1 = GameElement.getPoseWithOffset(elementPose, 0.475);
        
        List<Pose2d> waypoints = new ArrayList<>();

        if (firstWaypoint) waypoints.add(new Pose2d(GameElement.getPoseWithOffset(elementPose, 1.0).getX(), GameElement.getPoseWithOffset(elementPose, 1.0).getY(), targetRotation));
        if (secondWaypoint) waypoints.add(new Pose2d(GameElement.getPoseWithOffset(elementPose, 0.6).getX(), GameElement.getPoseWithOffset(elementPose, 0.6).getY(), targetRotation));

        Pose2d targetPose = new Pose2d(offsetPose1.getX(), offsetPose1.getY(), targetRotation);
        
        goToPositionCommand = swerve.goToPositionCommand(targetPose, waypoints);

        desiredPosePublisher.set(targetPose);
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
