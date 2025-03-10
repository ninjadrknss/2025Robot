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
import frc.robot.util.ControlBoard;
import frc.robot.util.FieldConstants.GameElement;
import frc.robot.util.FieldConstants.GameElement.*;

public class AssistCommand extends Command {
    //private final ElevatorWristSubsystem elevatorWristSubsystem = ElevatorWristSubsystem.getInstance();
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private Command goToPositionCommand;

    private GameElement gameElement;

    private  Branch selectedBranch = null;

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
        addRequirements(this.swerve);
    }
    

    public AssistCommand(Superstructure superstructure, Branch selectedBranch, boolean firstWaypoint, boolean secondWaypoint) {
        this.selectedBranch = selectedBranch;
        this.firstWaypoint = firstWaypoint;
        this.secondWaypoint = secondWaypoint;
        addRequirements(this.swerve);
    }

    public AssistCommand(Superstructure superstructure, boolean firstWaypoint, boolean secondWaypoint) {
        this.selectedBranch = null;
        this.firstWaypoint = firstWaypoint;
        this.secondWaypoint = secondWaypoint;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        gameElement = ControlBoard.getInstance().desiredGoal;
        Pose2d elementPose = gameElement.getCenter();
        //elementPose = gameElement.getRightBranch();
        if (gameElement.hasBranches() && selectedBranch == null) {
            selectedBranch = closestBranch(swerve.getPose(), gameElement);
        } 

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

    private Branch closestBranch(Pose2d currentPose, GameElement gameElement) {
        double leftDistance = currentPose.getTranslation().getDistance(gameElement.getLeftBranch().getTranslation());
        double rightDistance = currentPose.getTranslation().getDistance(gameElement.getRightBranch().getTranslation());
        return leftDistance < rightDistance ? Branch.LEFT : Branch.RIGHT;
    }

    @Override
    public void execute() {
		goToPositionCommand.execute();
        ControlBoard.getInstance().isAssisting = true;
        ControlBoard.getInstance().previousConfirmedGoal = gameElement;
    }

    @Override
    public boolean isFinished() {
        return goToPositionCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        ControlBoard.getInstance().isAssisting = false;
        System.out.println("Assist Command Ended");
    }
}
