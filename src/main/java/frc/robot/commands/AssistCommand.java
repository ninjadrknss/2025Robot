package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.ControlBoard;
import frc.robot.util.FieldConstants.GameElement;
import frc.robot.util.FieldConstants.GameElement.*;

public class AssistCommand extends Command {
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private MoveCommand goToPositionCommand;

    private GameElement gameElement;

    private Branch selectedBranch = null;

    private boolean firstWaypoint = true;
    private boolean secondWaypoint = true;
    private boolean dynamic = true;

    private final StructPublisher<Pose2d> desiredPosePublisher = NetworkTableInstance.getDefault().getTable("Auton")
        .getStructTopic("Desired Pose", Pose2d.struct)
        .publish();

    public AssistCommand() {
        addRequirements(this.swerve);
    }
    

    public AssistCommand(boolean firstWaypoint, boolean secondWaypoint) {
        this.firstWaypoint = firstWaypoint;
        this.secondWaypoint = secondWaypoint;
        addRequirements(this.swerve);
    }

    public AssistCommand(GameElement gameElement, Branch selectedBranch) {
        this(true, true);
        this.gameElement = gameElement;
        this.selectedBranch = selectedBranch;
        dynamic = false;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Assist Command Active", true);
        System.out.println("Assist Command Active");
        if (dynamic) gameElement = ControlBoard.getInstance().desiredGoal;
        Pose2d elementPose = gameElement.getCenter();

        if (gameElement.hasBranches()) {
            if (dynamic) selectedBranch = ControlBoard.getInstance().selectedBranch;
            if (selectedBranch != Branch.CENTER)
                elementPose = selectedBranch == Branch.LEFT ? gameElement.getLeftBranch() : gameElement.getRightBranch();
        }

        Rotation2d targetRotation = gameElement.getLocation()
            .getRotation()
            .minus(Rotation2d.k180deg);

        Pose2d offsetPose1 = GameElement.getPoseWithOffset(elementPose, 0.45);
        
        List<Pose2d> waypoints = new ArrayList<>();

        if (firstWaypoint) waypoints.add(new Pose2d(GameElement.getPoseWithOffset(elementPose, 1.2).getX(), GameElement.getPoseWithOffset(elementPose, 1.0).getY(), targetRotation));
        if (secondWaypoint) waypoints.add(new Pose2d(GameElement.getPoseWithOffset(elementPose, 0.85).getX(), GameElement.getPoseWithOffset(elementPose, 0.6).getY(), targetRotation));

        Pose2d targetPose = new Pose2d(offsetPose1.getX(), offsetPose1.getY(), targetRotation);
        
        goToPositionCommand = new MoveCommand(targetPose, waypoints, swerve);

        desiredPosePublisher.set(targetPose);
        goToPositionCommand.initialize();
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
        SmartDashboard.putBoolean("Assist Command Active", false);
    }
}
