package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;

public class MoveCommand extends Command {
    private static final PathConstraints constraints = new PathConstraints(6.5, 3.5, 4 * Math.PI, 2 * Math.PI);

    private final Pose2d targetPose;
    private final List<Pose2d> intermediatePoints;
    private final SwerveSubsystem swerveSubsystem;

    private PathPlannerPath path;
    private Command pathCommand;
    private final PreciseMoveCommand preciseMoveCommand;

    private boolean isPreciseMove = false;
    private boolean hasEnded = false;

    public MoveCommand(Pose2d targetPose, List<Pose2d> intermediatePoints, SwerveSubsystem swerveSubsystem) {
        this.targetPose = targetPose;
        this.intermediatePoints = new ArrayList<>(intermediatePoints);
        this.swerveSubsystem = swerveSubsystem;
        this.preciseMoveCommand = new PreciseMoveCommand(targetPose);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("PP Move Command Active", true);
        Pose2d currentPose = swerveSubsystem.getPose();

        if (distance(currentPose, targetPose) < 0.05 &&
            Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 3) {
            System.out.println("Already at target, skipping move.");
            hasEnded = true;
            return;
        }

        if (intermediatePoints.isEmpty()) {
            intermediatePoints.add(currentPose);
        }
        intermediatePoints.add(targetPose);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(intermediatePoints);

        path = new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(3, targetPose.getRotation()),
            new GoalEndState(0.5, targetPose.getRotation())
        );

        path.preventFlipping = true;

        pathCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
        pathCommand.initialize();
        System.out.println("Path command initialized.");
    }

    @Override
    public void execute() {
        if (hasEnded) return;

        if (pathCommand != null && !pathCommand.isFinished()) {
            pathCommand.execute();
        } else if ((pathCommand == null || pathCommand.isFinished()) && !preciseMoveCommand.isFinished()) {
            if (!isPreciseMove) {
                isPreciseMove = true;
                preciseMoveCommand.initialize();
            } else {
                preciseMoveCommand.execute();
                System.out.println("Precise move running...");
            }
        }
    }

    @Override
    public boolean isFinished() {
        boolean finished = (pathCommand == null || pathCommand.isFinished())
                        && (preciseMoveCommand == null || preciseMoveCommand.isFinished());

        if (finished && !hasEnded) {
            hasEnded = true;
            end(false);
        }

        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("MoveCommand end() called! Interrupted: " + interrupted);
        SmartDashboard.putBoolean("PP Move Command Active", false);

        if (pathCommand != null) {
            pathCommand.end(interrupted);
        }

        if (preciseMoveCommand != null) {
            preciseMoveCommand.end(interrupted);
        }
    }

    private double distance(Pose2d current, Pose2d target) {
        return Math.hypot(current.getX() - target.getX(), current.getY() - target.getY());
    }
}
