package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.Odometry;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.robot.util.Constants;
import java.util.List;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import frc.robot.subsystems.drive.SwerveConstants;
//import pathplanner's pathfinding algorithm
import com.pathplanner.lib.pathfinding.LocalADStar;

public class MoveCommand extends Command {
    private final Pose2d targetPose;
    private final List<Pose2d> intermediatePoints;
    private final SwerveSubsystem swerveSubsystem;

    private PathPlannerPath path;
    private Command pathCommand;
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyFieldSpeeds;
    private final PreciseMoveCommand preciseMoveCommand;
    private boolean isPreciseMove = false;

    // New fields for continuous timeout check
    private double lastMovedTime;
    private Pose2d lastMovedPose;
    private boolean timeoutTriggered = false;

    public MoveCommand(Pose2d targetPose, List<Pose2d> intermediatePoints, SwerveSubsystem swerveSubsystem) {
        this.targetPose = targetPose;
        this.intermediatePoints = new ArrayList<>(intermediatePoints);
        this.swerveSubsystem = swerveSubsystem;
        this.m_pathApplyFieldSpeeds = new SwerveRequest.ApplyRobotSpeeds();
        this.preciseMoveCommand = new PreciseMoveCommand(targetPose);
        LocalADStar pathfinder = new LocalADStar();
        Pathfinding.setPathfinder(pathfinder);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Record the initial pose and time for the continuous timeout feature.
        Pose2d currentPose = swerveSubsystem.getPose();
        this.lastMovedPose = currentPose;
        this.lastMovedTime = Timer.getFPGATimestamp();

        if (distance(currentPose, targetPose) < 0.05 &&
            Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 3) {
            end(true);
            return;
        }
        if (intermediatePoints.size() == 0) intermediatePoints.add(currentPose);
        intermediatePoints.add(targetPose);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(intermediatePoints);
        
        PathConstraints constraints = new PathConstraints(1.0, 1.0, 4 * Math.PI, 2 * Math.PI); // The constraints for this path.
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(1, targetPose.getRotation()), 
            new GoalEndState(1.0, targetPose.getRotation()) 
        );

        path.preventFlipping = true;

        PathFollowingController controller = new PPHolonomicDriveController(
            new PIDConstants(0.25, 0.0, 0.0),
            new PIDConstants(1, 0.0, 0.0)
        );
        //AutoBuilder.configure(swerveSubsystem::getPose, null, swerveSubsystem::getChassisSpeeds, this::drive, controller, SwerveConstants.robotConfig, () -> false, swerveSubsystem);
        pathCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
        pathCommand.initialize();
    }

    private void drive(ChassisSpeeds robotSpeeds, DriveFeedforwards feedforward) {
        swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(robotSpeeds));
    }

    private double distance(Pose2d current, Pose2d target) {
        return Math.hypot(current.getX() - target.getX(), current.getY() - target.getY());
    }

    @Override
    public void execute() {
        // Continuously check if the robot has moved at least 5cm within any 3-second period.
        double currentTime = Timer.getFPGATimestamp();
        Pose2d currentPose = swerveSubsystem.getPose();
        
        if (distance(lastMovedPose, currentPose) >= 0.05) {
            // Robot has moved significantly, update the reference.
            lastMovedPose = currentPose;
            lastMovedTime = currentTime;
        } else if (currentTime - lastMovedTime > 3.0) {
            // The robot hasn't moved at least 5cm in the last 3 seconds.
            timeoutTriggered = true;
        }
        
        // If timeout was triggered, skip further execution.
        if (timeoutTriggered) {
            return;
        }
        
        // Existing execution logic.
        if (pathCommand != null && !pathCommand.isFinished()) {
            pathCommand.execute();
        } else if ((pathCommand == null || pathCommand.isFinished()) && preciseMoveCommand != null && !preciseMoveCommand.isFinished()) {
            if (!isPreciseMove) {
                isPreciseMove = true;
                preciseMoveCommand.initialize();
            }
            preciseMoveCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return timeoutTriggered ||
               (pathCommand != null && pathCommand.isFinished() && preciseMoveCommand != null && preciseMoveCommand.isFinished());
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.end(interrupted);
        }
        if (preciseMoveCommand != null) {
            preciseMoveCommand.end(interrupted);
        }
    }
}
