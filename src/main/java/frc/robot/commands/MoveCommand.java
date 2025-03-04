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
        Pose2d currentPose = swerveSubsystem.getPose();
        if (distance(currentPose, targetPose) < 0.05 && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 3) {
            end(true);
            return;
        }
        if (intermediatePoints.size() == 0) intermediatePoints.add(currentPose);
        intermediatePoints.add(targetPose);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(intermediatePoints);
        
        PathConstraints constraints = new PathConstraints(6.0, 3.0, 4 * Math.PI, 2 * Math.PI); // The constraints for this path.
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(6, targetPose.getRotation()), 
            new GoalEndState(1.0, targetPose.getRotation()) 
        );

        path.preventFlipping = true;

        PathFollowingController controller = new PPHolonomicDriveController(
            new PIDConstants(1, 0.0, 0.0),
            new PIDConstants(2, 0.05, 0.0)
        );
        AutoBuilder.configure(swerveSubsystem::getPose, null, swerveSubsystem::getChassisSpeeds, this::drive, controller, SwerveConstants.robotConfig, () -> false, swerveSubsystem);
        pathCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);


        //Pathfinding.getCurrentPath(null, null).generateTrajectory(null, null, null).getTotalTimeSeconds();
        //pathCommand = AutoBuilder.pathfindToPose(intermediatePoints.get(0), constraints, 0);

        
        
        //(new Pose2d(8, 6, new Rotation2d(30)), constraints, 2.0);
        
        //(new Pose2d(8, 6, new Rotation2d(30)), constraints);

        //pathCommand = new FollowPathCommand(path, swerveSubsystem::getPose, swerveSubsystem::getChassisSpeeds, this::drive, controller, SwerveConstants.robotConfig, () -> false, swerveSubsystem);   
        pathCommand.initialize();
        
    }

    private void drive(ChassisSpeeds robotSpeeds, DriveFeedforwards feedforward) {
        swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(robotSpeeds));
    }

    private double distance(Pose2d current, Pose2d target) {
        double positionError = Math.hypot(current.getX() - target.getX(), current.getY() - target.getY());
        return positionError;
    }

    @Override
    public void execute() {
        if (pathCommand != null && !pathCommand.isFinished()) {
            pathCommand.execute();
        }
        else if ((pathCommand == null || pathCommand.isFinished()) && preciseMoveCommand != null && !preciseMoveCommand.isFinished()) {
            if (!isPreciseMove){
                isPreciseMove = true;
                preciseMoveCommand.initialize();
            }
           preciseMoveCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand != null && pathCommand.isFinished() && preciseMoveCommand != null && preciseMoveCommand.isFinished();
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
