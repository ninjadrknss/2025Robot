package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.Odometry;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class MoveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final Odometry odometry = Odometry.getInstance();

    private Pose2d targetPose;
    private Trajectory trajectory;

    //private final HolonomicDriveController holonomicDriveController;
    private final Timer timer = new Timer();

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyFieldSpeeds;

    // make three profiled PId controllers for x, y, and theta
    private ProfiledPIDController xController = new ProfiledPIDController(
        1.5, 0.12, 0,
        SwerveConstants.AutoConstants.kXControllerConstraints
    );
    private ProfiledPIDController yController = new ProfiledPIDController(
        1.5, 0.12, 0,
        SwerveConstants.AutoConstants.kYControllerConstraints
    );

    private ProfiledPIDController thetaController;

    public MoveCommand(Pose2d targetPose) {
        this.targetPose = targetPose;

        this.m_pathApplyFieldSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        // Initialize the HolonomicDriveController with PID and Profiled PID controllers

        thetaController = new ProfiledPIDController(
            1, 0.05, 0,
            SwerveConstants.AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(3), Math.toRadians(3));

        
        // holonomicDriveController = new HolonomicDriveController(
        //         new PIDController(SwerveConstants.AutoConstants.kPXController, 0, 0),
        //         new PIDController(SwerveConstants.AutoConstants.kPYController, 0, 0),
        //         thetaController
        // );

        // // Enable continuous input for rotation (for swerve-specific rotational wrapping)
        // holonomicDriveController.setEnabled(true);

        addRequirements(swerveSubsystem);
    }

    @Override
public void initialize() {
    // Remove the reset of PID controllers
    System.out.println("Target Pose: " + targetPose);
    xController.reset(odometry.getPose().getX());
    yController.reset(odometry.getPose().getY());
    thetaController.reset(odometry.getPose().getRotation().getRadians());
    timer.reset();
    timer.start();
}

@Override
public void execute() {
    Pose2d currentPose = swerveSubsystem.getPose();

    // Use intended motion towards the target as feedforward
    double feedforwardVx = (targetPose.getX() - currentPose.getX()) * 0.5; // Scale feedforward by a constant factor
    double feedforwardVy = (targetPose.getY() - currentPose.getY()) * 0.5; // Scale feedforward by a constant factor
    double feedforwardOmega = 0; // Replace with desired baseline angular velocity if needed

    // Feedback control (PID)
    double feedbackVx = xController.calculate(currentPose.getX(), targetPose.getX());
    double feedbackVy = yController.calculate(currentPose.getY(), targetPose.getY());
    double feedbackOmega = thetaController.calculate(
        currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians()
    );

    // Combine feedforward and feedback
    double vx = feedforwardVx + feedbackVx;
    double vy = feedforwardVy + feedbackVy;
    double omega = feedforwardOmega + feedbackOmega;

    // Constrain velocities
    double maxVelocity = SwerveConstants.AutoConstants.kMaxSpeedMetersPerSecond;
    double maxAngularVelocity = SwerveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond;
    vx = Math.max(-maxVelocity, Math.min(maxVelocity, vx));
    vy = Math.max(-maxVelocity, Math.min(maxVelocity, vy));
    omega = Math.max(-maxAngularVelocity, Math.min(maxAngularVelocity, omega));

    // Convert field-relative to robot-relative speeds
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, currentPose.getRotation());

    // Apply the calculated speeds to the swerve subsystem
    swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(speeds));
}

private boolean isClose() {
    Pose2d currentPose = swerveSubsystem.getPose();
    double positionError = Math.hypot(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());
    double rotationError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());

    // Increase tolerances for smoother stops
    return positionError < 0.1 && rotationError < Math.toRadians(5);
}

@Override
public void end(boolean interrupted) {
    timer.stop();

    // Gradually reduce velocity to zero
    ChassisSpeeds stopSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(stopSpeeds));
}
}
