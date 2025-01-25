package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.Odometry;


public class MoveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private Pose2d targetPose;

    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds;
    private final Odometry m_odometry = Odometry.getInstance();

    private final PIDController m_pathXController;
    private final PIDController m_pathYController;
    private final PIDController m_pathThetaController;

    

    public MoveCommand(Pose2d targetPose, PIDController pathXController, PIDController pathYController, PIDController pathThetaController) {
        this.targetPose = targetPose;

        this.m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

        this.m_pathXController = new PIDController(0.5, pathXController.getI(), pathXController.getD());
        this.m_pathYController = new PIDController(0.5, pathYController.getI(), pathYController.getD());
        this.m_pathThetaController = new PIDController(pathThetaController.getP(), pathThetaController.getI(), pathThetaController.getD());
        addRequirements(swerveSubsystem);
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;

        m_pathXController.setSetpoint(targetPose.getX());
        m_pathYController.setSetpoint(targetPose.getY());
        m_pathThetaController.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public void initialize() {
        m_pathXController.reset();
        m_pathYController.reset();
        m_pathThetaController.reset();

        m_pathXController.setSetpoint(targetPose.getX());
        m_pathYController.setSetpoint(targetPose.getY());
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_pathThetaController.setSetpoint(targetPose.getRotation().getRadians());
        System.out.println("I kove people");
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerveSubsystem.getPose();
        System.out.println("I egio people");

        // Feedforward and Feedback
        //TODO: get velocity
        double vx = 0
                + m_pathXController.calculate(currentPose.getX(), targetPose.getX());
        double vy = 0
                + m_pathYController.calculate(currentPose.getY(), targetPose.getY());
        double omega = 0
                + m_pathThetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // Constrain velocities
        double maxVelocity = SwerveConstants.AutonConstants.maxSpeed;
        double maxAngularVelocity = SwerveConstants.AutonConstants.maxAngularSpeed;
        vx = Math.max(-maxVelocity, Math.min(maxVelocity, vx));
        vy = Math.max(-maxVelocity, Math.min(maxVelocity, vy));
        omega = Math.max(-maxAngularVelocity, Math.min(maxAngularVelocity, omega));

        // Apply speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, currentPose.getRotation());
        swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(speeds));
    }

    @Override
    public boolean isFinished() {
        return false;
        // var pose = swerveSubsystem.getPose();
        // double positionError = Math.hypot(pose.getX() - targetPose.getX(), pose.getY() - targetPose.getY());
        // double rotationError = Math.abs(pose.getRotation().getRadians() - targetPose.getRotation().getRadians());
        // return (positionError < 0.05 && rotationError < Math.toRadians(10)); // 5 cm and 10 degrees tolerance
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ending mpve");

        swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }
}
