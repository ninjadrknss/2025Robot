package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.Odometry;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class PreciseMoveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final Odometry odometry = Odometry.getInstance();

    private final Pose2d targetPose;

    private final Timer timer = new Timer();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyFieldSpeeds;

    private final PIDController xController = new PIDController(3, 0.0, 0);
    private final PIDController yController = new PIDController(3, 0.0, 0);
    private final ProfiledPIDController thetaController;

    private static final double MAX_VELOCITY = 2;
    private static final double MAX_ACCELERATION = 3;

    private double prevVx = 0;
    private double prevVy = 0;
    private double prevOmega = 0;

    private final Timer stuckTimer = new Timer();
    private Pose2d lastPose = new Pose2d();
    private static final double STUCK_TIMEOUT_SECONDS = 0.5;
    private static final double STUCK_POSITION_THRESHOLD = 0.03; // meters

    public PreciseMoveCommand(Pose2d targetPose) {
        this.targetPose = targetPose;
        this.m_pathApplyFieldSpeeds = new SwerveRequest.ApplyRobotSpeeds();
        thetaController = new ProfiledPIDController(
            2, 0.05, 0, SwerveConstants.AutoConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(2), Math.toRadians(1));

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = odometry.getPose();

        // if (isClose(currentPose, targetPose, 0.05)
        //     && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 3) {
        //     System.out.println("Already at target position.");
        //     end(true);
        //     return;
        // }

        System.out.println("Starting MoveCommand to: " + targetPose);
        thetaController.reset(currentPose.getRotation().getRadians());

        prevVx = odometry.getVelocityX();
        prevVy = odometry.getVelocityY();
        prevOmega = odometry.getFieldYawRate();

        timer.reset();
        timer.start();

        lastPose = currentPose;
        stuckTimer.reset();
        stuckTimer.start();
    }

    @Override
    public void execute() {
        //System.out.println("tangius");
        Pose2d currentPose = swerveSubsystem.getPose();
        Pose2d goal = targetPose;

        double feedforwardVx = (goal.getX() - currentPose.getX()) * 0.5;
        double feedforwardVy = (goal.getY() - currentPose.getY()) * 0.5;

        double feedbackVx = xController.calculate(currentPose.getX(), goal.getX());
        double feedbackVy = yController.calculate(currentPose.getY(), goal.getY());
        double feedbackOmega = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            goal.getRotation().getRadians()
        );

        double headingDiff = goal.getRotation().minus(currentPose.getRotation()).getRadians();
        headingDiff = Math.atan2(Math.sin(headingDiff), Math.cos(headingDiff));

        double feedforwardOmega = 0.0;
        if (Math.abs(headingDiff) > Math.toRadians(5)) {
            feedforwardOmega = 0.1 * Math.signum(headingDiff);
        }

        double vx = feedforwardVx + feedbackVx;
        double vy = feedforwardVy + feedbackVy;
        double rawOmega = feedbackOmega + feedforwardOmega;

        double dt = timer.get();
        double vxLimited = applyAccelerationLimit(prevVx, vx, dt);
        double vyLimited = applyAccelerationLimit(prevVy, vy, dt);
        double omegaLimited = applyAccelerationLimit(prevOmega, rawOmega, dt);

        vxLimited = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, vxLimited));
        vyLimited = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, vyLimited));
        omegaLimited = Math.max(
            -SwerveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            Math.min(SwerveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, omegaLimited)
        );

        prevVx = vxLimited;
        prevVy = vyLimited;
        prevOmega = omegaLimited;
        timer.reset();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vxLimited,
            vyLimited,
            omegaLimited,
            currentPose.getRotation()
        );

        swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(speeds));

    }

    private boolean isClose(Pose2d current, Pose2d target, double tolerance) {
        double positionError = Math.hypot(current.getX() - target.getX(), current.getY() - target.getY());
        return positionError < tolerance;
    }

    private double applyAccelerationLimit(double previous, double desired, double dt) {
        double maxChange = MAX_ACCELERATION * dt;
        double lowerLimit = previous - maxChange;
        double upperLimit = previous + maxChange;
        return Math.max(lowerLimit, Math.min(upperLimit, desired));
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = odometry.getPose();

        double positionDelta = Math.hypot(
            currentPose.getX() - lastPose.getX(),
            currentPose.getY() - lastPose.getY()
        );

        if (positionDelta > STUCK_POSITION_THRESHOLD) {
            stuckTimer.reset();
            stuckTimer.start();
            lastPose = currentPose;
            System.out.println("Robot moved. Resetting stuck timer.");
        } else if (stuckTimer.get() > STUCK_TIMEOUT_SECONDS) {
            System.out.println("Robot seems stuck. Ending command.");
            return true;
        }

        return isClose(targetPose, currentPose, 0.02)
            && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 2;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        stuckTimer.stop();
        swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
        System.out.println("Ending tangius");
    }
}
