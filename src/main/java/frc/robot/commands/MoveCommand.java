package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.List;
import java.util.ArrayList;

public class MoveCommand extends Command {
    //TODO: obstacle avoidance
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final Odometry odometry = Odometry.getInstance();

    private final Pose2d targetPose;
    private final List<Pose2d> intermediatePoints;
    private int currentIntermediateIndex;
    private boolean movingToIntermediate;

    private final Timer timer = new Timer();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyFieldSpeeds;

    private final PIDController xController = new PIDController(
        1.4, 0.0, 0.5
    );
    private final PIDController yController = new PIDController(
        1.4, 0.0, 0.5
    );
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        2.5, 0.01, 0, SwerveConstants.AutoConstants.kThetaControllerConstraints
    );

    private final boolean smartMode;
    private final boolean skipPenultimateNode;

    // Max velocity and acceleration
	private static final double MAX_VELOCITY = SwerveConstants.maxSpeed;//SwerveConstants.AutoConstants.kMaxSpeedMetersPerSecond;
    private static final double MAX_ACCELERATION = 5; // meters per second^2

    private static final double CLOSE_THRESHOLD = 0.4;
    private static final double SMOOTHING_FACTOR = 3.0;

    // These track the commanded velocity from the previous loop, so we can ramp up/down.
    private double prevVx = 0;
    private double prevVy = 0;
    private double prevOmega = 0;

    public MoveCommand(Pose2d targetPose, List<Pose2d> intermediatePoints, boolean smartMode, boolean skipPenultimateNode) {
        this.targetPose = targetPose;
        this.intermediatePoints = new ArrayList<>(intermediatePoints);
        this.currentIntermediateIndex = 0;
        this.movingToIntermediate = !intermediatePoints.isEmpty();
        this.smartMode = smartMode;
        this.skipPenultimateNode = skipPenultimateNode;
        this.m_pathApplyFieldSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(2), Math.toRadians(1));

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = SwerveSubsystem.simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();

        // If we're effectively at the target, don't bother moving.
        if (isClose(currentPose, targetPose, 0.02)
            && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 3) {
            System.out.println("Already at target position.");
            end(true);
            return;
        }

        System.out.println("Starting MoveCommand to: " + targetPose);
        // xController.reset(currentPose.getX());
        // yController.reset(currentPose.getY());
        thetaController.reset(currentPose.getRotation().getRadians());

        // Initialize previous velocities to the robot's current chassis speeds
//        prevVx = odometry.getVelocityX();
//        prevVy = odometry.getVelocityY();
//        prevOmega = odometry.getFieldYawRate();

        ChassisSpeeds speeds = SwerveSubsystem.simDrivetrain.mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        prevVx = speeds.vxMetersPerSecond;
        prevVy = speeds.vyMetersPerSecond;
        prevOmega = speeds.omegaRadiansPerSecond;

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Pose2d currentPose = SwerveSubsystem.simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        Pose2d goal = getSmoothedGoal(currentPose);

        // Feedforward attempts to drive towards the goal
        double feedforwardVx = (goal.getX() - currentPose.getX() ) * 1.1;
        double feedforwardVy = (goal.getY() - currentPose.getY() ) * 1.1;

        // PID feedback for position + heading
        double feedbackVx = xController.calculate(currentPose.getX(), goal.getX());
        double feedbackVy = yController.calculate(currentPose.getY(), goal.getY());
        double feedbackOmega = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            goal.getRotation().getRadians()
        );

        // ------------------------ ADDED ROTATIONAL FEEDFORWARD ------------------------
        // This small feedforward ensures we start turning in the "intended" direction
        // rather than reversing if the PID decides the other way is shorter.
        double headingDiff = goal.getRotation().minus(currentPose.getRotation()).getRadians();
        // Normalize headingDiff to [-pi, pi]:
        headingDiff = Math.atan2(Math.sin(headingDiff), Math.cos(headingDiff));

        // Only apply feedforward if the difference in angle is more than a small threshold.
        double feedforwardOmega = 0.0;
        if (Math.abs(headingDiff) > Math.toRadians(5)) { // example threshold ~5 degrees
            // Scale this constant as needed for your robot (0.2 is just an example).
            feedforwardOmega = 0.1 * Math.signum(headingDiff);
        }

        // Combine feedforward and feedback
        double vx = feedforwardVx + feedbackVx;
        double vy = feedforwardVy + feedbackVy;
        double rawOmega = feedbackOmega + feedforwardOmega;
        // ------------------------------------------------------------------------------

        // Apply acceleration limit
        double dt = timer.get(); // Time since last update
        double vxLimited = applyAccelerationLimit(prevVx, vx, dt);
        double vyLimited = applyAccelerationLimit(prevVy, vy, dt);
        double omegaLimited = applyAccelerationLimit(prevOmega, rawOmega, dt);

        // Apply velocity limit
        vxLimited = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, vxLimited));
        vyLimited = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, vyLimited));
        omegaLimited = Math.max(-SwerveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                Math.min(SwerveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, omegaLimited));

        // Store for next iteration
        prevVx = vxLimited;
        prevVy = vyLimited;
        prevOmega = omegaLimited;
        timer.reset(); // Reset for next cycle measurement

        // Convert to field-relative speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vxLimited, 
            vyLimited, 
            omegaLimited, 
            currentPose.getRotation()
        );

        swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(speeds));
    }

    /**
     * Picks either the current intermediate point or the final target.
     * Then performs a simple smoothing so we don't lurch.
     */
    private Pose2d getSmoothedGoal(Pose2d currentPose) {
        Pose2d goal;
        if (movingToIntermediate) {
            goal = intermediatePoints.get(currentIntermediateIndex);

            boolean penultimateCondition = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 40 
                && (skipPenultimateNode ? intermediatePoints.size() - 2 : intermediatePoints.size() - 1) == currentIntermediateIndex; // Second to last intermediate point

            boolean ultimateCondition = skipPenultimateNode
                && (intermediatePoints.size() - 1 == currentIntermediateIndex)
                && isInLine(currentPose, intermediatePoints.get(currentIntermediateIndex-1), targetPose); // Last intermediate point

            boolean shouldSkipNow = smartMode && (penultimateCondition || ultimateCondition);

            if (isClose(currentPose, goal, CLOSE_THRESHOLD) || shouldSkipNow) {
                if (currentIntermediateIndex < intermediatePoints.size() - 1) {
                    currentIntermediateIndex++;
                    System.out.println("Moving to next intermediate point: " + intermediatePoints.get(currentIntermediateIndex));
                } else {
                    movingToIntermediate = false;
                    System.out.println("Switching to final targetPose: " + targetPose);
                }
            }
        } else {
            goal = targetPose;
        }

        return smoothTransition(currentPose, goal);
    }

    private boolean isInLine(Pose2d current, Pose2d initialPose, Pose2d finalPose) {
        // Vector from penultimate to final
        double dxGoal = finalPose.getX() - initialPose.getX();
        double dyGoal = finalPose.getY() - initialPose.getY();
        double angleGoal = Math.atan2(dyGoal, dxGoal);

        // Vector from final to current
        double dxCurr = finalPose.getX() - current.getX();
        double dyCurr = finalPose.getY() - current.getY();
        double angleCurr = Math.atan2(dyCurr, dxCurr);

        // If the difference in angles is small, we consider it "inline."
        double angleDiff = Math.toDegrees(Math.min(Math.abs(angleGoal - angleCurr), Math.abs(angleGoal - angleCurr - Math.PI)));
        return angleDiff < 10.0;  // example tolerance in degrees
    }

    /**
     * Gently pull our "goal" a little closer to the current position.
     * This can help reduce large jumps if a waypoint is far away.
     */
    private Pose2d smoothTransition(Pose2d current, Pose2d goal) {
        double smoothedX = current.getX() * (1 - SMOOTHING_FACTOR) + goal.getX() * SMOOTHING_FACTOR;
        double smoothedY = current.getY() * (1 - SMOOTHING_FACTOR) + goal.getY() * SMOOTHING_FACTOR;
        // Keep the goal rotation.
        return new Pose2d(smoothedX, smoothedY, goal.getRotation());
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
        // Finish if near target position + orientation
        return isClose(targetPose, currentPose, 0.02)
            && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 2;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerveSubsystem.setControl(m_pathApplyFieldSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }
}
