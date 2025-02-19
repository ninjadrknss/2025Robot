package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class PhotonvisionSubsystem extends SubsystemBase {
    private static PhotonvisionSubsystem instance;

    private final PhotonCamera leftCamera;
    private final Transform3d leftCameraToRobot;
    private final PhotonPoseEstimator leftPhotonPoseEstimator;

    private final PhotonCamera rightCamera;
    private final Transform3d rightCameraToRobot;
    private final PhotonPoseEstimator rightPhotonPoseEstimator;

    public static PhotonvisionSubsystem getInstance() {
        if (instance == null) instance = new PhotonvisionSubsystem();
        return instance;
    }

    private PhotonvisionSubsystem() {
        leftCamera = new PhotonCamera("left");
        leftCameraToRobot = new Transform3d(
                new Translation3d(
                        Meter.convertFrom(2.979, Inch),
                        Meter.convertFrom(9.41, Inch),
                        Meter.convertFrom(37.418, Inch)
                ),
                new Rotation3d(
                        0,
                        -Radians.convertFrom(-15, Degrees),
                        Radians.convertFrom(155, Degrees)
                )
        );
        leftPhotonPoseEstimator = new PhotonPoseEstimator(
                VisionConstants.PhotonVision.field,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                leftCameraToRobot
        );

        rightCamera = new PhotonCamera("right");
        rightCameraToRobot = new Transform3d(
            new Translation3d(
                Meter.convertFrom(2.979, Inch),
                -Meter.convertFrom(9.41, Inch),
                Meter.convertFrom(37.418, Inch)
            ), /*Y, X, Z*/
            new Rotation3d(
                0,
                -Radians.convertFrom(-15, Degrees),
                Radians.convertFrom(-155, Degrees)
            )
        );
        rightPhotonPoseEstimator = new PhotonPoseEstimator(
            VisionConstants.PhotonVision.field,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            rightCameraToRobot
        );
    }

    private Optional<EstimatedRobotPose> getLeftRobotPose(Pose2d prevPose) {
        leftPhotonPoseEstimator.setReferencePose(prevPose);
        return leftPhotonPoseEstimator.update(leftCamera.getAllUnreadResults().get(0));
    }

    private Optional<EstimatedRobotPose> getRightRobotPose(Pose2d prevPose) {
        rightPhotonPoseEstimator.setReferencePose(prevPose);
        return rightPhotonPoseEstimator.update(rightCamera.getAllUnreadResults().get(0));
    }

    public EstimatedRobotPose update(Pose2d prevPose) {
        EstimatedRobotPose leftPose = getLeftRobotPose(prevPose).orElse(null);
        EstimatedRobotPose rightPose = getRightRobotPose(prevPose).orElse(null);

        if (leftPose == null) return rightPose;
        if (rightPose == null) return leftPose;

        double leftDist = prevPose.getTranslation().getDistance(leftPose.estimatedPose.getTranslation().toTranslation2d());
        double rightDist = prevPose.getTranslation().getDistance(rightPose.estimatedPose.getTranslation().toTranslation2d());

        // Spaghetti merging of the two pose estimates
        Pose3d newPose = leftPose.estimatedPose.interpolate(rightPose.estimatedPose, leftDist/(leftDist + rightDist));
        List<PhotonTrackedTarget> targetsUsed = leftPose.targetsUsed;
        targetsUsed.addAll(rightPose.targetsUsed);

        return new EstimatedRobotPose(
                newPose,
                Math.max(leftPose.timestampSeconds, rightPose.timestampSeconds),
                targetsUsed,
                leftPose.strategy
        );
    }
}

