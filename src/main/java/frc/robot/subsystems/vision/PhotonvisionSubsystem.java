package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants.PhotonVision;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonvisionSubsystem extends SubsystemBase {
  private static PhotonvisionSubsystem instance;

  private final PhotonCamera leftCamera;
  private final PhotonPoseEstimator leftPhotonPoseEstimator;

  private final PhotonCamera rightCamera;

  private final PhotonPoseEstimator rightPhotonPoseEstimator;

  public static PhotonvisionSubsystem getInstance() {
    if (instance == null) instance = new PhotonvisionSubsystem();
    return instance;
  }

  private PhotonvisionSubsystem() {
    leftCamera = new PhotonCamera(PhotonVision.leftCameraName);
    leftPhotonPoseEstimator =
        new PhotonPoseEstimator(
            PhotonVision.field, PhotonVision.poseStrategy, PhotonVision.leftCameraToRobot);

    rightCamera = new PhotonCamera(PhotonVision.rightCameraName);
    rightPhotonPoseEstimator =
        new PhotonPoseEstimator(
            PhotonVision.field, PhotonVision.poseStrategy, PhotonVision.rightCameraToRobot);
  }

  private Optional<EstimatedRobotPose> getLeftRobotPose(Pose2d prevPose) {
    leftPhotonPoseEstimator.setReferencePose(prevPose);
    List<PhotonPipelineResult> unreadResults = leftCamera.getAllUnreadResults();
    if (unreadResults.isEmpty()) return Optional.empty();
    return leftPhotonPoseEstimator.update(unreadResults.get(0));
  }

  private Optional<EstimatedRobotPose> getRightRobotPose(Pose2d prevPose) {
    rightPhotonPoseEstimator.setReferencePose(prevPose);
    List<PhotonPipelineResult> unreadResults = rightCamera.getAllUnreadResults();
    if (unreadResults.isEmpty()) return Optional.empty();
    return rightPhotonPoseEstimator.update(unreadResults.get(0));
  }

  public EstimatedRobotPose update(Pose2d prevPose) {
    EstimatedRobotPose leftPose = getLeftRobotPose(prevPose).orElse(null);
    EstimatedRobotPose rightPose = getRightRobotPose(prevPose).orElse(null);

    if (leftPose == null) return rightPose;
    if (rightPose == null) return leftPose;

    double leftDist =
        prevPose
            .getTranslation()
            .getDistance(leftPose.estimatedPose.getTranslation().toTranslation2d());
    double rightDist =
        prevPose
            .getTranslation()
            .getDistance(rightPose.estimatedPose.getTranslation().toTranslation2d());

    // Spaghetti merging of the two pose estimates
    Pose3d newPose =
        leftPose.estimatedPose.interpolate(
            rightPose.estimatedPose, leftDist / (leftDist + rightDist + 1e-6));
    List<PhotonTrackedTarget> targetsUsed = leftPose.targetsUsed;
    targetsUsed.addAll(rightPose.targetsUsed);

    return new EstimatedRobotPose(
        newPose,
        Math.max(leftPose.timestampSeconds, rightPose.timestampSeconds),
        targetsUsed,
        leftPose.strategy);
  }
}
