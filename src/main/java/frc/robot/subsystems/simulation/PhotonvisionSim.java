package frc.robot.subsystems.simulation;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.subsystems.drive.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import static edu.wpi.first.units.Units.*;

public class PhotonvisionSim {
    private static PhotonvisionSim instance;

    private final VisionSystemSim visionSim = new VisionSystemSim("main");
    private final PhotonCameraSim camera1Sim;
    private final PhotonCameraSim camera2Sim;

    private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    private final StructPublisher<Pose2d> estimatedPose = visionTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose3d> camera1Pose = visionTable.getStructTopic("Camera1Pose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> camera2Pose = visionTable.getStructTopic("Camera2Pose", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> visionTargets1 = visionTable.getStructArrayTopic("Targets1", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> visionTargets2 = visionTable.getStructArrayTopic("Targets2", Pose3d.struct).publish();

    public static PhotonvisionSim getInstance() {
        if (instance == null) instance = new PhotonvisionSim();
        return instance;
    }

    private PhotonvisionSim() {
        if (!Utils.isSimulation()) throw new RuntimeException("PhotonvisionSim should only be instantiated in simulation");
        AprilTagFieldLayout tagLayout;
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load tag layout");
            throw new RuntimeException(e);
        }

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(800, 600, Rotation2d.fromDegrees(75));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        visionSim.addAprilTags(tagLayout);

        PhotonCamera camera1 = new PhotonCamera("leftCamera");
        camera1Sim = new PhotonCameraSim(camera1, cameraProp);
        PhotonCamera camera2 = new PhotonCamera("rightCamera");
        camera2Sim = new PhotonCameraSim(camera2, cameraProp);

        visionSim.addCamera(camera1Sim, new Transform3d(
                new Translation3d(
                    Meter.convertFrom(10.659, Inch),
                    Meter.convertFrom(37.418, Inch),
                    Meter.convertFrom(2.979, Inch)
                ),
                new Rotation3d(0, 0, Radians.convertFrom(-45, Degrees))));
        visionSim.addCamera(camera2Sim, new Transform3d(
                new Translation3d(
                        Meter.convertFrom(-10.659, Inch),
                        Meter.convertFrom(37.418, Inch),
                        Meter.convertFrom(2.979, Inch)
                ),
                new Rotation3d(0, 0, Radians.convertFrom(45, Degrees))));
    }

    private static final int lifetime = 5; // Number of frames to keep a target in the map to prevent weird flickering

    private final HashMap<Integer, Integer> targetMap1 = new HashMap<>();
    private final HashMap<Integer, Integer> targetMap2 = new HashMap<>();

    public void update() {
        // Publish the estimated robot pose to the network table
        estimatedPose.set(visionSim.getRobotPose().toPose2d());
        // Publish the camera poses to the network table
        if (visionSim.getCameraPose(camera1Sim).isPresent()) camera1Pose.set(visionSim.getCameraPose(camera1Sim).get());
        if (visionSim.getCameraPose(camera2Sim).isPresent()) camera2Pose.set(visionSim.getCameraPose(camera2Sim).get());
        // Update the vision system simulation to use the current robot pose
        visionSim.update(SwerveSubsystem.simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());

        // Decrement the lifetime of all targets
        targetMap1.replaceAll((k, v) -> v - 1);
        targetMap2.replaceAll((k, v) -> v - 1);

        // Update the target maps with the new targets
        camera1Sim.getCamera().getAllUnreadResults().forEach(result -> {
            result.targets.forEach(target -> {
                targetMap1.put(target.fiducialId, lifetime);
            });
        });
        camera2Sim.getCamera().getAllUnreadResults().forEach(result -> {
            result.targets.forEach(target -> {
                targetMap2.put(target.fiducialId, lifetime);
            });
        });

        // Create lists of the targets to publish
        ArrayList<Pose3d> targets1 = new ArrayList<>();
        ArrayList<Pose3d> targets2 = new ArrayList<>();
        for (VisionTargetSim tar : visionSim.getVisionTargets()) {
            if (targetMap1.containsKey(tar.fiducialID) && targetMap1.get(tar.fiducialID) > 0) {
                targets1.add(tar.getPose());
            }
            if (targetMap2.containsKey(tar.fiducialID) && targetMap2.get(tar.fiducialID) > 0) {
                targets2.add(tar.getPose());
            }
        }
        // Publish the vision targets to the network table
        visionTargets1.set(targets1.toArray(new Pose3d[0]));
        visionTargets2.set(targets2.toArray(new Pose3d[0]));
    }
}
