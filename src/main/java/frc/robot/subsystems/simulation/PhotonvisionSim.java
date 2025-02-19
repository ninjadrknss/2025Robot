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

    private final VisionSystemSim visionSim;
    private final PhotonCameraSim camera1Sim;
    private final PhotonCameraSim camera2Sim;
    private final PhotonCameraSim camera3Sim;

    private final NetworkTable visionTable;
    private final StructPublisher<Pose2d> estimatedPose;
    private final StructPublisher<Pose3d> camera1Pose;
    private final StructPublisher<Pose3d> camera2Pose;
    private final StructPublisher<Pose3d> camera3Pose;
    private final StructArrayPublisher<Pose3d> visionTargets1;
    private final StructArrayPublisher<Pose3d> visionTargets2;
    private final StructArrayPublisher<Pose3d> visionTargets3;

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

        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        estimatedPose = visionTable.getStructTopic("Pose", Pose2d.struct).publish();
        camera1Pose = visionTable.getStructTopic("Camera1Pose", Pose3d.struct).publish();
        camera2Pose = visionTable.getStructTopic("Camera2Pose", Pose3d.struct).publish();
        camera3Pose = visionTable.getStructTopic("Camera3Pose", Pose3d.struct).publish();
        visionTargets1 = visionTable.getStructArrayTopic("Targets1", Pose3d.struct).publish();
        visionTargets2 = visionTable.getStructArrayTopic("Targets2", Pose3d.struct).publish();
        visionTargets3 = visionTable.getStructArrayTopic("Targets3", Pose3d.struct).publish();

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(800, 600, Rotation2d.fromDegrees(75));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(tagLayout);

        PhotonCamera camera1 = new PhotonCamera("leftCamera");
        camera1Sim = new PhotonCameraSim(camera1, cameraProp);
        PhotonCamera camera2 = new PhotonCamera("rightCamera");
        camera2Sim = new PhotonCameraSim(camera2, cameraProp);
        PhotonCamera camera3 = new PhotonCamera("centerLimelight");
        camera3Sim = new PhotonCameraSim(camera3, cameraProp);

        visionSim.addCamera(camera1Sim, new Transform3d(
                new Translation3d(
                    Meter.convertFrom(2.979, Inch),
                    Meter.convertFrom(9.41, Inch),
                    Meter.convertFrom(37.418, Inch)
                ),
                new Rotation3d(0, -Radians.convertFrom(-15, Degrees), Radians.convertFrom(155, Degrees))));
        visionSim.addCamera(camera2Sim, new Transform3d(
                new Translation3d(
                        Meter.convertFrom(2.979, Inch),
                        -Meter.convertFrom(9.41, Inch),
                        Meter.convertFrom(37.418, Inch)

                        ), /*Y, X, Z*/
                new Rotation3d(0, -Radians.convertFrom(-15, Degrees), Radians.convertFrom(-155, Degrees))));
        visionSim.addCamera(camera3Sim, new Transform3d(
                new Translation3d(
                        Meter.convertFrom(10, Inch),
                        Meter.convertFrom(0, Inch),
                        Meter.convertFrom(10, Inch)
                ),
                new Rotation3d()
        ));
    }

    private static final int lifetime = 5; // Number of frames to keep a target in the map to prevent weird flickering

    private final HashMap<Integer, Integer> targetMap1 = new HashMap<>();
    private final HashMap<Integer, Integer> targetMap2 = new HashMap<>();
    private final HashMap<Integer, Integer> targetMap3 = new HashMap<>();

    public void update() {
        // Publish the estimated robot pose to the network table
        estimatedPose.set(visionSim.getRobotPose().toPose2d());
        // Publish the camera poses to the network table
        if (visionSim.getCameraPose(camera1Sim).isPresent()) {
            Pose3d pose = visionSim.getCameraPose(camera1Sim).get();
            camera1Pose.set(pose);
        }
        if (visionSim.getCameraPose(camera2Sim).isPresent()) {
            Pose3d pose = visionSim.getCameraPose(camera2Sim).get();
            camera2Pose.set(pose);
        }
        if (visionSim.getCameraPose(camera3Sim).isPresent()) {
            Pose3d pose = visionSim.getCameraPose(camera3Sim).get();
            camera3Pose.set(pose);
        }
        // Update the vision system simulation to use the current robot pose
        visionSim.update(SwerveSubsystem.simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());

        // Decrement the lifetime of all targets
        targetMap1.replaceAll((k, v) -> v - 1);
        targetMap2.replaceAll((k, v) -> v - 1);
        targetMap3.replaceAll((k, v) -> v - 1);

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
        camera3Sim.getCamera().getAllUnreadResults().forEach(result -> {
            result.targets.forEach(target -> {
                targetMap3.put(target.fiducialId, lifetime);
            });
        });

        // Create lists of the targets to publish
        ArrayList<Pose3d> targets1 = new ArrayList<>();
        ArrayList<Pose3d> targets2 = new ArrayList<>();
        ArrayList<Pose3d> targets3 = new ArrayList<>();
        for (VisionTargetSim tar : visionSim.getVisionTargets()) {
            if (targetMap1.containsKey(tar.fiducialID) && targetMap1.get(tar.fiducialID) > 0) {
                targets1.add(tar.getPose());
            }
            if (targetMap2.containsKey(tar.fiducialID) && targetMap2.get(tar.fiducialID) > 0) {
                targets2.add(tar.getPose());
            }
            if (targetMap3.containsKey(tar.fiducialID) && targetMap3.get(tar.fiducialID) > 0) {
                targets3.add(tar.getPose());
            }
        }
        // Publish the vision targets to the network table
        visionTargets1.set(targets1.toArray(new Pose3d[0]));
        visionTargets2.set(targets2.toArray(new Pose3d[0]));
        visionTargets3.set(targets3.toArray(new Pose3d[0]));
    }
}
