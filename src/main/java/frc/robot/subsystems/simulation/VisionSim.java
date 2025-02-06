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
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import static edu.wpi.first.units.Units.*;

public class VisionSim {
    private static VisionSim instance;

    private final VisionSystemSim visionSim = new VisionSystemSim("main");
    private final TargetModel targetModel = TargetModel.kAprilTag36h11;
    private final AprilTagFieldLayout tagLayout;
    private final SimCameraProperties cameraProp = new SimCameraProperties();
    private final PhotonCamera camera1 = new PhotonCamera("leftCamera");
    private final PhotonCamera camera2 = new PhotonCamera("rightCamera");
    private final PhotonCameraSim camera1Sim;
    private final PhotonCameraSim camera2Sim;

    NetworkTable vision = NetworkTableInstance.getDefault().getTable("Vision");
    private final StructPublisher<Pose2d> estimatedPose = vision.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose3d> camera1Pose = vision.getStructTopic("Camera1Pose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> camera2Pose = vision.getStructTopic("Camera2Pose", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> visionTargets1 = vision.getStructArrayTopic("Targets1", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> visionTargets2 = vision.getStructArrayTopic("Targets2", Pose3d.struct).publish();

    public static VisionSim getInstance() {
        if (instance == null) instance = new VisionSim();
        return instance;
    }

    private VisionSim() {
        if (!Utils.isSimulation()) throw new RuntimeException("VisionSim should only be instantiated in simulation");
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load tag layout");
            throw new RuntimeException(e);
        }

        cameraProp.setCalibration(800, 600, Rotation2d.fromDegrees(75));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        visionSim.addAprilTags(tagLayout);

        camera1Sim = new PhotonCameraSim(camera1, cameraProp);
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

    HashMap<Integer, Integer> targetMap1 = new HashMap<>();
    HashMap<Integer, Integer> targetMap2 = new HashMap<>();

    int lifetime = 5;

    public void update() {
        estimatedPose.set(visionSim.getRobotPose().toPose2d());
        visionSim.update(SwerveSubsystem.simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());

        targetMap1.replaceAll((k, v) -> v - 1);
        targetMap2.replaceAll((k, v) -> v - 1);

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
        visionTargets1.accept(targets1.toArray(new Pose3d[0]));
        visionTargets2.accept(targets2.toArray(new Pose3d[0]));

        if (visionSim.getCameraPose(camera1Sim).isPresent()) camera1Pose.set(visionSim.getCameraPose(camera1Sim).get());
        if (visionSim.getCameraPose(camera2Sim).isPresent()) camera2Pose.set(visionSim.getCameraPose(camera2Sim).get());
    }
}
