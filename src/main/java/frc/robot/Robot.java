// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.elevatorwrist.ElevatorWristSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;
import frc.robot.subsystems.simulation.PhotonvisionSim;
import frc.robot.subsystems.vision.LimelightSubsystem;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.PathfindingCommand;

import frc.robot.subsystems.drive.Odometry;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.auton.AutonSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.ControlBoard;
import frc.lib.TunableParameter;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.pathfinding.LocalADStar;
import frc.robot.commands.AssistCommand;


public class Robot extends TimedRobot {
    public static final CANBus riobus = new CANBus("rio");
    @SuppressWarnings("deprecation")
    public static final CANBus drivebus = new CANBus(Constants.drivebus);
    @SuppressWarnings("deprecation")
    public static final CANBus elevatorbus = new CANBus(Constants.elevatorbus);

    private Command autonomousCommand;

    private final ControlBoard controlBoard;
    private final CommandScheduler scheduler;
    private final AutonSubsystem autonSubsystem;
    //private final Field2d m_field = new Field2d();

    //temp here bc i dont think i can put it into swervesubsystems because of the way odometry is set up with swerve as a subsystem
    private final Odometry odometry;

    public Robot() {
        scheduler = CommandScheduler.getInstance();
        controlBoard = ControlBoard.getInstance();
        autonSubsystem = AutonSubsystem.getInstance();
        odometry = Odometry.getInstance();

        // SmartDashboard.putData("Field", m_field);
        // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        //     // Do whatever you want with the pose here
        //     m_field.setRobotPose(pose);
        // });



        // // Logging callback for target robot pose
        // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        //     // Do whatever you want with the pose here
        //     m_field.getObject("target pose").setPose(pose);
        // });

        // // Logging callback for the active path, this is sent as a list of poses
        // PathPlannerLogging.setLogActivePathCallback((poses) -> {
        //     // Do whatever you want with the poses here
        //     m_field.getObject("path").setPoses(poses);
        // });

    }

    @Override
    public void robotInit() {
        odometry.updateAllienceColor((DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue));
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

        //AutoBuilder.configure(swerveSubsystem::getPose, null, swerveSubsystem::getChassisSpeeds, this::drive, controller, SwerveConstants.robotConfig, () -> false, swerveSubsystem);

//        SignalLogger.start(); // TODO: enable this for competition
        Pathfinding.setPathfinder(new LocalADStar()); // can the LocalADStar be static? It likely takes a lot of resources even if its spun off into its own thread
        PathfindingCommand.warmupCommand().schedule();

        // TODO: disable this for competitions
        scheduler.onCommandInitialize(command -> System.out.println("Initializing command: " + command.getName() + "@" + command.getSubsystem() + " w/" + command.getRequirements()));
        scheduler.onCommandFinish(command -> System.out.println("Finishing command: " + command.getName() + "@" + command.getSubsystem() + " w/" + command.getRequirements()));

        SignalLogger.setPath("/media/sda1/");
        // SignalLogger.start();

        // get alliance color from FMS (defaults to Blue if unavailable)
    }

    @Override
    public void robotPeriodic() {
        TunableParameter.updateAll();
        scheduler.run();
        SmartDashboard.putBoolean("Limelight Has Target", LimelightSubsystem.getInstance().hasTarget());
//        printWatchdogEpochs(); // TODO: PRINT ALL THE EPOCHS ON EVERY LOOP
        // ControlBoard.getInstance().tryInit();
        controlBoard.displayUI();
    }

    @Override
    public void driverStationConnected() {
        LimelightSubsystem.getInstance().setAprilTagFilters(); // set the tag filters to the alliance color
        LightsSubsystem.getInstance().requestAllianceColors();
        ControlBoard.getInstance().tryInit();
    }

    @Override
    public void disabledInit() {
//        ElevatorWristSubsystem.getInstance().setCoastMode();
        SignalLogger.stop();
        LightsSubsystem.getInstance().requestAllianceColors();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
//         ElevatorWristSubsystem.getInstance().setBrakeMode();
    }

    @Override
    public void autonomousInit() {
        LimelightSubsystem.getInstance().setAprilTagFilters(); // set the tag filters to the alliance color
       // odometry.resetOdometryCommand().schedule();
        System.out.println("Auton Init");
        autonomousCommand = autonSubsystem.getSelectedAuton();
        if (autonomousCommand != null) autonomousCommand.schedule();
        LightsSubsystem.getInstance().requestRainbow();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {

        //schedule and quickly cancel new AssistCommand() to stop auton
        new AssistCommand().schedule(); 
        // cancel
        

    }

    @Override
    public void teleopInit() {
        odometry.updateAllienceColor((DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue));
        LimelightSubsystem.getInstance().setAprilTagFilters(); // set the tag filters to the alliance color
        if (autonomousCommand != null) autonomousCommand.cancel();

        //TODO: please dont forget about this: 
        //new AssistCommand(FieldConstants.GameElement.REEF_BLUE_1, FieldConstants.GameElement.Branch.LEFT).schedule();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        //new AssistCommand().schedule();
        // CommandScheduler.getInstance().cancelAll();
        // odometry.testResetOdo();
        // SwerveSubsystem.getInstance().resetPose(new Pose2d(2, 4, new Rotation2d()));
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
       PhotonvisionSim.getInstance().update();
       SimulatedArena.getInstance().simulationPeriodic();
    }
}
