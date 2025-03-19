// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.subsystems.simulation.PhotonvisionSim;

import java.util.List;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.PathPlannerLogging;

import frc.robot.subsystems.drive.Odometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AssistCommand;
import frc.robot.subsystems.auton.AutonSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.FieldConstants;
import frc.robot.util.ControlBoard;
import frc.lib.TunableParameter;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class Robot extends TimedRobot {
    public static final CANBus riobus = new CANBus("rio");
    @SuppressWarnings("deprecation")
    public static final CANBus drivebus = new CANBus(Constants.drivebus);
    @SuppressWarnings("deprecation")
    public static final CANBus elevatorbus = new CANBus(Constants.elevatorbus);

    public static DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

    private Command autonomousCommand;

    private final ControlBoard controlBoard;
    private final CommandScheduler scheduler;
    private final AutonSubsystem autonSubsystem;
    private final Field2d m_field = new Field2d();

    //temp here bc i dont think i can put it into swervesubsystems because of the way odometry is set up with swerve as a subsystem
    private final Odometry odometry;

    public Robot() {
        scheduler = CommandScheduler.getInstance();
        controlBoard = ControlBoard.getInstance();
        autonSubsystem = AutonSubsystem.getInstance();
        SwerveSubsystem.getInstance();
        odometry = Odometry.getInstance();

        SmartDashboard.putData("Field", m_field);
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            m_field.setRobotPose(pose);
        });
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
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }

//        SignalLogger.start(); // TODO: enable this for competition

        PathfindingCommand.warmupCommand().schedule();
        FollowPathCommand.warmupCommand().schedule();

        // TODO: disable this for competitions
        scheduler.onCommandInitialize(command -> System.out.println("Initializing command: " + command.getName() + "@" + command.getRequirements()));
        scheduler.onCommandFinish(command -> System.out.println("Finishing command: " + command.getName() + "@" + command.getRequirements()));

        // get alliance color from FMS (defaults to Blue if unavailable)
    }

    @Override
    public void robotPeriodic() {
        TunableParameter.updateAll();
        scheduler.run();
//        printWatchdogEpochs(); // TODO: PRINT ALL THE EPOCHS ON EVERY LOOP
        // ControlBoard.getInstance().tryInit();
    }

    @Override
    public void disabledInit() {
        // ElevatorWristSubsystem.getInstance().setCoastMode();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        // ElevatorWristSubsystem.getInstance().setBrakeMode();
    }
    SwerveRequest swerveRequest = new SwerveRequest.FieldCentric().withVelocityX(2).withVelocityY(0);

    @Override
    public void autonomousInit() {
        // autonomousCommand = autonSubsystem.getSelectedAuton();
        autonomousCommand = Commands.sequence(
        Commands.runOnce(() -> SwerveSubsystem.getInstance().resetRotation(SwerveSubsystem.getInstance().getOperatorForwardDirection())),
        SwerveSubsystem.getInstance().applyRequest(() -> swerveRequest));

        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) autonomousCommand.cancel();

        //TODO: please dont forget about this: 
        //new AssistCommand(null, FieldConstants.GameElement.Branch.LEFT).schedule();
    }

    @Override
    public void teleopPeriodic() {
        controlBoard.displayUI();

        
        if (controlBoard.operator != null) {
            double thing = controlBoard.operator.rightVerticalJoystick.getAsDouble();
            controlBoard.getRawVoltageCommand(thing);
        }
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        odometry.testResetOdo();
        SwerveSubsystem.getInstance().resetPose(new Pose2d(2, 4, new Rotation2d()));
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
