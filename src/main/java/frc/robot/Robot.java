// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.simulation.PhotonvisionSim;
import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import frc.robot.subsystems.drive.Odometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AssistCommand;
import frc.robot.subsystems.auton.AutonSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.ControlBoard;
import frc.lib.TunableParameter;

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

    //temp here bc i dont think i can put it into swervesubsystems because of the way odometry is set up with swerve as a subsystem
    private final Odometry odometry;

    public Robot() {
        scheduler = CommandScheduler.getInstance();
        controlBoard = ControlBoard.getInstance();
        autonSubsystem = AutonSubsystem.getInstance();
        SwerveSubsystem.getInstance();
        odometry = Odometry.getInstance();
    }

    @Override
    public void robotInit() {
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
        }


        PathfindingCommand.warmupCommand().schedule();
        FollowPathCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {
        TunableParameter.updateAll();
        scheduler.run();
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

    @Override
    public void autonomousInit() {
        autonomousCommand = autonSubsystem.getSelectedAuton(); // TODO: fetch from auton command chooser

        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) autonomousCommand.cancel();

        //TODO: please dont forget about this: 
        new AssistCommand(null, Constants.GameElement.Branch.LEFT).schedule();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        //odometry.testResetOdo();
        SwerveSubsystem.getInstance().resetPose(new Pose2d(2, 3, new Rotation2d()));
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
