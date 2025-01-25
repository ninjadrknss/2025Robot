// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.robot.subsystems.swerve.Odometry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.auton.AutonSubsystem;
import frc.robot.subsystems.elevatorarm.ElevatorArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.ControlBoard;
import frc.robot.util.TunableParameter;

public class Robot extends TimedRobot {
    public static final CANBus riobus = new CANBus("rio");
    @SuppressWarnings("deprecation")
    public static final CANBus drivebus = new CANBus(Constants.drivebus);

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
    public void robotPeriodic() {
        TunableParameter.updateAll();
        scheduler.run();
    }

    @Override
    public void disabledInit() {
        // ElevatorArmSubsystem.getInstance().setCoastMode();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        // ElevatorArmSubsystem.getInstance().setBrakeMode();
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
        odometry.testResetOdo();

    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
