// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.elevatorarm.ElevatorArmSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.ControlBoard;
import frc.robot.util.TunableParameter;

public class Robot extends TimedRobot {
    public static final CANBus riobus = new CANBus("rio");
    @SuppressWarnings("deprecation")
    public static final CANBus drivebus = new CANBus(Constants.drivebus);

    private Command autonomousCommand;

    private final ControlBoard controlBoard;

    public Robot() {
        controlBoard = ControlBoard.getInstance();
    }

    @Override
    public void robotPeriodic() {
        TunableParameter.updateAll();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        ElevatorArmSubsystem.getInstance().setCoastMode();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        ElevatorArmSubsystem.getInstance().setBrakeMode();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = null; // TODO: fetch from auton command chooser

        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) autonomousCommand.cancel();
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
