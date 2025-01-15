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

public class Robot extends TimedRobot {
    public static final CANBus riobus = new CANBus("rio");
    @SuppressWarnings("deprecation")
    public static final CANBus canivorebus = new CANBus(Constants.canbus);

    private Command autonomousCommand;

    private final RobotContainer robotContainer;
    private final ControlBoard controlBoard;

    public Robot() {
        if (Constants.canbus.equals("")) throw new IllegalStateException("CAN Bus ID not set in Constants.java");
        if (!canivorebus.isNetworkFD()) throw new IllegalStateException("CANivore bus is not in FD mode");

        robotContainer = new RobotContainer();
        controlBoard = ControlBoard.getInstance();
    }

    @Override
    public void robotPeriodic() {
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
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
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
