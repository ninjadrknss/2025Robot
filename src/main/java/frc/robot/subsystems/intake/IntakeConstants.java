package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.lib.MotorConfig;
import frc.robot.Robot;

public class IntakeConstants {
    public static final MotorConfig intakeMotorConfig = new MotorConfig().withCanID(0).withBus(Robot.elevatorbus);

    static {
        TalonFXConfiguration intakeConfig = intakeMotorConfig.config;
        intakeConfig.Slot0.kP = 0; // Increase until speed oscillates
        intakeConfig.Slot0.kI = 0; // Don't touch
        intakeConfig.Slot0.kD = 0; // Increase until jitter
        intakeConfig.Slot0.kS = 0; // Increase until just before motor starts moving
        intakeConfig.Slot0.kA = 0; //
        intakeConfig.Slot0.kV = 0; //
        intakeConfig.Slot0.kG = 0; // Don't touch

        intakeConfig.Feedback.RotorToSensorRatio = 1; // TODO: CHANGE
        intakeConfig.Feedback.SensorToMechanismRatio = 1; // TODO: CHANGE
    }

    public static final int beamBreakPort = 0;
    public static final int distanceSensorID = 0;

    public static final double intakeSpeed = 0.5;
    public static final double spitSpeed = 0.5; // Negated in request

    public static final int algaeDistanceThreshold = 0;
}
