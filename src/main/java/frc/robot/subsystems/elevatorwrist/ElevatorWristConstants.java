package frc.robot.subsystems.elevatorwrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import frc.lib.CTREConfig;
import frc.robot.Robot;

public class ElevatorWristConstants {
    public static final double revolutionsPerInch = 1;

    public static final CTREConfig<CANcoder, CANcoderConfiguration> homeHallEffect = new CTREConfig<>();
    static {
        homeHallEffect.withName("Home Hall Effect CANcoder")
                .withCanID(33)
                .withBus(Robot.elevatorbus)
                .withConfig(new CANcoderConfiguration());
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> rightElevatorMotorConfig = new CTREConfig<>();
    static {
        rightElevatorMotorConfig.withName("Right Elevator Motor")
                .withCanID(31)
                .withBus(Robot.elevatorbus)
                .withConfig(new TalonFXConfiguration());

        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.Slot0.kP = 0; // Increase until elevator oscillates
        leaderConfig.Slot0.kI = 0; // Don't touch
        leaderConfig.Slot0.kD = 0; // Increase until jitter
        leaderConfig.Slot0.kS = 0; // Increase until just before motor starts moving
        leaderConfig.Slot0.kG = 0; // Increase until elevator holds steady
        leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        leaderConfig.Feedback.RotorToSensorRatio = 1; // TODO: CHANGE
        leaderConfig.Feedback.SensorToMechanismRatio = 1; // TODO: CHANGE

        leaderConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
        leaderConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = ElevatorWristConstants.homeHallEffect.canID;

        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9999; // TODO: Change

        leaderConfig.CurrentLimits.StatorCurrentLimit = 80; // TODO: Change
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leaderConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        leaderConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;

        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: CHECK
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> leftElevatorMotorConfig = new CTREConfig<>();
    static {
        leftElevatorMotorConfig.withName("Left Elevator Motor")
                .withCanID(32)
                .withBus(Robot.elevatorbus)
                .withConfig(new TalonFXConfiguration());
        TalonFXConfiguration followerConfig = leftElevatorMotorConfig.config;
    }

    public static final CTREConfig<CANcoder, CANcoderConfiguration> wristEncoderConfig = new CTREConfig<>();
    static {
        wristEncoderConfig.withName("Wrist Encoder")
                .withCanID(42)
                .withBus(Robot.elevatorbus)
                .withConfig(new CANcoderConfiguration());
        CANcoderConfiguration wristConfig = wristEncoderConfig.config;
        wristConfig.MagnetSensor.MagnetOffset = 0; // TODO: CHANGE, in revolutions
        wristConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // TODO: CHANGE
        wristConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // TODO: CHANGE
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> wristMotorConfig = new CTREConfig<>();
    static {
        wristMotorConfig.withName("Wrist Motor")
                .withCanID(41)
                .withBus(Robot.elevatorbus)
                .withConfig(new TalonFXConfiguration());
        TalonFXConfiguration wristConfig = wristMotorConfig.config;
        wristConfig.Slot0.kP = 0;
        wristConfig.Slot0.kI = 0;
        wristConfig.Slot0.kD = 0;
        wristConfig.Slot0.kS = 0;
        wristConfig.Slot0.kG = 0;
        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        wristConfig.Feedback.RotorToSensorRatio = 1; // TODO: CHANGE
        wristConfig.Feedback.SensorToMechanismRatio = 1; // TODO: CHANGE
        wristConfig.Feedback.FeedbackRemoteSensorID = ElevatorWristConstants.wristEncoderConfig.canID;
        wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; // TODO: Change
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; // TODO: Change

        wristConfig.CurrentLimits.StatorCurrentLimit = 40; // TODO: Change
        wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        wristConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        wristConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;

        wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: CHECK
    }
}
