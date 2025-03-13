package frc.robot.subsystems.elevatorwrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.Units;
import frc.lib.CTREConfig;
import frc.robot.Robot;

public class ElevatorWristConstants {
    // 36 teeth of pulley, 5mm spacing, with a 10:58 gear ratio
    public static final double revolutionsPerInch = 36 * Units.Millimeter.of(5).in(Units.Inches) * 10 / 58; // I think

    public static final CTREConfig<CANcoder, CANcoderConfiguration> homeHallEffect = new CTREConfig<>(CANcoderConfiguration::new);
    static {
        homeHallEffect.withName("Home Hall Effect CANcoder")
                .withCanID(33)
                .withBus(Robot.elevatorbus);
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> rightElevatorMotorConfig = new CTREConfig<>(TalonFXConfiguration::new);
    static {
        rightElevatorMotorConfig.withName("Right Elevator Motor")
                .withCanID(31)
                .withBus(Robot.elevatorbus);
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.Slot0.kP = 0; // Increase until elevator oscillates
        leaderConfig.Slot0.kI = 0; // Don't touch
        leaderConfig.Slot0.kD = 0; // Increase until jitter
        leaderConfig.Slot0.kV = 0; // Voltage required to maintain speed
        leaderConfig.Slot0.kS = 0; // Increase until just before motor starts moving
        leaderConfig.Slot0.kG = 16; // Increase until elevator holds steady
        leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        leaderConfig.Feedback.RotorToSensorRatio = 1; // TODO: CHANGE
        leaderConfig.Feedback.SensorToMechanismRatio = 1; // TODO: CHANGE

        // leaderConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
        // leaderConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = ElevatorWristConstants.homeHallEffect.canID;

        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9999; // TODO: Change

        leaderConfig.CurrentLimits.StatorCurrentLimit = 80; // TODO: Tune
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leaderConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        leaderConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;

        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // Unlimited
        leaderConfig.MotionMagic.MotionMagicAcceleration = 400; // TODO: Tune
        leaderConfig.MotionMagic.MotionMagicJerk = 4000; // TODO: Tune

        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> leftElevatorMotorConfig = new CTREConfig<>(TalonFXConfiguration::new);
    static {
        leftElevatorMotorConfig.withName("Left Elevator Motor")
                .withCanID(32)
                .withBus(Robot.elevatorbus);

        TalonFXConfiguration followerConfig = leftElevatorMotorConfig.config;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    public static final CTREConfig<CANcoder, CANcoderConfiguration> wristEncoderConfig = new CTREConfig<>(CANcoderConfiguration::new);
    static {
        wristEncoderConfig.withName("Wrist Encoder")
                .withCanID(42)
                .withBus(Robot.elevatorbus);

        CANcoderConfiguration wristConfig = wristEncoderConfig.config;
        wristConfig.MagnetSensor.MagnetOffset = -0.291748046875 - 0.25; // in revs
        wristConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        wristConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> wristMotorConfig = new CTREConfig<>(TalonFXConfiguration::new);
    static {
        wristMotorConfig.withName("Wrist Motor")
                .withCanID(41)
                .withBus(Robot.elevatorbus);

        TalonFXConfiguration wristConfig = wristMotorConfig.config;
        wristConfig.Slot0.kP = 45;
        wristConfig.Slot0.kI = 0;
        wristConfig.Slot0.kD = 10;
        wristConfig.Slot0.kS = 0;
        wristConfig.Slot0.kG = 36;
        wristConfig.Slot0.kV = 0;
        // wristConfig.Slot0.kV = 0.89489;
        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        wristConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        wristConfig.Feedback.RotorToSensorRatio = 10;
        wristConfig.Feedback.SensorToMechanismRatio = 1;
        wristConfig.Feedback.FeedbackRemoteSensorID = ElevatorWristConstants.wristEncoderConfig.canID;
        wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        wristConfig.Feedback.FeedbackRotorOffset = 0;

        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0; // TODO: Change
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; // TODO: Change

        wristConfig.CurrentLimits.StatorCurrentLimit = 60; // TODO: Change
        wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        wristConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        wristConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;

        wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
}
