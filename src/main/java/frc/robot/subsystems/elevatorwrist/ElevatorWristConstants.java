package frc.robot.subsystems.elevatorwrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Per;
import frc.lib.CTREConfig;
import frc.robot.Robot;

public class ElevatorWristConstants {
    // 36 teeth of pulley, 5mm spacing, with a 10:58 gear ratio
    public static final Per<AngleUnit, DistanceUnit> revolutionsPerInch = Units.Revolutions.of(
            36.0 * Units.Millimeter.of(5.0).in(Units.Inches) * 10.0 / 58.0
    ).per(Units.Inch); // I think

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
        TalonFXConfiguration leaderConfig = rightElevatorMotorConfig.config;
        leaderConfig.Slot0.kG = (29.5 + 4.5) / 2; // Increase until elevator holds steady
        leaderConfig.Slot0.kS = (29.5 - 4.5) / 2; // Increase until just before motor starts moving
        leaderConfig.Slot0.kV = 0; // Voltage required to maintain speed
        leaderConfig.Slot0.kP = 0; // Increase until elevator oscillates
        leaderConfig.Slot0.kI = 0; // Don't touch
        leaderConfig.Slot0.kD = 0; // Don't touch
        leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        leaderConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 40; // TODO: Tune
        leaderConfig.MotionMagic.MotionMagicAcceleration = 1; // TODO: Tune

        leaderConfig.Feedback.RotorToSensorRatio = 1;
        leaderConfig.Feedback.SensorToMechanismRatio = 1;

        // leaderConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
        // leaderConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = ElevatorWristConstants.homeHallEffect.canID;

        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 9999; // TODO: Change

        leaderConfig.CurrentLimits.StatorCurrentLimit = 80;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leaderConfig.TorqueCurrent.PeakForwardTorqueCurrent = 30; // TODO; Change
        leaderConfig.TorqueCurrent.PeakReverseTorqueCurrent = -30; // TODO: change

        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> leftElevatorMotorConfig = new CTREConfig<>(TalonFXConfiguration::new);
    static {
        leftElevatorMotorConfig.withName("Left Elevator Motor")
                .withCanID(32)
                .withBus(Robot.elevatorbus);

        TalonFXConfiguration followerConfig = leftElevatorMotorConfig.config;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    public static final CTREConfig<CANcoder, CANcoderConfiguration> wristEncoderConfig = new CTREConfig<>(CANcoderConfiguration::new);
    static {
        wristEncoderConfig.withName("Wrist Encoder")
                .withCanID(42)
                .withBus(Robot.elevatorbus);

        CANcoderConfiguration wristConfig = wristEncoderConfig.config;
        wristConfig.MagnetSensor.MagnetOffset = -0.348876953125 + 0.25; // in revs
        wristConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        wristConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> wristMotorConfig = new CTREConfig<>(TalonFXConfiguration::new);
    static {
        wristMotorConfig.withName("Wrist Motor")
                .withCanID(41)
                .withBus(Robot.elevatorbus);

        TalonFXConfiguration wristConfig = wristMotorConfig.config;
        wristConfig.Slot0.kG = -(21.5 + 13.5) / 2; // Increase until wrist holds steady
        wristConfig.Slot0.kS = (21.5 - 13.5) / 2; // Increase to overcome static friction
        // wristConfig.Slot0.kV = 0.5 / 10;
        // wristConfig.Slot0.kA = 0.25 / 10; // Acceleration for given TorqueCurrent
        wristConfig.Slot0.kP = 55; // Increase until wrist oscillates
        wristConfig.Slot0.kI = 0; // Don't touch
        wristConfig.Slot0.kD = 16.7; // Increase to reduce overshoot
        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        wristConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        wristConfig.Slot1 = Slot1Configs.from(SlotConfigs.from(wristConfig.Slot0)); //might just have to tune fully
        wristConfig.Slot1.kG = -30; // increase kG when holding coral
        wristConfig.Slot1.kD = 21; // increase kD when holding coral

        wristConfig.MotionMagic.MotionMagicCruiseVelocity = 10; // TODO: Tune
        wristConfig.MotionMagic.MotionMagicAcceleration = 10; // TODO: Tune

        wristConfig.Feedback.RotorToSensorRatio = 10;
        wristConfig.Feedback.SensorToMechanismRatio = 1;
        wristConfig.Feedback.FeedbackRemoteSensorID = ElevatorWristConstants.wristEncoderConfig.canID;
        wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        wristConfig.Feedback.FeedbackRotorOffset = 0;

        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.57; // TODO: Change
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.05; // TODO: Change

        wristConfig.CurrentLimits.StatorCurrentLimit = 80; // TODO: Change
        wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        wristConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        wristConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        wristConfig.TorqueCurrent.TorqueNeutralDeadband = 2;

        wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
}
