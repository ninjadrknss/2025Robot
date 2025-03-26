package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import frc.lib.CTREConfig;
import frc.robot.Robot;

public class IntakeConstants {
    public static final double intakeSpeed = 12;
    public static final double spitSpeed = 2; // Negated in request
    public static final int beamBreakPort = 9;
    public static final int coralDistanceThreshold = 0;
    public static final int stalledCurrentThreshold = 30;

    public static final CTREConfig<TalonFX, TalonFXConfiguration> intakeMotorConfig = new CTREConfig<>(TalonFXConfiguration::new);
    static {
        intakeMotorConfig.withName("Intake Motor")
                .withCanID(51)
                .withBus(Robot.elevatorbus);

        TalonFXConfiguration intakeConfig = intakeMotorConfig.config;
        intakeConfig.Slot0.kP = 0; // Increase until speed oscillates
        intakeConfig.Slot0.kI = 0; // Don't touch
        intakeConfig.Slot0.kD = 0; // Increase until jitter
        intakeConfig.Slot0.kS = 0; // Increase until just before motor starts moving
        intakeConfig.Slot0.kA = 0; //
        intakeConfig.Slot0.kV = 0; //
        intakeConfig.Slot0.kG = 0; // Don't touch

        intakeConfig.Feedback.RotorToSensorRatio = 1;
        intakeConfig.Feedback.SensorToMechanismRatio = 1;

        intakeConfig.CurrentLimits.StatorCurrentLimit = 40; // TODO: Change
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        intakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: CHECK
    }

    public static final CTREConfig<CANrange, CANrangeConfiguration> distanceSensorConfig = new CTREConfig<>(CANrangeConfiguration::new);
    static {
        distanceSensorConfig.withName("Algae Distance Sensor")
                .withCanID(52)
                .withBus(Robot.elevatorbus);

        CANrangeConfiguration distanceSensorConfig = new CANrangeConfiguration();

        distanceSensorConfig.FovParams.FOVCenterX = 0;
        distanceSensorConfig.FovParams.FOVCenterY = 0;
        distanceSensorConfig.FovParams.FOVRangeX = 27; // TODO: tune (in degrees)
        distanceSensorConfig.FovParams.FOVRangeY = 27; // TODO: tune (in degrees)

        distanceSensorConfig.ProximityParams.ProximityThreshold = 0.15; // TODO: tune (in m)
        distanceSensorConfig.ProximityParams.ProximityHysteresis = 0.05; // TODO: tune (in m)
        distanceSensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500; // TODO: tune

        distanceSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq; // TODO: change to 100hz mode?
        distanceSensorConfig.ToFParams.UpdateFrequency = 50; // TODO: tune (in Hz)
    }
}
