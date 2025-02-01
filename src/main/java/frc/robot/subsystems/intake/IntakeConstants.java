package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;
import frc.lib.TalonFXConfig;
import frc.robot.Robot;

public class IntakeConstants {
    public static final TalonFXConfig intakeMotorConfig = new TalonFXConfig()
            .withName("Intake Motor")
            .withCanID(57)
            .withBus(Robot.elevatorbus);

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

    public static final int beamBreakPort = 2;

    public static final int distanceSensorID = 58;
    public static final int algaeDistanceThreshold = 0;

    public static final CANrangeConfiguration distanceSensorConfig = new CANrangeConfiguration();
    static {
        distanceSensorConfig.FovParams.FOVCenterX = 0;
        distanceSensorConfig.FovParams.FOVCenterY = 0;
        distanceSensorConfig.FovParams.FOVRangeX = 27; // TODO: tune (in degrees)
        distanceSensorConfig.FovParams.FOVRangeY = 27; // TODO: tune (in degrees)

        distanceSensorConfig.ProximityParams.ProximityThreshold = 0.4; // TODO: tune (in m)
        distanceSensorConfig.ProximityParams.ProximityHysteresis = 0.05; // TODO: tune (in m)
        distanceSensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500; // TODO: tune

        distanceSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq; // TODO: change to 100hz mode?
        distanceSensorConfig.ToFParams.UpdateFrequency = 50; // TODO: tune (in Hz)
    }

    public static final double intakeSpeed = 0.5;
    public static final double spitSpeed = 0.5; // Negated in request
}
