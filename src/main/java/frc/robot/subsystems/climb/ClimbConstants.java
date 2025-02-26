package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.CTREConfig;
import frc.robot.Robot;

public class ClimbConstants {
    public static final int servoPort = 8;

    public static final CTREConfig<CANcoder, CANcoderConfiguration> pivotEncoderConfig = new CTREConfig<>(CANcoderConfiguration::new);
    static {
        pivotEncoderConfig.withName("Climber Pivot Encoder")
                .withCanID(0)
                .withBus(Robot.riobus);
        CANcoderConfiguration encoderConfig = pivotEncoderConfig.config;
        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // TODO: check
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> pivotMotorConfig = new CTREConfig<>(TalonFXConfiguration::new);
    static {
        pivotMotorConfig.withName("Pivot Motor")
                        .withCanID(61)
                        .withBus(Robot.riobus);

        TalonFXConfiguration pivotConfig = pivotMotorConfig.config;
        pivotConfig.Slot0.kP = 0; // Increase until speed oscillates
        pivotConfig.Slot0.kI = 0; // Don't touch
        pivotConfig.Slot0.kD = 0; // Increase until jitter
        pivotConfig.Slot0.kS = 0; // Increase until just before motor starts moving
        pivotConfig.Slot0.kA = 0; //
        pivotConfig.Slot0.kV = 0; //
        pivotConfig.Slot0.kG = 0; // Increase until arm doesnt move

        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoderConfig.canID;
        pivotConfig.Feedback.SensorToMechanismRatio = 1;
        pivotConfig.Feedback.RotorToSensorRatio = 125;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // TODO: make sure spencer adds a easy way to disconnect power
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: Check
    }

    /** All in Degrees */
    public static final Angle changeRate = Units.Degrees.of(3);
    public static final Angle flapStoreAngle = Units.Degrees.of(0);
    public static final Angle flapDeployAngle = Units.Degrees.of(0);

    public static final Angle pivotStoreAngle = Units.Degrees.of(0);
    public static final Angle pivotDeployAngle = Units.Degrees.of(0);
}
