package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.lib.TalonFXConfig;
import frc.robot.Robot;

public class ClimbConstants {
    public static int servoPort = 8;
    public static int pivotEncoderID = 0;

    public static TalonFXConfig pivotMotorConfig = new TalonFXConfig()
            .withName("Pivot Motor")
            .withCanID(61)
            .withBus(Robot.riobus);
    static {
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
        pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoderID;
    }

    public static CANcoderConfiguration pivotEncoderConfig = new CANcoderConfiguration();

    static {
        CANcoderConfiguration encoderConfig = pivotEncoderConfig;
        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // TODO: check
    }
}
