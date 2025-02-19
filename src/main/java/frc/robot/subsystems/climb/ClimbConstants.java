package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import frc.lib.TalonFXConfig;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Degrees;

public class ClimbConstants {
    public static final int servoPort = 8;
    public static final int pivotEncoderID = 0;

    public static final TalonFXConfig pivotMotorConfig = new TalonFXConfig()
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

    public static final CANcoderConfiguration pivotEncoderConfig = new CANcoderConfiguration();

    static {
        CANcoderConfiguration encoderConfig = pivotEncoderConfig;
        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; // TODO: check
    }

    /** All in Degrees */
    public static final Angle changeRate = Degrees.of(3);
    public static final Angle flapStoreAngle = Degrees.of(0);
    public static final Angle flapDeployAngle = Degrees.of(0);

    public static final Angle pivotStoreAngle = Degrees.of(0);
    public static final Angle pivotDeployAngle = Degrees.of(0);
}
