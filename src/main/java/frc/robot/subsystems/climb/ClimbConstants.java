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
                .withCanID(60)
                .withBus(Robot.riobus);
        CANcoderConfiguration encoderConfig = pivotEncoderConfig.config;
        encoderConfig.MagnetSensor.MagnetOffset = 0.32763671875;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    }

    public static final CTREConfig<TalonFX, TalonFXConfiguration> pivotMotorConfig = new CTREConfig<>(TalonFXConfiguration::new);
    static {
        pivotMotorConfig.withName("Pivot Motor")
                        .withCanID(61)
                        .withBus(Robot.riobus);

        TalonFXConfiguration pivotConfig = pivotMotorConfig.config;
        pivotConfig.Slot0.kP = 18.00;   // Increase until speed oscillates
        pivotConfig.Slot0.kI = 0;       // Don't touch
        pivotConfig.Slot0.kD = 3.0;  // Increase until jitter
        pivotConfig.Slot0.kS = 0;  // Increase until just before motor starts moving
        pivotConfig.Slot0.kA = 0;  //
        pivotConfig.Slot0.kV = 0;  //
        pivotConfig.Slot0.kG = 0.95644; // Increase until arm moved
        pivotConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoderConfig.canID;
        pivotConfig.Feedback.SensorToMechanismRatio = 1;
        pivotConfig.Feedback.RotorToSensorRatio = 100;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // TODO: make sure spencer adds a easy way to disconnect power
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 100;
        pivotConfig.CurrentLimits.SupplyCurrentLowerTime = 2.5;
        pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;

        // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 200/360;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 10/360;
    }

    /** All in Degrees */
    public static final Angle changeRate = Units.Degrees.of(10);
    public static final Angle flapStoreAngle = Units.Degrees.of(90);
    public static final Angle flapDeployAngle = Units.Degrees.of(360);

    public static final Angle pivotStoreAngle = Units.Degrees.of(10);
    public static final Angle pivotDeployAngle = Units.Degrees.of(145);

    public static final Angle pivotSetpointTolerance = Units.Degrees.of(2);
}
