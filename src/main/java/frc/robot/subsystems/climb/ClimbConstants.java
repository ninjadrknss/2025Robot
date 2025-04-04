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
    public static final int flapServoPort = 8;
    public static final int rachetServoPort = 7;

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
        pivotConfig.Slot0.kG = 0.95644; // Increase until arm moved
        pivotConfig.Slot0.kS = 0;  // Increase until just before motor starts moving
        pivotConfig.Slot0.kV = 0;  //
        pivotConfig.Slot0.kP = 18.00;   // Increase until speed oscillates
        pivotConfig.Slot0.kI = 0;       // Don't touch
        pivotConfig.Slot0.kD = 3.0;  // Increase until jitter
        pivotConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 0; // TODO: Tune
        pivotConfig.MotionMagic.MotionMagicAcceleration = 400; // TODO: Tune
        pivotConfig.MotionMagic.MotionMagicJerk = 4000; // TODO: Tune

        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoderConfig.canID;
        pivotConfig.Feedback.SensorToMechanismRatio = 1;
        pivotConfig.Feedback.RotorToSensorRatio = 100;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 100;
        pivotConfig.CurrentLimits.SupplyCurrentLowerTime = 2.5;
        pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;

        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // TODO: Change
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 200.0/360;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // TODO: Change
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 10.0/360;
    }

    /** All in Degrees */
    public static final Angle flapStoreAngle = Units.Degrees.of(90);
    public static final Angle flapDeployAngle = Units.Degrees.of(360);

    public static final Angle pivotStoreAngle = Units.Degrees.of(10);
    public static final Angle pivotDeployAngle = Units.Degrees.of(145);
    public static final Angle pivotClimbAngle = Units.Degrees.of(30);

    public static final double rachetActive = 1;
    public static final double rachetInActive = -1;
}
