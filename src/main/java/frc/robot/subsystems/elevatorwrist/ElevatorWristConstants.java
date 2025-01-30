package frc.robot.subsystems.elevatorwrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.lib.MotorConfig;
import frc.robot.Robot;

public class ElevatorWristConstants {
    public static int homeCANcoderID = 0;

    public static MotorConfig rightElevatorMotorConfig = new MotorConfig()
            .withName("Right Elevator Motor")
            .withCanID(0)
            .withBus(Robot.elevatorbus);
    static {
        TalonFXConfiguration leaderConfig = rightElevatorMotorConfig.config;
        leaderConfig.Slot0.kP = 0; // Increase until elevator oscillates
        leaderConfig.Slot0.kI = 0; // Don't touch
        leaderConfig.Slot0.kD = 0; // Increase until jitter
        leaderConfig.Slot0.kS = 0; // Increase until just before motor starts moving
        leaderConfig.Slot0.kG = 0; // Increase until elevator holds steady
        leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        leaderConfig.Feedback.RotorToSensorRatio = 1; // TODO: CHANGE
        leaderConfig.Feedback.SensorToMechanismRatio = 1; // TODO: CHANGE

        leaderConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder; // TODO: might be ReverseLimit
        leaderConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = homeCANcoderID; // TODO: might be ReverseLimit

    }

    public static MotorConfig leftElevatorMotorConfig = new MotorConfig()
            .withName("Left Elevator Motor")
            .withCanID(0)
            .withBus(Robot.elevatorbus);
    static {
        TalonFXConfiguration followerConfig = leftElevatorMotorConfig.config;
    }

    public static MotorConfig wristMotorConfig = new MotorConfig()
            .withName("Wrist Motor")
            .withCanID(0)
            .withBus(Robot.drivebus);
    static {
        TalonFXConfiguration wristConfig = wristMotorConfig.config;
        wristConfig.Slot0.kP = 0;
        wristConfig.Slot0.kI = 0;
        wristConfig.Slot0.kD = 0;
        wristConfig.Slot0.kS = 0;
        wristConfig.Slot0.kG = 0;
        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        wristConfig.Feedback.RotorToSensorRatio = 1; // TODO: CHANGE
        wristConfig.Feedback.SensorToMechanismRatio = 1; // TODO: CHANGE
        wristConfig.Feedback.FeedbackRemoteSensorID = ElevatorWristConstants.wristCANcoder;
        wristConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    }

    public static int wristMotor = 0;
    public static int wristCANcoder = 0;
}
