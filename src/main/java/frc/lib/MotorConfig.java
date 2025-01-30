package frc.lib;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;

public class MotorConfig {
    public String name = "UNNAMED";
    public int canID = 0;
    public CANBus canbus = Robot.riobus;

    public TalonFXConfiguration config = new TalonFXConfiguration();

    public MotorConfig withName(String name) {
        this.name = name;
        return this;
    }

    public MotorConfig withCanID(int canID) {
        this.canID = canID;
        return this;
    }

    public MotorConfig withBus(CANBus bus) {
        this.canbus = bus;
        return this;
    }

    public MotorConfig withConfig(TalonFXConfiguration config) {
        this.config = config;
        return this;
    }

    public TalonFX createMotor() {
        TalonFX motor = new TalonFX(canID, canbus);
        motor.getConfigurator().apply(config);
        CTREUtil.applyConfiguration(motor, config);
        return motor;
    }
}
