package frc.lib;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Robot;

public class TalonFXConfig {
    public String name = "UNNAMED";
    public int canID = 0;
    public CANBus canbus = Robot.riobus;

    public TalonFXConfiguration config = new TalonFXConfiguration();

    public TalonFXConfig withName(String name) {
        this.name = name;
        return this;
    }

    public TalonFXConfig withCanID(int canID) {
        this.canID = canID;
        return this;
    }

    public TalonFXConfig withBus(CANBus bus) {
        this.canbus = bus;
        return this;
    }

    public TalonFXConfig withConfig(TalonFXConfiguration config) {
        this.config = config;
        return this;
    }

    public TalonFX createMotor() {
        TalonFX motor = new TalonFX(canID, canbus);
        CTREUtil.applyConfiguration(motor, config);
        return motor;
    }
}
