package frc.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import frc.robot.Robot;

import java.util.function.Supplier;

public class CTREConfig<Device extends ParentDevice, Config extends ParentConfiguration> {
    public String name = "UNNAMED";
    public int canID = 0;
    public CANBus canbus = Robot.riobus;
    public Config config;

    public CTREConfig(Supplier<Config> configSupplier) {
        this.config = configSupplier.get();
    }

    public CTREConfig<Device, Config> withName(String name) {
        this.name = name;
        return this;
    }

    public CTREConfig<Device, Config> withCanID(int canID) {
        this.canID = canID;
        return this;
    }

    public CTREConfig<Device, Config> withBus(CANBus bus) {
        this.canbus = bus;
        return this;
    }

    public Device createDevice(DeviceSupplier<Device> deviceSupplier) {
        Device device = deviceSupplier.get(canID, canbus);
        CTREUtil.applyConfiguration(device, config);

        if (device instanceof TalonFX talon) {
            StatusSignal<Angle> positionSignal = talon.getPosition();
            StatusSignal<AngularVelocity> velocitySignal = talon.getVelocity();
            StatusSignal<Voltage> voltageSignal = talon.getMotorVoltage();
            StatusSignal<Current> currentStatorSignal = talon.getStatorCurrent();
            StatusSignal<Current> currentSupplySignal = talon.getSupplyCurrent();

            BaseStatusSignal[] signals = new BaseStatusSignal[] {
                    positionSignal, velocitySignal, voltageSignal,
                    currentStatorSignal, currentSupplySignal };

            CTREUtil.tryUntilOK(() -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals), talon.getDeviceID());
            CTREUtil.tryUntilOK(talon::optimizeBusUtilization, talon.getDeviceID());
        }
        return device;
    }

    @Override
    public String toString() {
        return name + ": " + canID + " @ " + canbus;
    }

    public interface DeviceSupplier<Device> {
        Device get(int canID, CANBus canbus);
    }
}
