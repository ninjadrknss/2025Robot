package frc.lib;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
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
