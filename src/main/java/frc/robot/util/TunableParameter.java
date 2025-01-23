package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class TunableParameter {
    private static final ArrayList<TunableParameter> parameters = new ArrayList<>();

    public interface Callback {
        void callback(double value);
    }

    private final String name;
    private double lastValue;
    private final Callback callback;

    public TunableParameter(String name, double initialValue, Callback callback) {
        this.name = name;
        this.lastValue = initialValue;
        this.callback = callback;

        SmartDashboard.putNumber(name, initialValue);

        parameters.add(this);
    }


    private void fetch() {
        double value = SmartDashboard.getNumber(name, lastValue);
        if (value != lastValue) {
            lastValue = value;
            callback.callback(value);
        }
    }

    public static void updateAll() {
        for (TunableParameter parameter : parameters) parameter.fetch();
    }
}
