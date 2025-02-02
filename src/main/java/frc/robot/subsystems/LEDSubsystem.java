package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Taken from FIRST Robotics Competition Team 9496, Lynk

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem instance = null;

    private final CANdle candle = new CANdle(0, "rio");

    private static BaseState baseState = BaseState.DISABLED;
    private static TempState tempState = null;
    private static BaseState lastBaseState = null;
    private static TempState lastTempState = null;

    private static double tempStateExpiry = 0.0;
    private static final Timer tempStateTimer = new Timer();

    private static double blinkInterval = 0.25;
    private static final Timer blinkTimer = new Timer();
    private static boolean blinkOff = false;

    public static class Color {
        private final int R, G, B;

        public Color(int r, int g, int b) {
            R = r;
            G = g;
            B = b;
        }
    }

    public enum BaseState {
        DISABLED,
        READY,
    }

    public enum TempState {
        ERROR
    }

    @SuppressWarnings("unused")
    public static final class Colors {
        public static final Color off = new LEDSubsystem.Color(0, 0, 0);
        public static final Color red = new LEDSubsystem.Color(255, 0, 0);
        public static final Color green = new LEDSubsystem.Color(0, 255, 0);
        public static final Color blue = new LEDSubsystem.Color(0, 0, 255);
        public static final Color cyan = new LEDSubsystem.Color(0, 255, 255);
        public static final Color magenta = new LEDSubsystem.Color(255, 0, 255);
        public static final Color yellow = new LEDSubsystem.Color(255, 255, 0);
        public static final Color white = new LEDSubsystem.Color(255, 255, 255);
        public static final Color disabled = new LEDSubsystem.Color(200, 0, 0);
    }

    public static LEDSubsystem getInstance() {
        if (instance == null) instance = new LEDSubsystem();
        return instance;
    }

    private LEDSubsystem() {
        candle.configBrightnessScalar(0.50);
        candle.configLEDType(LEDStripType.GRB);
        candle.configV5Enabled(true);
        candle.configLOSBehavior(false); // TODO: true -- why is this triggering?
        setBaseState(BaseState.READY);
    }

    public static void setBaseState(BaseState newState) {
        baseState = newState;
    }

    public static void setTempState(TempState newState) {
        tempState = newState;
    }

    public static void clearTempState() {
        tempState = null;
    }

    public void setColor(Color color) {
        System.out.printf("Setting color (%d, %d, %d)%n", color.R, color.G, color.B);
        candle.setLEDs(color.R, color.G, color.B);
    }

    public void setRainbow() {
        candle.clearAnimation(0);
        candle.animate(new RainbowAnimation(0.50, 0.5, 68, false, 8));
    }

    private Color tempStateColor(TempState state) {
        return state == TempState.ERROR ? Colors.red : Colors.off;
    }

    private Color baseStateColor(BaseState state) {
        return switch (state) {
            case DISABLED -> Colors.disabled;
            case READY -> Colors.green;
        };
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("LED/Base state", baseState == null ? "NULL" : baseState.toString());
        SmartDashboard.putString("LED/Temp state", tempState == null ? "NULL" : tempState.toString());

        // If the temporary state is active...
        if (tempState != null) {
            if (tempState == lastTempState) {
                // Temporary state unchanged
                if (tempStateExpiry > 0.0 && tempStateTimer.hasElapsed(tempStateExpiry)) {
                    // Temporary state has expired, and base state should be shown
                    tempState = null;
                } else if (blinkTimer.hasElapsed(blinkInterval)) { // Temporary state is active, but might need to be blinked
                    blinkOff = !blinkOff;
                    setColor(blinkOff ? Colors.off : tempStateColor(tempState));
                    blinkTimer.restart();
                }
            } else {
                // Start new temporary state
                setColor(tempStateColor(tempState));
                blinkOff = false;
                blinkTimer.restart();
                if (tempState == TempState.ERROR) {
                    blinkInterval = 0.10;
                    tempStateExpiry = 0.80;
                    tempStateTimer.restart();
                } else {
                    blinkInterval = 0.20;
                    tempStateExpiry = 0.0;
                }
            }
        }

        // Check for a changed base state, or a dropped temporary state
        if (tempState == null && baseState != null &&
                (baseState != lastBaseState || lastTempState != null))
            setColor(baseStateColor(baseState));

        // Update the last states processed for reference in the next iteration
        lastTempState = tempState;
        lastBaseState = baseState;
    }
}