package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ClimbSubsystem extends SubsystemBase {
    private static ClimbSubsystem instance;

    // Motor, sensor, and servo devices.
    private final TalonFX pivotMotor = ClimbConstants.pivotMotorConfig.createDevice(TalonFX::new);
    private final PositionTorqueCurrentFOC pivotControl = new PositionTorqueCurrentFOC(0)
            .withSlot(0);
    private final CANcoder pivotEncoder = ClimbConstants.pivotEncoderConfig.createDevice(CANcoder::new);
    private final Servo flapServo = new Servo(ClimbConstants.servoPort);

    // Define the states of the climber.
    public enum ClimbState {
        IDLE,
        STORE,
        DEPLOY,
        CONTROL
    }

    private ClimbState currentState = ClimbState.IDLE;

    // Target angles for pivot and flap (tunable via ClimbConstants).
    private Angle targetPivotAngle = ClimbConstants.pivotStoreAngle;
    private Angle targetFlapAngle = ClimbConstants.flapStoreAngle;

    public static ClimbSubsystem getInstance() {
        if (instance == null) instance = new ClimbSubsystem();
        return instance;
    }
    
    private ClimbSubsystem() {
    }

   private final SysIdRoutine climbIdRoutine = new SysIdRoutine(
       new SysIdRoutine.Config(
           null,
           Units.Volts.of(4),
           null,
           state -> SignalLogger.writeString("SysIdClimberState", state.toString())
       ),
       new SysIdRoutine.Mechanism(
           (volts) -> pivotMotor.setControl(new VoltageOut(volts.in(Units.Volts))),
           null,
           this
       )
   );

    public Command climberQuasistaticRoutine(boolean forward) {
        return climbIdRoutine.quasistatic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    public Command climberDynamicRoutine(boolean forward) {
        return climbIdRoutine.dynamic(forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
    }

    public void setTargetPivotAngle(Angle angle) {
        targetPivotAngle = angle;
        currentState = ClimbState.CONTROL;
    }

    public void setTargetFlapAngle(Angle angle) {
        targetFlapAngle = angle;
    }

    public void requestStore() {
        targetPivotAngle = ClimbConstants.pivotStoreAngle;
        targetFlapAngle = ClimbConstants.flapStoreAngle;
        currentState = ClimbState.STORE;
    }

    public void requestDeploy() {
        targetPivotAngle = ClimbConstants.pivotDeployAngle;
        targetFlapAngle = ClimbConstants.flapDeployAngle;
        currentState = ClimbState.DEPLOY;
    }
    
    public void increasePivotAngle() {
        double aChange = ClimbConstants.changeRate.in(Units.Degrees);
        modifyPivotAngle(Units.Degrees.of(aChange));
    }

    public void decreasePivotAngle() {
        double aChange = -1 * ClimbConstants.changeRate.in(Units.Degrees);
        modifyPivotAngle(Units.Degrees.of(aChange));
    }

    public void modifyPivotAngle(Angle delta) {
        double newAngle = targetPivotAngle.in(Units.Degrees) + delta.in(Units.Degrees);

        double minAngle = ClimbConstants.pivotDeployAngle.in(Units.Degrees);
        double maxAngle = ClimbConstants.pivotStoreAngle.in(Units.Degrees);

        /*if (newAngle < minAngle) {
            newAngle = minAngle;
        } else if (newAngle > maxAngle) {
            newAngle = maxAngle;
        }*/

        targetPivotAngle = Units.Degrees.of(newAngle);
        currentState = ClimbState.CONTROL;
    }

    @Override
    public void periodic() {
        pivotControl.withPosition(targetPivotAngle);
        pivotMotor.setControl(pivotControl);
        flapServo.set(targetFlapAngle.in(Units.Degrees));
    }
}
