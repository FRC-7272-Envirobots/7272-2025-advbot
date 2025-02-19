package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;

public class ElevatorSubsystem extends SubsystemBase {

    TalonFX m_leftelevator = new TalonFX(RobotConstants.LeftElevatorCanId);
    TalonFX m_rightelevator = new TalonFX(RobotConstants.RightElevartorCanId);
    Follower right_follower;
    TalonFXConfiguration config;

    public double lfourpos = ElevatorConstants.elevatorL4;
    public double lthreerpos = ElevatorConstants.elevatorL3;
    public double ltwopos = ElevatorConstants.elevatorL2;
    public double lonepos = ElevatorConstants.elevatorL1;
    public double intakepos = ElevatorConstants.elevatorIntake;

    DigitalInput toplimitSwitch = new DigitalInput(0);
    DigitalInput bottomlimitSwitch = new DigitalInput(1);

    public ElevatorSubsystem() {
        right_follower = new Follower(m_leftelevator.getDeviceID(), false);
        m_rightelevator.setControl(right_follower);

        // Arm motion logic
        config = new TalonFXConfiguration();
        // config.Slot0.kP = .5;
        // "Nobody uses I" apparently, so dont set it.
        // config.Slot0.kD = 0.056523;
        // config.Slot0.kS = 0.20933;
        // config.Slot0.kV = 0.10312;
        // config.Slot0.kA = 0.00085311;

        // Limit Switch
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;
        // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
        // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 82.0;

        // config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        // config.Slot0.kG = 0.15387;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_leftelevator.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    public void setControl(ControlRequest control) {
        m_leftelevator.setControl(control);
    }

    // basic elevator commands
    public void elevatorUp() {
        m_leftelevator.set(.2);
        // m_rightelevator.set(.2);
        if (toplimitSwitch.get()) {
            m_leftelevator.set(0);
            // m_rightelevator.set(0);
            System.out.println("top limit swtich triggered");
        }
    }

    public void elevatorDown() {
        // m_leftelevator.set(-.2);
        m_rightelevator.set(-.2);
        if (bottomlimitSwitch.get()) {
            m_leftelevator.set(0);
            // m_rightelevator.set(0);
            System.out.println("bottom limit swtich triggered");
        }

    }

    public void elevatorStop() {
        m_leftelevator.set(0);
        // m_rightelevator.set(0);
    }

    private final VoltageOut m_sysidControl = new VoltageOut(0);
    private SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Default ramp rate is acceptable
                    Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                    null, // Default timeout is acceptable
                          // Log state with Phoenix SignalLogger class
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    drive -> m_leftelevator.setControl(m_sysidControl.withOutput(drive.in(Volts))),
                    log -> {
                    },
                    this, ""));

    /**
     * Returns a command that will execute a quasistatic test in the given
     * direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        SignalLogger.start();

        return m_SysIdRoutine.quasistatic(direction);
        // SignalLogger.stop();
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        SignalLogger.start();

        return m_SysIdRoutine.dynamic(direction);
        // SignalLogger.stop();
    }

}
