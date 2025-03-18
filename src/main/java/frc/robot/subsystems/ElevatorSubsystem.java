package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.Constants.RobotConstants;

public class ElevatorSubsystem extends SubsystemBase {

    TalonFX m_leftelevator = new TalonFX(RobotConstants.LeftElevatorCanId);
    TalonFX m_rightelevator = new TalonFX(RobotConstants.RightElevartorCanId);
    Follower right_follower;
    TalonFXConfiguration config;



 

    public ElevatorSubsystem() {
        right_follower = new Follower(m_leftelevator.getDeviceID(), false);
        m_rightelevator.setControl(right_follower);

        // Arm motion logic
        config = new TalonFXConfiguration();
         config.Slot0.kP = 0.1;
        //"Nobody uses I" apparently, so dont set it.
         config.Slot0.kD = 0.0087045;
         config.Slot0.kS = 0.034662;
         config.Slot0.kV = 0.10919;
         config.Slot0.kA = 0.0015569;


         config.MotionMagic.MotionMagicCruiseVelocity = 250; // (12 inches / second) * (1 sprocket rotation / 1.8 inches ) * (90 motor rotations / sprocket rotation) = ~191 motor roations / second.8 inches / 
         config.MotionMagic.MotionMagicAcceleration = config.MotionMagic.MotionMagicCruiseVelocity * 2; // .5 seconds to reach full speed
         config.MotionMagic.MotionMagicJerk = config.MotionMagic.MotionMagicAcceleration * 10; // spread jerk over .1 second

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

    private void setControl(ControlRequest control) {
        m_leftelevator.setControl(control);
    }

    // basic elevator commands
 
        
    public void setElevatorL1() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        m_leftelevator.setControl(m_request.withPosition(100));
    }
    public void setElevatorL2() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        m_leftelevator.setControl(m_request.withPosition(200));
    }
    public void setElevatorL0() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        m_leftelevator.setControl(m_request.withPosition(0));
    }
    public void setElevatorL4() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        m_leftelevator.setControl(m_request.withPosition(420));
    }
    public void setElevatorL3() {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        m_leftelevator.setControl(m_request.withPosition(300));
    }
    public void elevatorUp() {
        m_leftelevator.set(.2);
    }

    public void elevatorDown() {
        m_leftelevator.set(-.2);

    }

    
    public void elevatorStop() {
        m_leftelevator.set(0);
    }

    public Command elevatorUpStop() {
        return Commands.startEnd(() -> {this.elevatorUp();}, () -> {this.elevatorStop();}, this);
    }


    public Command elevatorDownStop() {
        return Commands.startEnd(() -> {this.elevatorDown();}, () -> {this.elevatorStop();}, this);
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
                    this, "elev"));

    /**
     * Returns a command that will execute a quasistatic test in the given
     * direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        //SignalLogger.start();

        return m_SysIdRoutine.quasistatic(direction);
        // SignalLogger.stop();
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        //SignalLogger.start();

        return m_SysIdRoutine.dynamic(direction);
        // SignalLogger.stop();
    }

}
