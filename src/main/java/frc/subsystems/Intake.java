package frc.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.sim.PhysicsSim;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;

public class Intake extends SubsystemBase {
    private TalonFX motor;
    private static Intake instance;
    private AnalogInput closeToFlywheelSensor;
    private AnalogInput closeToIntakeSensor;
    private PositionVoltage positionOut =
            new PositionVoltage(0, 0, false, 0, 0, false, false, false);

    private final Flywheel flywheel = Flywheel.getInstance();

    private MechanismLigament2d speedWidget;

    public boolean isBeingIntook = false;
    public boolean isBeingQueued = false;
    private boolean state = false;

    public Intake() {
        motor = new TalonFX(RobotMap.Intake.INTAKE_MOTOR);
        configureMotor();

        closeToFlywheelSensor = new AnalogInput(RobotMap.Intake.FLYWHEEL_IR);
        closeToIntakeSensor = new AnalogInput(RobotMap.Intake.INTAKE_IR);

        configureSmartDashboardWidgets();

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(motor, 0.001);
        }
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private void configureMotor() {
        motor.optimizeBusUtilization();
        motor.setInverted(true);
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentThreshold(40));
        //motor.getConfigurator().apply(new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0.2));
    }

    private void configureSmartDashboardWidgets() {
        Mechanism2d intakeMech2d = new Mechanism2d(3, 3);

        MechanismRoot2d root = intakeMech2d.getRoot("intake", 1.2, 0.25);
        speedWidget = root.append(new MechanismLigament2d("intake", 0.1, 0));

        Shuffleboard.getTab("Mechanism2d").add("Intake Mechanism", intakeMech2d);
    }

    /** Sets the motor's speed to the IntakeSpeed. */
    private void motorIn() {
        Logger.log(this, LogLevel.INFO, "motor in at " + Constants.Intake.IntakeSpeed);
        motor.set(Constants.Intake.IntakeSpeed);
        speedWidget.setColor(new Color8Bit(0, (int) (Constants.Intake.IntakeSpeed * 255), 0));
    }

    /** Sets the motor's speed to the UptakeSpeed. */
    private void motorUptake() {
        Logger.log(this, LogLevel.INFO, "motor uptaking at " + Constants.Intake.UptakeSpeed);
        motor.set(Constants.Intake.UptakeSpeed);
        speedWidget.setColor(new Color8Bit(0, (int) (Constants.Intake.UptakeSpeed * 255), 0));
    }

    /**
     * Sets the motor's speed to 0. Note that the motor is not configured to be in brake or coast
     * mode in this subsystem.
     */
    private void motorOff() {
        Logger.log(this, LogLevel.INFO, "motor off");
        motor.set(0);
        speedWidget.setColor(new Color8Bit(0, 0, 0));
    }

    /** Sets the motor's speed to the OuttakeSpeed */
    private void motorOut() {
        Logger.log(this, LogLevel.INFO, "motor outtaking at " + Constants.Intake.OuttakeSpeed);
        motor.set(SmartDashboard.getNumber("Outtake", Constants.Intake.OuttakeSpeed));
        speedWidget.setColor(new Color8Bit(0, 0, (int) (Constants.Intake.OuttakeSpeed * 255)));
    }

    // COMMANDS
    /**
     * @return Command that sets the motor speed to 0.
     */
    public Command off() {
        return runOnce(
                () -> {
                    isBeingIntook = false;
                    isBeingQueued = false;
                    state = false;
                    motorOff();
                });
    }

    /**
     * @return Command that sets the motor speed to OuttakeSpeed.
     */
    public Command out() {
        isBeingIntook = false;
        isBeingQueued = false;
        return runOnce(
                () -> {
                    isBeingIntook = false;
                    isBeingQueued = false;
                    motorOut();
                });
    }

    /**
     * @return Command that operates the complete intake using the IR sensors. The Note is intaked then cleared of the flywheels.
     */

    public Command intakeNote(){
        return new SequentialCommandGroup(
          in(),
          new InstantCommand(() -> {state = true;}),
          new WaitUntilCommand(() -> hasNote()),
          new InstantCommand(() -> {state = false;}),
          new WaitCommand(0.2),
          stopNoteForShooting() 
        ); 
    }

    /**
     * @return Command that sets the motor speed to IntakeSpeed.
     */
    public Command in() {
        return runOnce(
                () -> {
                    isBeingIntook = true;
                    isBeingQueued = false;
                    motorIn();
                });
    }

    /**
     * @return Command used in conjunction with the IR sensor to stop the motor
     * from queueing the note.
     */
    public Command stopNoteForShooting() {
        return runOnce(
                () -> {
                    isBeingIntook = false;
                    isBeingQueued = false;
                    motorOff();
                });
    }

    /**
     * @return Command that sets the motor speed to UptakeSpeed.
     */
    public Command uptake() {
        return runOnce(() -> motorUptake());
    }

    /**
     * @return Command that toggles between intaking and not intaking. Does NOT take into account
     *     other motor modes, ie. If this has been called, and the out Command is run, then when
     *     this command is run again, the motor will just be turned off.
     */
    public Command toggleIntake() {
        return runOnce(
                () -> {
                    if (state == true) {
                        off().schedule();
                        state = false;
                    } else {
                        intakeNote().schedule();
                        state = true;
                    }
                });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IR Sensor", closeToFlywheelSensor.getValue());
        SmartDashboard.putBoolean("Intake isOn", isBeingIntook);
    }

    /**
     * @return true if the note has been intook (used to signal intake off)
     */
    public boolean hasNote() {
        // return ((ir.rightIR.getValue() > Constants.Intake.IRThreshold) || (ir.leftIR.getValue() >
        // Constants.Intake.IRThreshold)) && isBeingIntook;
        return (closeToFlywheelSensor.getValue() < Constants.Intake.IRThreshold) && isBeingIntook;
    }

    public boolean hasNoteIntakingOrNot() {
        return (closeToFlywheelSensor.getValue() < Constants.Intake.IRThreshold);
    }

    /**
     * @return true if note is queued for shooting (used to clear note from flywheel)
     */
    public boolean noteIsQueued() {
        return (closeToFlywheelSensor.getValue() > Constants.Intake.IRThreshold) && isBeingQueued;
        // return ((ir.rightIR.getValue() < Constants.Intake.IRThreshold) && (ir.leftIR.getValue() <
        // Constants.Intake.IRThreshold)) && isBeingQueued;
    }

    public boolean isBeingIntook() {
        return isBeingIntook;
    }

    public boolean isBeingQueued() {
        return isBeingQueued;
    }

    public void motorInWithRotations(double rotations) {
        motor.setControl(
                positionOut.withPosition(rotations)); // Not 100% Sure PositionOut uses rotations
    }

    /* 
    public Command fire() {
        return new SequentialCommandGroup(
            toggleIntake(),
            new WaitCommand(0.5),
            toggleIntake() 
        );
    }
    */ 
}
