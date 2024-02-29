package frc.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.subsystems.Flywheel;
import frc.sim.PhysicsSim;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.subsystems.sensors.IR;

public class Intake extends SubsystemBase {
  private TalonFX motor;
  private static Intake instance;
  private IR ir;
  private PositionVoltage positionOut = new PositionVoltage(0, 0, false, 0, 0, false, false, false); 
  
  private final Flywheel flywheel = Flywheel.getInstance();

  private boolean state = false;

  private MechanismLigament2d speed;

  private double uptakeSpeed = Constants.Intake.UptakeSpeed;
  private double intakeSpeed = Constants.Intake.IntakeSpeed;
  private double outtakeSpeed = Constants.Intake.OuttakeSpeed;

  public boolean isBeingIntook = false;
  public boolean isBeingQueued = false;

  public Intake() {
    motor = new TalonFX(RobotMap.Intake.INTAKE_MOTOR);
    motor.optimizeBusUtilization();
    motor.setInverted(true);
    state = false;

    SmartDashboard.setDefaultNumber("Uptake", uptakeSpeed);
    SmartDashboard.setDefaultNumber("Intake", intakeSpeed);
    SmartDashboard.setDefaultNumber("Outtake", outtakeSpeed);

    Mechanism2d intakeMech2d = new Mechanism2d(3, 3);

    MechanismRoot2d root = intakeMech2d.getRoot("intake", 1.2, 0.25);
    speed = root.append(new MechanismLigament2d("intake", 0.1, 0));

    Shuffleboard.getTab("Mechanism2d").add("Intake Mechanism", intakeMech2d);

    if (Robot.isSimulation()) {
      PhysicsSim.getInstance().addTalonFX(motor, 0.001);
    }
  }

  /**
   * Sets the motor's speed to the IntakeSpeed.
   */
  public void motorIn() {
    System.out.println("in");
    motor.set(SmartDashboard.getNumber("Intake", intakeSpeed));
    speed.setColor(new Color8Bit(0, (int) (intakeSpeed * 255), 0));
  }

  /**
   * Sets the motor's speed to the UptakeSpeed.
   */
  public void motorUptake() {
    System.out.println("uptake");
    motor.set(SmartDashboard.getNumber("Uptake", uptakeSpeed));
    speed.setColor(new Color8Bit(0, (int) (uptakeSpeed * 255), 0));
  }

  /**
   * Sets the motor's speed to 0. Note that the motor is not configured to be in
   * brake
   * or coast mode in this subsystem.
   */
  public void motorOff() {
    System.out.println("off");
    motor.set(0);
    speed.setColor(new Color8Bit(0, 0, 0));
  }

  /**
   * Sets the motor's speed to the OuttakeSpeed
   */
  public void motorOut() {
    System.out.println("out");
    motor.set(SmartDashboard.getNumber("Outtake", outtakeSpeed));
    speed.setColor(new Color8Bit(0, 0, (int) (outtakeSpeed * 255)));
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  // COMMANDS
  /**
   * @return Command that sets the motor speed to 0.
   */
  public Command off() {
    isBeingIntook = false;
    isBeingQueued = false;
    return runOnce(() -> motorOff());
  }

  /**
   * @return Command that sets the motor speed to OuttakeSpeed.
   */
  public Command out() {
    isBeingIntook = false;
    isBeingQueued = false;
    return runOnce(() -> motorOut());
  }

  /**
   * @return Command that sets the motor speed to IntakeSpeed.
   */
  public Command in() {
    isBeingIntook = true;
    isBeingQueued = false;
    return runOnce(() -> motorIn());
  }

  public Command queueNote(){ //outake note slighly
    isBeingIntook = false;
    isBeingQueued = true;
    return runOnce(() -> motor.set(-0.05));
  }

  public Command stopNoteForShooting(){
    isBeingIntook = false;
    isBeingQueued = false;

    flywheel.on(500,500).schedule();

    return runOnce(() -> motorOff());
  }

  /**
   * @return Command that sets the motor speed to UptakeSpeed.
   */
  public Command uptake() {
    return runOnce(() -> motorUptake());
  }

  /**
   * @return Command that toggles between intaking and not intaking.
   *         Does NOT take into account other motor modes, ie. If this has been
   *         called,
   *         and the out Command is run, then when this command is run again, the
   *         motor
   *         will just be turned off.
   */
  public Command toggleIntake() {
    return runOnce(
        () -> {
          if (state == true) {
            motorOff();
            state = false;
          } else {
            motorIn();
            state = true;
        }
      });
    }
  
  @Override
  public void periodic() {
    // SmartDashboard.putNumber("IR", leftIR.getValue()); 
  }

  public boolean leftIRTrue() { // Returns true if the ir sensor laser reaches detector
    return ir.leftIR.getValue() > Constants.Intake.IRThreshold; 
  }

  public boolean hasNote() {
      return ((ir.rightIR.getValue() > Constants.Intake.IRThreshold) || (ir.leftIR.getValue() > Constants.Intake.IRThreshold)) && isBeingIntook;
  }

  public boolean noteIsQueued() { // note reversed and is clear from shooter
    return ((ir.rightIR.getValue() < Constants.Intake.IRThreshold) && (ir.leftIR.getValue() < Constants.Intake.IRThreshold)) && isBeingQueued;
  }

  public boolean getisBeingIntook(){
    return isBeingIntook;
  }

  public boolean getisBeingQueued(){
    return isBeingQueued;
  }

  public void motorInWithRotations(double rotations){ 
    motor.setControl(positionOut.withPosition(rotations)); //Not 100% Sure PositionOut uses rotations 
  }
}