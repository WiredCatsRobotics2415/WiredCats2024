package frc.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX motor;
  private static Intake instance;
  private AnalogInput topIR;
  private AnalogInput bottomIR;

  private boolean state = false;

  private double uptakeSpeed = Constants.Intake.UptakeSpeed;
  private double intakeSpeed = Constants.Intake.IntakeSpeed;
  private double outtakeSpeed = Constants.Intake.OuttakeSpeed;

  public Intake() {
    motor = new TalonFX(RobotMap.Intake.INTAKE_MOTOR);
    motor.optimizeBusUtilization();
    topIR = new AnalogInput(RobotMap.Intake.TOP_IR);
    bottomIR = new AnalogInput(RobotMap.Intake.BOTTOM_IR);
    motor.setInverted(true);
    state = false;

    SmartDashboard.setDefaultNumber("Uptake", uptakeSpeed);
    SmartDashboard.setDefaultNumber("Intake", intakeSpeed);
    SmartDashboard.setDefaultNumber("Outtake", outtakeSpeed);
  }

  /**
   * Sets the motor's speed to the IntakeSpeed.
   */
  public void motorIn() {
    System.out.println("in");
    motor.set(SmartDashboard.getNumber("Intake", intakeSpeed));
  }

  /**
   * Sets the motor's speed to the UptakeSpeed.
   */
  public void motorUptake() {
    System.out.println("uptake");
    motor.set(SmartDashboard.getNumber("Uptake", uptakeSpeed));
  }

  /**
   * Sets the motor's speed to 0. Note that the motor is not configured to be in brake
   * or coast mode in this subsystem.
   */
  public void motorOff() {
    System.out.println("off");
    motor.set(0);
  }

  /**
   * Sets the motor's speed to the OuttakeSpeed
   */
  public void motorOut() {
    System.out.println("out");
    motor.set(SmartDashboard.getNumber("Outtake", outtakeSpeed));
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
    return runOnce(() -> motorOff());
  }

  /**
   * @return Command that sets the motor speed to OuttakeSpeed.
   */
  public Command out() {
    return runOnce(() -> motorOut());
  }

  /**
   * @return Command that sets the motor speed to IntakeSpeed.
   */
  public Command in() {
    return runOnce(() -> motorIn());
  }

  /**
   * @return Command that sets the motor speed to UptakeSpeed.
   */
  public Command uptake() {
    return runOnce(() -> motorUptake());
  }

  /**
   * @return Command that toggles between intaking and not intaking.
   * Does NOT take into account other motor modes, ie. If this has been called,
   * and the out Command is run, then when this command is run again, the motor
   * will just be turned off.
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

  /**
   * @return true if the IR sensor placed to detect a note intaked is tripped.
   */
  public boolean hasNote() {
    return topIR.getValue() > Constants.Intake.IRThreshold;
  }

  /**
   * @return true if the IR sensor placed to detect if the note is in the shooter is tripped.
   */
  public boolean inShooter() {
    return bottomIR.getValue() > Constants.Intake.IRThreshold;
  }
}