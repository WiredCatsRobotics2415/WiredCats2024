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
    // motor = new CANSparkMax(RobotMap.Intake.INTAKE_MOTOR,
    // CANSparkMax.MotorType.kBrushless);
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

  public void motorIn() {
    System.out.println("in");
    motor.set(SmartDashboard.getNumber("Intake", intakeSpeed));
  }

  public void motorUptake() {
    System.out.println("uptake");
    motor.set(SmartDashboard.getNumber("Uptake", uptakeSpeed));
  }

  public void motorOff() {
    System.out.println("off");
    motor.set(0);
  }

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
  public Command off() {
    return runOnce(() -> motorOff());
  }

  public Command out() {
    return runOnce(() -> motorOut());
  }

  public Command in() {
    return runOnce(() -> motorIn());
  }

  public Command uptake() {
    return runOnce(() -> motorUptake());
  }

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

  public boolean hasNote() {
    return topIR.getValue() > Constants.Intake.IRThreshold;
  }

  public boolean inShooter() {
    return bottomIR.getValue() > Constants.Intake.IRThreshold;
  }
}