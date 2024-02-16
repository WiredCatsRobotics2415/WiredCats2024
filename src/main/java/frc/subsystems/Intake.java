package frc.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX motor;
  private static Intake instance;
  private AnalogInput topIR;
  private AnalogInput bottomIR;

  private boolean state = false;

  public Intake() {
    motor = new TalonFX(RobotMap.Intake.INTAKE_MOTOR);
    topIR = new AnalogInput(RobotMap.Intake.TOP_IR);
    bottomIR = new AnalogInput(RobotMap.Intake.BOTTOM_IR);
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  //COMMANDS
  public Command off() {
    return runOnce(() -> motorOff());
  }

  public Command out() {
    return runOnce(() -> motorOut());
  }

  public Command in() {
    return runOnce(() -> motorIn());
  }

  public Command toggleIntake() {
    return runOnce(
        () -> {
          if (state == true) {
            motorOff();
          } else {
            motorIn();
          }
        });
  }

  public Command startOutake() {
    return runOnce(() -> motorOut());
  }

  public Command stopOutake() {
    return runOnce(() -> motorOff());
  }

  //MOTOR METHODS
  public void motorIn() {
    Logger.log(this, LogLevel.DEBUG, "Motor in");
    motor.set(-Constants.Intake.IntakeSpeed);
    state = true;
  }

  public void motorOff() {
    Logger.log(this, LogLevel.DEBUG, "Motor off");
    motor.set(0);
    state = false;
  }

  public void motorOut() {
    Logger.log(this, LogLevel.DEBUG, "Motor out");
    motor.set(Constants.Intake.IntakeSpeed);
    state = true;
  }

  //SENSOR METHODS
  public boolean hasNote() {
    return topIR.getValue() > Constants.Intake.IRThreshold;
  }

  public boolean inShooter() {
    return bottomIR.getValue() > Constants.Intake.IRThreshold;
  }
}