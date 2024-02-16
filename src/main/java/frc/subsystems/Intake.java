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
      //motor = new CANSparkMax(RobotMap.Intake.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
      motor = new TalonFX(0);
      topIR = new AnalogInput(RobotMap.Intake.TOP_IR);
      bottomIR = new AnalogInput(RobotMap.Intake.BOTTOM_IR); 
      motor.setInverted(true);
      state = false;
  }

  public void motorIn() {
      System.out.println("in");
      motor.set(-Constants.Intake.IntakeSpeed);
  }

  public void motorOff() {
    System.out.println("off");
    motor.set(0);
  }

  public void motorOut() {
    System.out.println("out");
    motor.set(Constants.Intake.IntakeSpeed);
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