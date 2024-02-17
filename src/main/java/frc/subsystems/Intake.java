package frc.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.subsystems.Flywheel;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX motor;
  private static Intake instance;
  private AnalogInput rightIR;
  private AnalogInput leftIR;
  private PositionVoltage positionOut = new PositionVoltage(0, 0, false, 0, 0, false, false, false); 
  
  private final Flywheel flywheel = Flywheel.getInstance();

  private boolean state = false;
  public boolean isBeingIntook = false;
  public boolean isBeingQueued = false;

  public Intake() {
      //motor = new CANSparkMax(RobotMap.Intake.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
      motor = new TalonFX(0);
      rightIR = new AnalogInput(RobotMap.Intake.TOP_IR);
      leftIR = new AnalogInput(RobotMap.Intake.BOTTOM_IR); 
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
    isBeingIntook = false;
    isBeingQueued = false;
    return runOnce(() -> motorOff());
  }

  public Command out() {
    isBeingIntook = false;
    isBeingQueued = false;
    return runOnce(() -> motorOut());
  }

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

    flywheel.on();

    return runOnce(() -> motorOff());
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
      return ((rightIR.getValue() > Constants.Intake.IRThreshold) || (leftIR.getValue() > Constants.Intake.IRThreshold)) && isBeingIntook;
  }

  public boolean noteIsQueued() { // note reversed and is clear from shooter
    return ((rightIR.getValue() < Constants.Intake.IRThreshold) && (leftIR.getValue() < Constants.Intake.IRThreshold)) && isBeingQueued;
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