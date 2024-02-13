package frc.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Constants.Arm.EncoderOption;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends ProfiledPIDSubsystem {
  
  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private AnalogPotentiometer potentiometer;
  private ArmFeedforward ff = new ArmFeedforward(Constants.Arm.KA, 0, Constants.Arm.KV, Constants.Arm.KA);
  private double newGravityVolts = 0.0d;

  public Arm() {
    super(
        new ProfiledPIDController(
            Constants.Arm.KP,
            0,
            Constants.Arm.KD,
            new TrapezoidProfile.Constraints(
                Constants.Arm.VELO_MAX,
                Constants.Arm.ACCEL_MAX)),
        0);
    
    potentiometer = new AnalogPotentiometer(RobotMap.Arm.ANALOG_POT_PORT); //TO-DO

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    
    leftMotor = new TalonFX(RobotMap.Arm.LEFT_MOTOR_PORT);
    leftMotor.getConfigurator().apply(feedbackConfigs);

    rightMotor = new TalonFX(RobotMap.Arm.RIGHT_MOTOR_PORT);
    rightMotor.setInverted(true);
    rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

    setGoal(0);
    if (Robot.isSimulation()) {
      SmartDashboard.setDefaultNumber("Sim Distance", 0.0d);
    }
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    leftMotor.setVoltage(newGravityVolts + output + feedforward);
    SmartDashboard.putNumber("Volt out", (output + feedforward));
  }

  @Override
  public double getMeasurement() {
    if (Robot.isSimulation()) {
      return SmartDashboard.getNumber("Sim Distance (rotations)", 0.0d);
    }

    double rotations = 0.0d;
    if (Constants.Arm.ENCODER_TO_USE.equals(EncoderOption.ANALOG_POT)) {
      rotations = (potentiometer.get()+Constants.Arm.POT_OFFSET)/360;
    } else {
      rotations = Constants.Arm.falconToRotations(leftMotor.getPosition().getValue()); //Needs gear ratio? we will find out
    }
    SmartDashboard.putNumber("Arm Measurement", rotations);
    newGravityVolts = (Constants.Arm.KG_PROPORTION * (rotations*360)) * Constants.Arm.KG;
    return rotations;
  }
}
