package frc.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
    // Initialize motors 
    private TalonFX right; 
    private TalonFX left; 
    private PositionVoltage positionOut = new PositionVoltage(0, 0, false, 0, 0, false, false, false); 
    // Guarantee only one instance of the Climber class exists 
    private static Climber instance;

    // Declare limit switches 
    private final DigitalInput leftTopSwitch;
    private final DigitalInput rightTopSwitch; 
    private final DigitalInput leftBotSwitch; 
    private final DigitalInput rightBotSwitch; 

    private Climber() {
        //From the back of the robot
        right = new TalonFX(RobotMap.Climber.CLIMBER_MASTER, RobotMap.CANBUS_NAME); // Initialize right motor
        left = new TalonFX(RobotMap.Climber.CLIMBER_FOLLOWER, RobotMap.CANBUS_NAME); // Initialize left motor

        // Initialize limit switches
        leftTopSwitch = new DigitalInput(RobotMap.Climber.LEFT_TOP_LIMIT_SWITCH); 
        rightTopSwitch = new DigitalInput(RobotMap.Climber.RIGHT_TOP_LIMIT_SWITCH); 
        leftBotSwitch = new DigitalInput(RobotMap.Climber.LEFT_BOT_LIMIT_SWITCH);
        rightBotSwitch = new DigitalInput(RobotMap.Climber.RIGHT_BOT_LIMIT_SWITCH); 

        configClimberMotors();
    }

    public static Climber getInstance() {
        if (instance == null) {
          instance = new Climber();
        }
        return instance;
    }

    public void configClimberMotors() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = 1.5;  // An error of 0.5 rotations results in 1.2 volts output
        configs.Slot0.kD = 0.00048; // A change of 1 rotation per second results in 0.1 volts output

        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = right.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        /* Make sure we start at 0 */
        left.setPosition(0); 
        right.setPosition(0);
    }

    public Command releaseTop() {
        return runOnce(
            () -> {
                target(Constants.Climber.ClimberMax, Constants.Climber.ClimberMax);
            });
    }

    public Command retract() {
        return runOnce(
            () -> {
                target(0, 0);
            });
    }

    // Manually control the arms up - stop the motors if the climbers have already reached max height - check limit switches
    public Command manualUp(double leftSpeed, double rightSpeed) {
        return runOnce(
            () -> {
                if (notMax()) {
                    setMotorSpeeds(leftSpeed, rightSpeed);
                } else {
                    stop();
                }
            });
    }

    // Manually control the arms down - stop the motors if the climbers have already reached minimum height - check limit switches
    public Command manualDown(double leftSpeed, double rightSpeed) {
        return runOnce(
            () -> {
                if (notMin()) {
                    setMotorSpeeds(-leftSpeed, -rightSpeed);
                } else {
                    stop();
                }
            });
    }

    public void stop() {
        setMotorSpeeds(0, 0);
    }

    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
        left.set(leftSpeed); 
        right.set(rightSpeed);
    }

    // Return true if the climber has not reached its min - check limit switch too 
    public boolean notMin() {
        return (right.getRotorPosition().getValueAsDouble() > Constants.Climber.ClimberMin && 
        left.getRotorPosition().getValueAsDouble() > Constants.Climber.ClimberMin && 
        !limitSwitchTriggered()); 
    }

    // Return true if the climber has not reached its max 
    public boolean notMax() {
        return (right.getRotorPosition().getValueAsDouble() <= Constants.Climber.ClimberMax && 
        left.getRotorPosition().getValueAsDouble() <= Constants.Climber.ClimberMax && 
        !limitSwitchTriggered()); 
    }

    public boolean limitSwitchTriggered() { // return true if any of the limit switches have been triggered
        return leftBotSwitch.get() && rightBotSwitch.get() && leftTopSwitch.get() && rightTopSwitch.get(); 
    }

    // Set motors to travel a certain number of rotations 
    public void target(double leftGoal, double rightGoal) {
        right.setControl(positionOut.withPosition(leftGoal)); 
        left.setControl(positionOut.withPosition(rightGoal)); 
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Motor position", right.getRotorPosition().getValueAsDouble() / Constants.Climber.ClimberGearRatio);
    }
}
