package frc.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
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

    private void configClimberMotors() {
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
    
    /**
     * @return A command that releases both climbers (ie. moves them to thier resting position,
     * which is up.)
     */
    public Command releaseTop() {
        return runOnce(
            () -> {
                target(Constants.Climber.ClimberMax, Constants.Climber.ClimberMax);
            });
    }

    /**
     * @return A command that retracts both climbers (ie. moves them to thier climbed position,
     * which is all the way down.)
     */
    public Command retract() {
        return runOnce(
            () -> {
                target(0, 0);
            });
    }

    /**
     * @return A command that sets the speed of individual climbers, to move them UP.
     * Automatically stops the motors if the climbers have already reached max height,
     * or the limit switches are pressed.
     */
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

    /**
     * @return A command that sets the speed of individual climbers, to move them DOWN.
     * Automatically stops the motors if the climbers have already reached minimum height,
     * or the limit switches are pressed.
     */
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

    /**
     * Sets both motor's speeds to 0.
     */
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    /**
     * Sets each motor's SPEED (as opposed to position).
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
        left.set(leftSpeed); 
        right.set(rightSpeed);
    }

    /**
     * @return true if the climber has not reached its minimum height, or true if the limit switches
     * are not triggered. false if either of those conditions are true.
     */
    public boolean notMin() {
        return (right.getRotorPosition().getValueAsDouble() > Constants.Climber.ClimberMin && 
        left.getRotorPosition().getValueAsDouble() > Constants.Climber.ClimberMin && 
        !limitSwitchTriggered()); 
    }

    /**
     * @return true if the climber has not reached its maximum height, or true if the limit switches
     * are not triggered. false if either of those conditions are true.
     */
    public boolean notMax() {
        return (right.getRotorPosition().getValueAsDouble() <= Constants.Climber.ClimberMax && 
        left.getRotorPosition().getValueAsDouble() <= Constants.Climber.ClimberMax && 
        !limitSwitchTriggered()); 
    }

    /**
     * @return true if ANY limit switches are triggered.
     */
    public boolean limitSwitchTriggered() {
        return leftBotSwitch.get() && rightBotSwitch.get() && leftTopSwitch.get() && rightTopSwitch.get(); 
    }

    /**
     * Sets a POSITION (as opposed to speed) goal for each motor.
     */
    public void target(double leftGoal, double rightGoal) {
        right.setControl(positionOut.withPosition(leftGoal)); 
        left.setControl(positionOut.withPosition(rightGoal)); 
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Motor position", right.getRotorPosition().getValueAsDouble() / Constants.Climber.ClimberGearRatio);
    }
}
