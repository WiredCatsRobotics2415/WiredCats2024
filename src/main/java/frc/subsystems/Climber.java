package frc.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

    private Climber() {
        //From the back of the robot
        right = new TalonFX(RobotMap.Climber.CLIMBER_MASTER, RobotMap.CANBUS_NAME); // Initialize right motor
        left = new TalonFX(RobotMap.Climber.CLIMBER_FOLLOWER, RobotMap.CANBUS_NAME); // Initialize left motor
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
        configs.Voltage.PeakForwardVoltage = 6;
        configs.Voltage.PeakReverseVoltage = -6;

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

        System.out.println("Right position=" + right.getRotorPosition());
        System.out.println("Left position= " + left.getRotorPosition());
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

    public Command runUntil() {
        return runOnce(
            () -> {
                if (notMax()) {
                    left.set(0.25);
                    right.set(0.25);
                } else {
                    left.set(0);
                    right.set(0);
                }
            });
    }

    public Command stop() {
        return runOnce(
            () -> {
                left.set(0);
                right.set(0);
            });
    }

    public Command release() {
        return new ConditionalCommand(
            run(
                () -> {
                    target(left.getRotorPosition().getValueAsDouble() + 6, right.getRotorPosition().getValueAsDouble() + 6); 
                }
            ), 
            run(
                () -> {
                    target(left.getRotorPosition().getValueAsDouble(), right.getRotorPosition().getValueAsDouble()); 
                }
            ), () -> notMax()); 
    }

    // Return true if the climber has not reached its max 
    public boolean notMax() {
        return (right.getRotorPosition().getValueAsDouble() <= Constants.Climber.ClimberMax && left.getRotorPosition().getValueAsDouble() <= Constants.Climber.ClimberMax); 
    }

    // Set motors to travel a certain number of rotations 
    public void target(double leftGoal, double rightGoal) {
        right.setControl(positionOut.withPosition(leftGoal)); 
        left.setControl(positionOut.withPosition(rightGoal)); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor position", right.getRotorPosition().getValueAsDouble() / Constants.Climber.ClimberGearRatio);
    }
}
