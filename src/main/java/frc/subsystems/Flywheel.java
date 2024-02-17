package frc.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Flywheel extends SubsystemBase {
    // Intialize flywheel motors
    private TalonFX left;
    private TalonFX right;
    // Initialize velocity feedforward
    private VelocityVoltage m_voltageVelocity;

    private static Flywheel instance;

    public Flywheel() {
        left = new TalonFX(RobotMap.Flywheel.LEFT_FLYWHEEL, Constants.CANBusName);
        right = new TalonFX(RobotMap.Flywheel.RIGHT_FLYWHEEL, Constants.CANBusName);
        m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
        configFlywheel();
    }

    public static Flywheel getInstance() {
        if (instance == null) {
            return new Flywheel();
        }
        return instance;
    }

    public void configFlywheel() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kP = 0.13; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.11; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = left.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
        System.out.println("Could not apply configs, error code:] " + status.toString());
        }

        right.setControl(new Follower(left.getDeviceID(), false)); // Set right motor to follow left
    }

    public Command on() {
        return runOnce(
                () -> {
                    left.setControl(m_voltageVelocity.withVelocity(Constants.Flywheel.FLYWHEEL_SPEED));
                });
    }

    public Command off() {
        return runOnce(
                () -> {
                    left.setControl(m_voltageVelocity.withVelocity(0));
                });
    }
}
