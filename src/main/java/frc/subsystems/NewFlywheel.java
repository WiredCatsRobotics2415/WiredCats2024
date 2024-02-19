package frc.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class NewFlywheel extends SubsystemBase {
    private enum TestType {
        LOCK,
        RATIO
    }

    private double leftSpeedRatio = 1.0d;
    private double leftSetRPM = 0d;

    private TestType currentMode = TestType.LOCK;

    // Intialize flywheel motors
    private TalonFX left;
    private TalonFX right;
    // Initialize velocity feedforward
    private VelocityVoltage m_voltageVelocity;
    private final SendableChooser<TestType> testChooser = new SendableChooser<TestType>();

    private static NewFlywheel instance;

    private MechanismLigament2d leftGoal;
    private MechanismLigament2d rightGoal;

    private boolean shouldSpinUp = false;

    private NewFlywheel() {
        left = new TalonFX(RobotMap.Flywheel.LEFT_FLYWHEEL);
        right = new TalonFX(RobotMap.Flywheel.RIGHT_FLYWHEEL);
        m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
        configFlywheel();
        configSmartDashboard();
    }

    /**
     * Changes the test mode, between Lock and Ratio, from the SendableChooser.
     * Intended to be called in teleopInit.
     */
    public void teleopInit() {
        currentMode = testChooser.getSelected();

        if (currentMode.equals(TestType.LOCK)) {
            // left.setControl(new StaticBrake());
        }
    }

    public static NewFlywheel getInstance() {
        if (instance == null) {
            return new NewFlywheel();
        }
        return instance;
    }

    private void configSmartDashboard() {
        testChooser.setDefaultOption(TestType.LOCK.toString(), TestType.LOCK);
        testChooser.addOption(TestType.RATIO.toString(), TestType.RATIO);
        SmartDashboard.setDefaultNumber("Left:Right Ratio", leftSpeedRatio);
        SmartDashboard.setDefaultNumber("Set Speed (Left Motor - RPM)", leftSetRPM);
        SmartDashboard.setDefaultNumber("Current Speed (Left)", left.getRotorVelocity().getValueAsDouble());
        SmartDashboard.setDefaultNumber("Current Speed (Right)", right.getRotorVelocity().getValueAsDouble());

        Mechanism2d flywheelMech2d = new Mechanism2d(3, 3);

        MechanismRoot2d leftGoalRoot = flywheelMech2d.getRoot("leftGoal", 1.4, 0.25);
        leftGoal = leftGoalRoot.append(new MechanismLigament2d("leftGoal", 0.1, 0));
        MechanismRoot2d rightGoalRoot = flywheelMech2d.getRoot("rightGoal", 1.6, 0.25);
        rightGoal = rightGoalRoot.append(new MechanismLigament2d("rightGoal", 0.1, 0));

        SmartDashboard.putData("Flywheel Mechanism", flywheelMech2d);
    }

    public void configFlywheel() {
        right.setInverted(true);
        left.setInverted(false);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kP = Constants.Flywheel.FlywheelPIDS.kP; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = Constants.Flywheel.FlywheelPIDS.kI; // An error of 1 rotation per second increases output by
                                                               // 0.5V every second
        configs.Slot0.kD = Constants.Flywheel.FlywheelPIDS.kD; // A change of 1 rotation per second squared results in
                                                               // 0.01 volts output
        configs.Slot0.kV = Constants.Flywheel.FlywheelPIDS.kV; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps
                                                               // per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = left.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code:] " + status.toString());
        }
    }

    /**
     * @return Command that spins up each motor to the specified RPM.
     */
    public Command on(double leftSpeed, double rightSpeed) {
        return runOnce(
                () -> {
                    left.setControl(m_voltageVelocity.withVelocity(leftSpeed));
                    right.setControl(m_voltageVelocity.withVelocity(rightSpeed));
                });
    }

    /**
     * @return Command that sets both motor's RPM to 0.
     *         Note that the flywheel is in coast.
     */
    public Command off() {
        return runOnce(
                () -> {
                    left.setControl(m_voltageVelocity.withVelocity(0));
                    right.setControl(m_voltageVelocity.withVelocity(0));
                });
    }

    /**
     * @return Command that toggles whether or not the flywheel should spin up to
     *         its goal RPM.
     *         If already spinned up, spin down to 0.
     *         Otherwise, spin up to the goal.
     */
    public Command toggleSpinedUp() {
        return runOnce(() -> {
            System.out.println("Flywheel toggle " + (!shouldSpinUp ? "on" : "off"));
            shouldSpinUp = !shouldSpinUp;
        });
    }

    /**
     * True if the current speed of the left shooter motor is within + or -
     * GOAL_TOLERANCE_RPM
     */
    public boolean withinGoal() {
        double currentValue = Constants.Flywheel.falconToRPM(left.getRotorVelocity().getValue());
        if (shouldSpinUp) {
            double goalValue = Constants.Flywheel.rpmToFalcon(leftSetRPM);
            return currentValue < (goalValue + Constants.Flywheel.GOAL_TOLERANCE_RPM) ||
                    currentValue > (goalValue - Constants.Flywheel.GOAL_TOLERANCE_RPM);
        } else {
            return currentValue < Constants.Flywheel.GOAL_TOLERANCE_RPM ||
                    currentValue > -Constants.Flywheel.GOAL_TOLERANCE_RPM;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(testChooser);
        leftSpeedRatio = SmartDashboard.getNumber("Left:Right Ratio", leftSpeedRatio);
        leftSetRPM = SmartDashboard.getNumber("Set Speed (Left Motor - RPM)", leftSetRPM);

        // Display current speed of both motors
        SmartDashboard.putNumber("Current Speed (Left)", left.getRotorVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Current Speed (Right)", right.getRotorVelocity().getValueAsDouble());

        // Set the speed of the motors
        if (shouldSpinUp) {
            on(leftSetRPM, leftSetRPM / leftSpeedRatio);
        } else {
            off();
        }

        rightGoal.setColor(getColorForRPM(leftSetRPM / leftSpeedRatio));
        leftGoal.setColor(getColorForRPM(leftSetRPM));
    }

    private Color8Bit getColorForRPM(double rpm) {
        return new Color8Bit((int) ((rpm / 6380) * 255), 0, 0);
    }
}