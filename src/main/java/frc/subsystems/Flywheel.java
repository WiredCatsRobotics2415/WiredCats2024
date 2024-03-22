package frc.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commands.ShootingPresets;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.sim.PhysicsSim;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;

public class Flywheel extends SubsystemBase {
    private double rightSetRPM = 0d;
    private double leftSetRPM = 0d;

    private TalonFX left;
    private TalonFX right;
    private VelocityVoltage voltageVelocity;

    private static Flywheel instance;

    private MechanismLigament2d leftGoal;
    private MechanismLigament2d rightGoal;

    private boolean isOn = false;

    private Flywheel() {
        left = new TalonFX(RobotMap.Flywheel.LEFT_FLYWHEEL);
        right = new TalonFX(RobotMap.Flywheel.RIGHT_FLYWHEEL);
        BaseStatusSignal.setUpdateFrequencyForAll(50, 
            left.getRotorVelocity(), right.getRotorVelocity(),
            left.getDeviceTemp(), right.getDeviceTemp(),
            left.getStatorCurrent(), right.getStatorCurrent());
        voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
        configMotors();
        configSmartDashboard();
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(left, 0.001);
            PhysicsSim.getInstance().addTalonFX(right, 0.001);
        }

        SmartDashboard.setDefaultNumber("Set Speed (Right Motor - RPM)", 6000);
        SmartDashboard.setDefaultNumber("Set Speed (Left Motor - RPM)", 8000);
    }

    public static Flywheel getInstance() {
        if (instance == null) {
            instance = new Flywheel();
        }
        return instance;
    }

    private void configSmartDashboard() {
        Mechanism2d flywheelMech2d = new Mechanism2d(3, 3);

        MechanismRoot2d leftGoalRoot = flywheelMech2d.getRoot("leftGoal", 1.4, 0.25);
        leftGoal = leftGoalRoot.append(new MechanismLigament2d("leftGoal", 0.1, 0));
        MechanismRoot2d rightGoalRoot = flywheelMech2d.getRoot("rightGoal", 1.6, 0.25);
        rightGoal = rightGoalRoot.append(new MechanismLigament2d("rightGoal", 0.1, 0));

        Shuffleboard.getTab("Mechanism2d").add("Flywheel Mechanism", flywheelMech2d);
    }

    private void configMotors() {
        TalonFXConfigurator rightCfg = right.getConfigurator();
        rightCfg.apply(Constants.Flywheel.RIGHT_PID);
        rightCfg.apply(Constants.Flywheel.COAST_CONFIG);
        rightCfg.apply(Constants.Flywheel.CURRENT_LIMITS);
        right.setInverted(true);

        TalonFXConfigurator leftCfg = left.getConfigurator();
        leftCfg.apply(Constants.Flywheel.LEFT_PID);
        leftCfg.apply(Constants.Flywheel.COAST_CONFIG);
        leftCfg.apply(Constants.Flywheel.CURRENT_LIMITS);
        left.setInverted(false);
    }

    /**
     * @return Command that spins up each motor to the specified RPM.
     */
    public Command on(double leftSpeed, double rightSpeed) {
        return runOnce(
                () -> {
                    isOn = true;
                    Logger.log(this, LogLevel.INFO, "Initial Flywheel On", leftSpeed, rightSpeed);
                    left.setControl(voltageVelocity.withVelocity(Constants.Flywheel.rpmToRPS(leftSpeed)));
                    right.setControl(voltageVelocity.withVelocity(Constants.Flywheel.rpmToRPS(rightSpeed)));
                });
    }

    /**
     * @return Command that calls on with speeds set from SmartDashboard.
     */
    public Command onFromSmartDashboard() {
        return runOnce(
                () -> {
                    isOn = true;
                    Logger.log(this, LogLevel.INFO, "Flywheel on from smart dashboard");
                    rightSetRPM = SmartDashboard.getNumber("Set Speed (Right Motor - RPM)", rightSetRPM);
                    leftSetRPM = SmartDashboard.getNumber("Set Speed (Left Motor - RPM)", leftSetRPM);
                    on(leftSetRPM, rightSetRPM).schedule();
                });
    }

    /**
     * @return Command that sets both motor's RPM to 0.
     *         Note that the flywheels is in coast.
     */
    public Command off() {
        return runOnce(
                () -> {
                    isOn = false;
                    Logger.log(this, LogLevel.INFO, "Flywheel off");
                    left.setControl(voltageVelocity.withVelocity(0));
                    right.setControl(voltageVelocity.withVelocity(0));
                });
    }

    /**
     * True if the current speed of the left shooter motor is within + or -
     * GOAL_TOLERANCE_RPM
     */
    public boolean withinSetGoal() {
        double currentValue = Constants.Flywheel.rpsToRPM(right.getRotorVelocity().getValue());
        if (isOn) {
            System.out.println(currentValue);
            System.out.println( currentValue < (ShootingPresets.Settings.subwoofer.right_flywheel + Constants.Flywheel.GOAL_TOLERANCE_RPM) &&
                    currentValue > (ShootingPresets.Settings.subwoofer.right_flywheel - Constants.Flywheel.GOAL_TOLERANCE_RPM));
            return currentValue < (ShootingPresets.Settings.subwoofer.right_flywheel + Constants.Flywheel.GOAL_TOLERANCE_RPM) &&
                    currentValue > (ShootingPresets.Settings.subwoofer.right_flywheel - Constants.Flywheel.GOAL_TOLERANCE_RPM);
        }
        return false;
    }

    @Override
    public void periodic() {
        // Display current speed of both motors
        double leftSpeedRaw = left.getRotorVelocity().getValueAsDouble();
        double rightSpeedRaw = right.getRotorVelocity().getValueAsDouble();

        SmartDashboard.putNumber("Current Wheel Speed (Left - RPM)", Constants.Flywheel.rpsToRPM(leftSpeedRaw));
        SmartDashboard.putNumber("Current Wheel Speed (Right - RPM)", Constants.Flywheel.rpsToRPM(rightSpeedRaw));

        rightGoal.setColor(getColorForRPM(Constants.Flywheel.rpsToRPM(rightSpeedRaw)));
        leftGoal.setColor(getColorForRPM(Constants.Flywheel.rpsToRPM(leftSpeedRaw)));

        SmartDashboard.putBoolean("Shooter isOn", isOn);
        SmartDashboard.putBoolean("Within goal", withinSetGoal());
    }

    private Color8Bit getColorForRPM(double rpm) {
        return new Color8Bit((int) ((rpm / 6380) * 255), 0, 0);
    }
}