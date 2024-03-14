package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Finger extends SubsystemBase{
    private CANSparkMax motor; 
    private PIDController pidController;
    private RelativeEncoder relativeEncoder;
    private static Finger instance; 

    public Finger() {
        motor = new CANSparkMax(RobotMap.Finger.FINGER_MOTOR, CANSparkMax.MotorType.kBrushless); // initialize motor
        configureMotor();
        relativeEncoder.setPosition(0);
        pidController = new PIDController(2, 0, 0.1);
    }

    public static Finger getInstance() {
        if (instance == null) {
            instance = new Finger();
        }
        return instance;
    }

    public void configureMotor() {
        motor.restoreFactoryDefaults();

        // motor.setInverted(true);
        //motor.setSmartCurrentLimit(30);

        relativeEncoder = motor.getEncoder(); // getEncoder​(SparkRelativeEncoder.Type encoderType, int countsPerRev) // absolute encoder

        motor.setOpenLoopRampRate(0.1);
    }

    /**
     * @return current position of the finger. 
     */
    public double getPosition() {
        return relativeEncoder.getPosition();
    }

    /**
     * @return Command that fires note by running the finger 1 full rotation.
     */ 
    public Command fire() {
        return new InstantCommand(() -> {
            pidController.setSetpoint(getPosition() + (Constants.Finger.DISTANCE * Constants.Finger.FINGER_GEAR_RATIO));
            //pidController.setReference((Constants.Finger.DISTANCE + getPosition()) * Constants.Finger.FINGER_GEAR_RATIO, CANSparkMax.ControlType.kPosition);
            System.out.println("finger");
        });
        //return run(Constants.Finger.DISTANCE); 
    }

    @Override
    public void periodic() {
        double volt = pidController.calculate(getPosition());
        System.out.println(volt);
        motor.setVoltage(MathUtil.clamp(volt, -12, 12));
    }
}
