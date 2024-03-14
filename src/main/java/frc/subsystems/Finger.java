package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Finger extends SubsystemBase{
    private CANSparkMax motor; 
    private SparkPIDController pidController;
    private double kP, kI, kD, kIzone, kFF, kMaxOutput, kMinOutput;
    private RelativeEncoder relativeEncoder;
    private static Finger instance; 

    public Finger() {
        motor = new CANSparkMax(RobotMap.Finger.FINGER_MOTOR, CANSparkMax.MotorType.kBrushless); // initialize motor
        configureMotor();
        relativeEncoder.setPosition(0);
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
        motor.setSmartCurrentLimit(30);

        relativeEncoder = motor.getEncoder(); // getEncoder​(SparkRelativeEncoder.Type encoderType, int countsPerRev) // absolute encoder
        pidController = motor.getPIDController();

        // pidController.setFeedbackDevice(m_encoder); // Absolute encoder 

        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIzone = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIzone);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    /**
     * @param position rotation number to run the finger
     * @return Command that runs finger a certain number of rotations. 
     */
    public Command run(double position) {
        return new InstantCommand(() -> pidController.setReference(position * Constants.Finger.FINGER_GEAR_RATIO, CANSparkMax.ControlType.kPosition));
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
        return run(Constants.Finger.DISTANCE); 
    }

    @Override
    public void periodic() {
        System.out.println(getPosition());
    }
}
