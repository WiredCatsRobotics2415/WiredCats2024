package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Finger extends SubsystemBase{
    private CANSparkMax n_motor; 
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private RelativeEncoder e_encoder; 
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private static Finger instance; 

    public Finger() {
        n_motor = new CANSparkMax(RobotMap.Finger.FINGER_MOTOR, CANSparkMax.MotorType.kBrushless); // initialize motor 
        // n_motor.setInverted(true);
        n_motor.setSmartCurrentLimit(30);
        configure();
    }

    public static Finger getInstance() {
        if (instance == null) {
            instance = new Finger();
        }
        return instance;
    }

    public void configure() {
        n_motor.restoreFactoryDefaults();
        n_motor.getEncoder().setPosition(0); 
        // m_pidController.setFeedbackDevice(m_encoder); // absolute encoder 
        m_encoder = n_motor.getEncoder(); // getEncoder​(SparkRelativeEncoder.Type encoderType, int countsPerRev) // absolute encoder

        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public Command run(double position) {
        return new InstantCommand(() -> m_pidController.setReference(position * Constants.Finger.FINGER_GEAR_RATIO, CANSparkMax.ControlType.kPosition));
    }

    public double getPosition() {
        return n_motor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {}

}
