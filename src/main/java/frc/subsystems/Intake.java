package frc.subsystems;
import com.revrobotics.CANSparkMax;
import frc.robot.RobotMap;

public class Intake {
    private CANSparkMax motor;
    private static Intake instance;
    private boolean state;

    public Intake() {
        motor = new CANSparkMax(RobotMap.Intake.INTAKE_MOTOR, CANSparkMax.MotorType.kBrushless);
        state = false;
    }

    public static Intake getInstance() {
        if (instance == null) {
          instance = new Intake();
        }
        return instance;
      }

      public void toggle() {
        if (state == true) {
            motor.set(0);
            state = false;
        } else {
            motor.set(1);
            state = true;
        }
      }
}
