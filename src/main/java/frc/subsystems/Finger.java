package frc.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.RobotMap;

public class Finger {
    private CANSparkMax n_motor; 

    public Finger() {
        n_motor = new CANSparkMax(RobotMap.Finger.FINGER_MOTOR, CANSparkMax.MotorType.kBrushless); // initialize motor 
    }


}
