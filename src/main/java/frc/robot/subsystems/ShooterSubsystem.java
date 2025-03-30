package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor;

    // Constructor
    public ShooterSubsystem() {
        shooterMotor = new SparkMax(ShooterConstants.shooterMoterID, MotorType.kBrushless);
    }


    public void stopMotor() {
        shooterMotor.set(0.0);
    }

    // Run the motor at a specified speed
    public void runMotor(double speed) {
        shooterMotor.set(speed);
    }
}
