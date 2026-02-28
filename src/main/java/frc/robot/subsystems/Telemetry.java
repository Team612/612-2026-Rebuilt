package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;


import javax.naming.directory.InvalidSearchControlsException;
import java.util.Map;
import java.util.Set;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.PoseEstimator;
// import frc.robot.subsystems.Vision;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.measure.Velocity;

public class Telemetry extends SubsystemBase {
    Intake m_intake;
    Shooter m_shooter;
    Vision m_vision;

    // public void Telemetry() {
    //     pass;
    // }
    public Telemetry(Shooter shooter, Intake intake, Vision vision) {
        m_shooter = shooter;
        m_vision = vision;
        m_intake = intake;
    }

    public void updateData(){
        // Driver Tab:
        // APRILTag Shooter Camera (Green/Red)
        // APRIL Tag Drivetrain Camera (Green/Red)

        SmartDashboard.putBoolean("Drivetrain Has Tag", m_vision.hasTag());
        SmartDashboard.putBoolean("shooter Has Tag", m_shooter.hasTag());

        // Driver Camera Stream
        // If Manual Mode On (Green/Red)
        SmartDashboard.putBoolean("Manual Mode", RobotContainer.manualMode);
        // SmartDashboard.putBoolean("Manual Mode", m_vision.hasTag());

        // Shooter Velocity 
        SmartDashboard.putNumber("Shooter Velocity", m_shooter.getShooterVelocity());
        SmartDashboard.putNumber("Turret Position", m_shooter.getTurretAngleDriver());
        SmartDashboard.putBoolean("Ready to Fire!!!!", true);

        SmartDashboard.putNumber("Ramp Up Time", 50); 
        Constants.TransferConstants.rampUpTime = (int) SmartDashboard.getNumber("Constant of Time", 50);
        SmartDashboard.putNumber("Shoot Time",202); 
        Constants.TransferConstants.shootTime = (int) SmartDashboard.getNumber("Shoot Time", 20);
        SmartDashboard.putNumber("Recovery Time", 10); 
        Constants.TransferConstants.recoveryTime = (int) SmartDashboard.getNumber("Recovery Time", 10);

        // SmartDashboard.putBoolean("Distance To Tag", m_shooter.dist());
        // SmartDashboard.putBoolean("Manual Mode", m_shooter.has);
        // Turret Position (Degrees)
        // Locked on (READY TO FIRE)
        // Distance To Tag (number)
        // Testing Tab:
        // Set Motor Speeds

        // SHOOTER

        
        // Output Values
        SmartDashboard.putNumber("e", 2);
        // Color exampleColor = new Color(68, 238, 255);
        // SmartDashboard.putString("Example Color", exampleColor.toHexString());

        // Input values
    }
}