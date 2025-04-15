package frc.robot.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveTest extends Command {

    CommandSwerveDrivetrain m_Drivetrain;
    SwerveRequest.FieldCentric drive;
    SwerveRequest.ApplyRobotSpeeds robotSpeeds;
    Pose2d currentPose;
    Pose2d desiredPose;
    Translation2d calculatedTranslation;
    double calcuatedRotation;
    double speed;
    double minSpeed = -1000;
    double maxSpeed = 1000;
 

    public DriveTest(CommandSwerveDrivetrain m_Drivetrain, SwerveRequest.FieldCentric drive, SwerveRequest.ApplyRobotSpeeds robotSpeeds, double speed) {
        this.m_Drivetrain = m_Drivetrain;
        this.drive = drive;
        this.robotSpeeds = robotSpeeds;
        this.speed = speed;
        addRequirements(m_Drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      m_Drivetrain.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(5, 0, 0)));
    }

    @Override
    public void end(boolean interrupted) {
      Commands.runOnce(() ->{
      
        m_Drivetrain.applyRequest(() ->
                  drive.withVelocityX(0) // Drive forward with negative Y (forward)
                      .withVelocityY(0) // Drive left with negative X (left)
                      .withRotationalRate(0) // Drive counterclockwise with negative X (left)
              );}, m_Drivetrain
        );
    }

    @Override
    public boolean isFinished() {
      return m_Drivetrain.isAtDesired();

    }
    

    @Override
    public boolean runsWhenDisabled() {  
      return false;
    }
}