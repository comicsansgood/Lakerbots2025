package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  public int[] validIds = {12, 21};
  public LimelightHelpers.PoseEstimate mt2;





  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

Double [] values={0.0,0.0,0.0};

  public LimelightSubsystem() {
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIds);

  }

public Pose2d getEstimatedPose(){
 //if there is not a tag or the robot is rotating above 720 deg/s, return null, else return the estimated pose
      if(!(mt2.tagCount == 0 ||   (180/Math.PI)*RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond >= 720)){
          return mt2.pose;
      }
      else{
          return null;
      }
    }

    public double getTimeStamp(){
        return mt2.timestampSeconds;
    }
  
  
  
  
  public Command exampleMethodCommand() {

    return runOnce(
        () -> {
        });
  }

  public boolean exampleCondition() {
    return false;
  }

  public Double[] getValues(){
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    values[0] = x;
    values[1] = y;
    values[2] = area;
    return values;
  }




  @Override
  public void periodic() {
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);


    LimelightHelpers.SetRobotOrientation("limelight", RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");


  }

  @Override
  public void simulationPeriodic() {
  }
}
