package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final double wheelBase = 22.75;
    public static final double trackWidth = 20.75;
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    
    public class ClimberConstants{
        public static final double climberUp = 0;
        public static final double climberDown = 1;
    }
    public class ElevatorConstants{
        public static final double elevatorDown = 0;
        public static final double elevatorUp = 0;
        public static final double elevatorStop = 0;
    }
}
