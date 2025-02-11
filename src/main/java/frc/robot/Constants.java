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
    
    

    public class HookFlapConstants{
        public static final double flapHookOpen = 0;
        public static final double hookPrepare = 4;
        public static final double hookLatch = 10;
        public static final double flapCollect = 5;
    }

    public class ClimberConstants{
        public static final double climberHome = 0;
        public static final double climberClimb = 10;
    }

    public class ElevatorConstants{
        public static final double elevatorHome = 0;
        public static final double elevatorCollect = 5;
        public static final double elevatorCoralL1 = 3;
        public static final double elevatorCoralL2 = 5;
        public static final double elevatorCoralL3 = 7;
        public static final double elevatorCoralL4 = 9;
        public static final double elevatorProcess = 1;
        public static final double elevatorAlgaeL2 = 5;
        public static final double elevatorAlgaeL3 = 7;
    }

    public class ManipulatorConstants{
        public static final double manipulatorHome = 0;
        public static final double manipulatorCollect = 4;
        public static final double manipulatorTravel = 5;
        public static final double manipulatorCoralL1 = 2;
        public static final double manipulatorCoralL2 = 3;
        public static final double manipulatorCoralL3 = 4;
        public static final double manipulatorCoralL4 = 5;
        public static final double manipulatorAlgeaCollect = 8;
        public static final double manipulatorAlgaeProcess = 4;
    }
    
}
