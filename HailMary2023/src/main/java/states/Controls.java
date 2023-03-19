package states;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface Controls 
{
    public boolean GetDeployIntake();
    public boolean GetRetractIntake();

    public boolean GetGrabIntakenPiece();
    public boolean GetDropHeldPiece();

    public boolean GetPrepareForHighCone();
    public boolean GetPrepareForMiddleCone();
    public boolean GetPrepareForHighCube();
    public boolean GetPrepareForMiddleCube();
    public boolean GetPrepareForLowPiece();

    public boolean GetGoToDesiredScoringArmPosition();
    // public boolean DropHeldPiece();

    public boolean GetGoToIntaking();


    public double GetSwerveXComponent();

    public double GetSwerveYComponent();
    public double GetSwerveLinearSpeed();
    public double GetSwerveRotationalSpeed();

    public boolean GetSwerveModeSwitch();

    public enum SwerveNudgingDirection
    {
        NONE,
        NORTH,
        SOUTH,
        EAST,
        WEST
    }

    public SwerveNudgingDirection GetSwerveNudgingDirection();
    public enum SwerveGoalTracking
    {
        NONE,
        SOUTH_EAST,
        NORTH_EAST,
        NORTH_WEST,
        SOUTH_WEST
    }

    public SwerveGoalTracking GetSwerveGoalTracking();


}