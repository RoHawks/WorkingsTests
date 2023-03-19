package frc.robot.routines;

import java.util.List;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmMovementComponentStep extends AComponentStep 
{
    private double kDt = 0.02;  //time delta of main robot loop, seconds.
    private TrapezoidProfile.Constraints mConstraints;
    private TrapezoidProfile.State mGoal;
    private TrapezoidProfile.State mSetpoint;
    private SparkMaxPIDController mPIDController;
    private AbsoluteEncoder mEncoder;
    private double mPassPositiveBound;
    private double mPassNegativeBound;
    private double mInitialVelocity = 0;

    public ArmMovementComponentStep(String pName, List<StepPrequisite> pPrerequisites, TrapezoidProfile.Constraints pConstraints, double pFinalEndGoal,
        SparkMaxPIDController pPIDController, AbsoluteEncoder pEncoder, double pPassPositiveBound, double pPassNegativeBound)
        {
            super(pName, pPrerequisites);
            mGoal = new TrapezoidProfile.State(pFinalEndGoal, 0);
            mConstraints = pConstraints;
            mPIDController = pPIDController;
            mEncoder = pEncoder;
            mPassPositiveBound = pPassPositiveBound;
            mPassNegativeBound = pPassNegativeBound;
            mSetpoint = new State(mEncoder.getPosition(), 0);
        }

    public void SetInitialVelocity(double pInitialVelocity)
    {
        mInitialVelocity = pInitialVelocity;
    }

    @Override 
    public void EnterStep()
    {
        super.EnterStep();
        //System.out.println("Enter Step called for " + mName);
        mSetpoint = new State(mEncoder.getPosition(), mInitialVelocity);
    }

    @Override
    public void Run() {

    
          // Create a motion profile with the given maximum velocity and maximum
          // acceleration constraints for the next setpoint, the desired goal, and the
          // current setpoint.
          var profile = new TrapezoidProfile(mConstraints, mGoal, mSetpoint);
    
          // Retrieve the profiled setpoint for the next timestep. This setpoint moves
          // toward the goal while obeying the constraints.
          mSetpoint = profile.calculate(kDt);
          
          SmartDashboard.putNumber(mName + "_SmartMotionPosition", mSetpoint.position );
          SmartDashboard.putNumber(mName + "_SmartMotionVelocity", mSetpoint.velocity );
          // Send setpoint to offboard controller PID
          mPIDController.setReference(mSetpoint.position,ControlType.kPosition);
        
    }

    @Override
    public double DistanceFromCompletion() {
        return mEncoder.getPosition() - mGoal.position;
    }


    @Override
    public boolean IsComplete()
    {
        if(mHasBeenCompleted)
        {
            return true;
        }
        double distanceFromComplete = DistanceFromCompletion();
        mHasBeenCompleted = mHasBeenStarted && distanceFromComplete <= mPassPositiveBound && distanceFromComplete >= mPassNegativeBound;
        return mHasBeenCompleted;
    }

    @Override
    public void Reset()
    {
        super.Reset();
        mSetpoint = new State(mEncoder.getPosition(), 0);
    }
    
}
