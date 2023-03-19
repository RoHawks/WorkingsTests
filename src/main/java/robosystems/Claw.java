package robosystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; //import static class representing the value of the DoubleSolenoid (sm Sam noted)
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * A class for our claw mechanism
 **/
public class Claw { //a bunch of variables/objects that are going to be used
  private long mLastTapTime;
  private int mTapCounter;
  private DoubleSolenoid mRightDoubleSolenoid;
  private DoubleSolenoid mLeftDoubleSolenoid;
  /**
 * Creates a new instance of Claw 
 * @param pRightDoubleSolenoid: the right DoubleSolenoid object
 * @param pLeftDoubleSolenoid: the left DoubleSolenoid
 **/
  public Claw (DoubleSolenoid pRightDoubleSolenoid, DoubleSolenoid pLeftDoubleSolenoid) {
    mLastTapTime = 0;
    mTapCounter = 0;
    mRightDoubleSolenoid = pRightDoubleSolenoid;
    mLeftDoubleSolenoid = pLeftDoubleSolenoid;
  }
  /**
 * Opens the claw if it is not already open.
 **/ 
  public void OpenClaw() {
    mLeftDoubleSolenoid.set(kForward);
    mRightDoubleSolenoid.set(kForward);
  }
  /**
 * Closes the claw if it is not already closed.
 **/
  public void CollapseClaw() {
    mLeftDoubleSolenoid.set(kReverse);
    mRightDoubleSolenoid.set(kReverse);
  }
  /**
 * Switches left and right solenoid being pushed forward ever 100 milliseconds
 **/
  public void Tap() {
    if ((System.currentTimeMillis() - mLastTapTime) >= 100) { //Only tap if 100 ms has passed since last tap 
      if (mTapCounter%2 == 0) { 
        mRightDoubleSolenoid.set(kForward);
        mLeftDoubleSolenoid.set(kReverse);
      }
      else {
        mRightDoubleSolenoid.set(kReverse);
        mLeftDoubleSolenoid.set(kForward);
      }
      mLastTapTime = System.currentTimeMillis(); //Sets the last tap time to current time
      mTapCounter++;
    }
  }
}
