package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class MotorPIDSlideSubsystem extends SubsystemBase {
  protected boolean isResetting = false;
  protected Telemetry telemetry;

  /**
   * Make motor run using open loop control
   *
   * @param percent The desired power output directly to motor. DON'T NORMALIZE with battery voltage
   */
  abstract void runOpenLoop(double percent);

  /** Gets the current position of motor in ticks. */
  abstract long getCurrentPosition();

  /** Gets the reset power of motor in [-1,1]. */
  abstract double getResetPower();

  /** Sets the encoder value of motor to 0. It does not guarantee current motor velocity is 0 */
  abstract void resetEncoder();

  /**
   * Construct a manual reset command, it will start to reset when scheduled, and stop when
   * canceled. It dose not provide verification on whether the slide structure is reset.
   *
   * @return The command constructed.
   */
  public Command manualResetCommand() {
    return new StartEndCommand(
        () -> {
          runOpenLoop(getResetPower());
          isResetting = true;
        },
        () -> {
          resetEncoder();
          isResetting = false;
        },
        this);
  }

  /**
   * Construct a auto reset command, it will start to reset when scheduled, and stop automatically
   * when it thinks the slide was in place.
   *
   * @return The command constructed.
   */
  public Command autoResetCommand() {
    return new CommandBase() {
      double minVal = Double.POSITIVE_INFINITY;
      double velo, lastvelo;
      double startpos, lastpos, lastTime = 0;
      final ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

      @Override
      public void initialize() {
        lastpos = getCurrentPosition();
        startpos = lastpos;
        isResetting = true;
        velo = 0;
        t.reset();
        runOpenLoop(getResetPower());
      }

      @Override
      public void execute() {
        lastvelo = velo;
        double nowpos = getCurrentPosition();
        double nowtime = t.time();
        velo = (nowpos - lastpos) / (nowtime - lastTime);
        lastTime = nowtime;
        if (minVal > nowpos) {
          minVal = nowpos;
        }
        lastpos = nowpos;
        telemetry.addData("AutoResetCommand.vel", velo);
        telemetry.addData("AutoResetCommand.dt", lastpos - nowtime);
        telemetry.addData("AutoResetCommand.minVal", minVal);
        telemetry.addData(
            "AutoResetCommand.AvgVel", ((getCurrentPosition() - startpos) / t.time()));
      }

      @Override
      public boolean isFinished() {
        if (Math.abs(velo) < Math.abs(((getCurrentPosition() - startpos) / t.time()) * 0.9)
            && lastvelo > velo) return true;
        return false;
      }

      @Override
      public void end(boolean interrupted) {
        runOpenLoop(0);
        //                if(!interrupted){
        resetEncoder();
        //                }
        isResetting = false;
      }
    };
  }
}
