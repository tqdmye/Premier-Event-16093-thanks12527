package org.firstinspires.ftc.teamcode.lib.roadrunner.drive;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.lib.Units;
import org.firstinspires.ftc.teamcode.lib.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;

@Config
@Deprecated
public class GoBildaPinPointLocalizer extends TwoTrackingWheelLocalizer {
  private final GoBildaPinpointDriver odometry;

  public GoBildaPinPointLocalizer(final HardwareMap hardwareMap) {
    super(Arrays.asList(DriveConstants.xPose, DriveConstants.yPose));
    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "od");
    odometry.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.REVERSED,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);
    odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    odometry.setOffsets(0, 0);
    odometry.resetPosAndIMU();
  }

  @Override
  public double getHeading() {
    odometry.update();
    return odometry.getHeading();
  }

  @NonNull
  @Override
  public List<Double> getWheelPositions() {
    odometry.update();
    return Arrays.asList(
        Units.mmToInches(odometry.getPosX()), Units.mmToInches(odometry.getPosY()));
  }

  @NonNull
  @Override
  public List<Double> getWheelVelocities() {
    odometry.update();
    return Arrays.asList(
        Units.mmToInches(odometry.getVelX()), Units.mmToInches(odometry.getVelY()));
  }

  @NonNull
  @Override
  public Double getHeadingVelocity() {
    odometry.update();
    return odometry.getHeadingVelocity();
  }
}
