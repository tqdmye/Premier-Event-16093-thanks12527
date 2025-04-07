package org.firstinspires.ftc.teamcode.lib.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
// @Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);

    if (DriveConstants.isSquid) {
      drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.SLOW);
    } else {
      drive.setCurrentTrajectoryMode(SampleMecanumDrive.TrajectoryMode.MEDIUM);
    }

    drive.setPoseEstimate(new Pose2d());

    waitForStart();

    if (isStopRequested()) return;

    Trajectory traj =
        drive.trajectoryBuilder(new Pose2d()).splineTo(new Vector2d(30, 30), 0).build();

    drive.followTrajectory(traj);

    sleep(2000);

    drive.followTrajectory(
        drive
            .trajectoryBuilder(traj.end(), true)
            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
            .build());
  }
}
