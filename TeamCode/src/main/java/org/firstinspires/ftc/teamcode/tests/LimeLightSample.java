package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "LimeLightSample")
public class LimeLightSample extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

//        limelight.pipelineSwitch(0);

//        limelight.resetDeviceConfigurationForOpMode();
//
        limelight.setPollRateHz(50);


        /*
         * Starts polling for data.
         */
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            limelight.updatePythonInputs(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                telemetry.addData("time",getRuntime());
                telemetry.addData("valid",result.isValid());

                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("ta", result.getTa());
                telemetry.addData("angle",result.getPythonOutput()[3]);
            }
            telemetry.update();
        }
    }
}