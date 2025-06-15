package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "变速器测试")
@Config
public class derailleurTest extends LinearOpMode {
    public static double motorPower = 0;
    public static double servoPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo shifter = hardwareMap.get(Servo.class, "shifter");
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "motor2");
        waitForStart();
        while(opModeIsActive()){
            shifter.setPosition(servoPos);
            motor1.setPower(motorPower);
            motor2.setPower(motorPower);
        }
    }
}
