package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class SuperStructure extends LinearOpMode {
    public Servo wristTurnServo;
    public Servo intakeClawServo;
    public Servo wristServo;
    public Servo liftClawServo;
    public Servo slideArmServo;
    public Servo liftArmServo;

    public DcMotorEx leftBack;
    public DcMotorEx rightBack;
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;

    public DcMotorEx slideMotor;
    public DcMotorEx liftMotorUp;
    public DcMotorEx liftMotorDown;

    @Override
    public void runOpMode() {
        wristTurnServo = hardwareMap.get(Servo.class, "wristTurnServo");
        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        liftClawServo = hardwareMap.get(Servo.class, "liftClawServo");
        slideArmServo = hardwareMap.get(Servo.class, "slideArmServo");
        liftArmServo = hardwareMap.get(Servo.class, "liftArmServo");

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        liftMotorUp = hardwareMap.get(DcMotorEx.class, "liftUp");
        liftMotorDown = hardwareMap.get(DcMotorEx.class, "liftDown");


    }
}
