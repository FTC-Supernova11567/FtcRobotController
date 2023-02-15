package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    private boolean is_auto;

    private DcMotor leftTop;
    private DcMotor rightTop;
    private DcMotor leftBottom;
    private DcMotor rightBottom;
    private DcMotor right_arm_motor;
    private DcMotor left_arm_motor;

    public Servo rightServo;
    public Servo leftServo;

    public Gripper gripper;
    public Drive drive;
    public Arm arm;
    public RevIMU imu;

    public Robot(HardwareMap map, boolean is_auto) {
        this.is_auto = is_auto;
        this.leftTop = map.get(DcMotor.class, "leftTop");
        this.rightTop = map.get(DcMotor.class, "rightTop");
        this.leftBottom = map.get(DcMotor.class, "leftBottom");
       this. rightBottom = map.get(DcMotor.class, "rightBottom");
        this.leftServo = map.get(Servo.class, "leftServo");
        this.rightServo = map.get(Servo.class, "rightServo");
        this.left_arm_motor = map.get(DcMotor.class, "LeftArmMotor");
        this.right_arm_motor = map.get(DcMotor.class, "RightArmMotor");

        this.imu = new RevIMU(map);
        this.imu.init();

        this.drive = new Drive(leftTop, rightTop, leftBottom, rightBottom);
        this.gripper = new Gripper(rightServo, leftServo);
        this.arm = new Arm(left_arm_motor, right_arm_motor);

        this.right_arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftTop.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        this.right_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.left_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
