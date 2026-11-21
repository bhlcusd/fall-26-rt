package frc.robot.subsystems.dualArms;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.devices.motor.MotorConfig.MeasurementUnit;

public class DualArmConstants {

    // Motor config
    public static final int LEFT_MOTOR_ID = 17;
    public static final int RIGHT_MOTOR_ID = 18;

    public static final int LEFT_ROLLER_MOTOR_ID = 19;
    public static final int RIGHT_ROLLER_MOTOR_ID = 20;

    public static final String CANBUS_LEFT = "rio";
    public static final String CANBUS_RIGHT = "rio";

    public static final boolean INVERTED_LEFT = false;
    public static final boolean INVERTED_RIGHT = false;

    public static final boolean BRAKE_LEFT = true;
    public static final boolean BRAKE_RIGHT = true;

    public static final boolean FOC_LEFT = true;
    public static final boolean FOC_RIGHT = true;

    public static final MeasurementUnit UNIT_LEFT = MeasurementUnit.DEGREES;
    public static final MeasurementUnit UNIT_RIGHT = MeasurementUnit.DEGREES;

    public static final double SENSOR_TO_MECHANISM_RATIO_LEFT = 1;
    public static final double SENSOR_TO_MECHANISM_RATIO_RIGHT = 1;

    public static final double kPLeft = 10;
    public static final double kILeft = 0;
    public static final double kDLeft = 2;
    public static final GravityType GRAVITY_LEFT = GravityType.ARM;

    public static final double kPRight = 10;
    public static final double kIRight = 0;
    public static final double kDRight = 2;
    public static final GravityType GRAVITY_RIGHT = GravityType.ARM;

    public static final double CRUISE_VELOCITY_LEFT_degps = 90;
    public static final double ACCELERATION_LEFT_degps2 = 180;
    public static final double JERK_LEFT_degp3 = 1000;

    public static final double CRUISE_VELOCITY_RIGHT_degps = 90;
    public static final double ACCELERATION_RIGHT_degps2 = 180;
    public static final double JERK_RIGHT_degp3 = 1000;

    public static final DCMotor DC_MOTOR_LEFT = DCMotor.getKrakenX44Foc(1);
    public static final double GEARING_LEFT = 1;
    public static final double SIM_MOI_LEFT_kgm2 = 1;

    public static final DCMotor DC_MOTOR_RIGHT = DCMotor.getKrakenX44Foc(1);
    public static final double GEARING_RIGHT = 1;
    public static final double SIM_MOI_RIGHT_kgm2 = 1;

    // Roller config
    public static final boolean INVERTED_ROLLER_LEFT = false;
    public static final boolean INVERTED_ROLLER_RIGHT = true;

    public static final boolean BRAKE_ROLLER_LEFT = true;
    public static final boolean BRAKE_ROLLER_RIGHT = true;

    public static final boolean FOC_ROLLER_LEFT = true;
    public static final boolean FOC_ROLLER_RIGHT = true;

    public static final MeasurementUnit UNIT_ROLLER_LEFT = MeasurementUnit.DEGREES;
    public static final MeasurementUnit UNIT_ROLLER_RIGHT = MeasurementUnit.DEGREES;

    public static final double SENSOR_TO_MECHANISM_RATIO_ROLLER_LEFT = 1;
    public static final double SENSOR_TO_MECHANISM_RATIO_ROLLER_RIGHT = 1;

    public static final DCMotor DC_MOTOR_ROLLER_LEFT = DCMotor.getKrakenX44Foc(1);
    public static final double GEARING_ROLLER_LEFT = 1;
    public static final double SIM_MOI_ROLLER_LEFT_kgm2 = 0.001;

    public static final DCMotor DC_MOTOR_ROLLER_RIGHT = DCMotor.getKrakenX44Foc(1);
    public static final double GEARING_ROLLER_RIGHT = 1;
    public static final double SIM_MOI_ROLLER_RIGHT_kgm2 = 0.001;

    public static final double INTAKE_VOLTS = 6;
    public static final double EXPEL_VOLTS = -6;

    // Physical properties (for constraints)
    public static final double MIN_ANGLE_LEFT_deg = -90;
    public static final double MAX_ANGLE_LEFT_deg = 90;

    public static final double MIN_ANGLE_RIGHT_deg = -90;
    public static final double MAX_ANGLE_RIGHT_deg = 90;

    public static final double RAISE_PHASE_2_THRESHOLD_deg = 45;
    public static final double RAISE_FINAL_THRESHOLD_deg = 60;
    public static final double RAISE_TOLERANCE_deg = 5;
    public static final double LOWER_PHASE_2_THRESHOLD_deg = -45;
    public static final double LOWER_FINAL_THRESHOLD_deg = -60;
    public static final double LOWER_TOLERANCE_deg = 5;

    // DualArm2d
    public static final int VISUALIZER_WIDTH = 10;
    public static final int VISUALIZER_HEIGHT = 10;

    public static final double LEFT_ARM_LENGTH = 1;
    public static final double LEFT_ARM_ANGLE = MIN_ANGLE_LEFT_deg;
    public static final double LEFT_LINE_WIDTH = 2;
    public static final double LEFT_ARM_VISUAL_OFFSET_deg = 180;

    public static final double RIGHT_ARM_LENGTH = 1;
    public static final double RIGHT_ARM_ANGLE = MIN_ANGLE_RIGHT_deg;
    public static final double RIGHT_LINE_WIDTH = 2;
    public static final double RIGHT_ARM_VISUAL_OFFSET_deg = 0;

    public static final double LEFT_ROLLER_LENGTH = 0.2;
    public static final double RIGHT_ROLLER_LENGTH = 0.2;
    public static final double LEFT_ROLLER_LINE_WIDTH = 2;
    public static final double RIGHT_ROLLER_LINE_WIDTH = 2;
    public static final double LEFT_ROLLER_ANGLE_OFFSET_deg = -90;
    public static final double RIGHT_ROLLER_ANGLE_OFFSET_deg = 90;
}
