use serialport;
use std::time::Duration;
use std::thread::sleep;
use std::f64::consts::PI;
use std::time::SystemTime;

fn generate_cmd<'a>(motor: &[i8; 3], cmd_str: &'a mut String) -> &'a [u8] {

    for i in 0..3
    {
        *cmd_str += (i + 2).to_string().as_ref();
        if motor[i] >= 0 { //at 0 meaning not force stop command here
            *cmd_str += "F";
        } else {
            *cmd_str += "R";
        }

        let param: u8 = motor[i].abs() as u8;
        //assert_eq!(param <= 100, true); // should remove at release

        let param_str: String = format!("{:>03}", param);
        *cmd_str += param_str.as_str();

    }

    cmd_str.push_str("\n");
    let cmd = (*cmd_str).as_bytes();
    //println!("start motor: {}", *cmd_str);
    return &cmd;
}

fn main() {
    let mut port = serialport::new("/dev/ttyAMA0", 115200)
        .stop_bits(serialport::StopBits::One)
        .data_bits(serialport::DataBits::Eight)
        .timeout(Duration::from_millis(10))
        .open()
        .unwrap_or_else(|e| {
            eprintln!("Failed to open \"{}\". Error: {}", "/dev/ttyAMA0", e);
            ::std::process::exit(1);
    });

    const MOTOR1_DIR :f64 = ((-60 + 90) as f64) / (360 as f64) * 2.0 * PI;
    const MOTOR2_DIR :f64 = ((60 + 90) as f64) / (360 as f64) * 2.0 * PI;
    const MOTOR3_DIR :f64 = ((180 + 90) as f64) / (360 as f64) * 2.0 * PI;

    let motor_dir_x_y :[[f64; 2]; 3] = [[MOTOR1_DIR.cos(), MOTOR1_DIR.sin()],
                                        [MOTOR2_DIR.cos(), MOTOR2_DIR.sin()],
                                        [MOTOR3_DIR.cos(), MOTOR3_DIR.sin()]];

    let now = SystemTime::now();
    loop {
        if now.elapsed().unwrap().as_secs_f64() >= 2.0 {
            break;
        }

        let direction_sceta :u16 = 90;// 0 ~ 360
        let direction_sceta_dig : f64 = (direction_sceta as f64) / (360 as f64) * 2.0 * PI;
        let direction_sceta_dig_x_y : [f64; 2] = [direction_sceta_dig.cos(), direction_sceta_dig.sin()];

        /*
           let motor1 :f64 = (direction_sceta_dig - MOTOR1_DIR).cos();
           let motor2 :f64 = (direction_sceta_dig - MOTOR2_DIR).cos();
           let motor3 :f64 = (direction_sceta_dig - MOTOR3_DIR).cos();
           */

        let motor1 :f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[0][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[0][1];
        let motor2 :f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[1][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[1][1];
        let motor3 :f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[2][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[2][1];

        let motor :[i8; 3] = [(motor1*-50.0) as i8, (motor2*-50.0) as i8, (motor3*-50.0) as i8]; //should -100 to 100
        let mut cmd_str =  String::from("1F000"); //unused motor channel 1
        let cmd = generate_cmd(&motor, &mut cmd_str);

        //println!("start motor(byte): {:?}", cmd);

        port.write(cmd).unwrap();
        port.flush().unwrap();
    }

    let poweroff_all_motor: &[u8; 21] = b"1F0002F0003F0004F000\n";
    //let force_stop_motor: &[u8; 21] = b"1R0002R0003R0004R000\n";
    match port.write(poweroff_all_motor)
    {
        Ok(_) => println!("poweroff motor"),
        Err(e) => eprintln!("{:?}", e),
    }
    port.flush().unwrap();
}
