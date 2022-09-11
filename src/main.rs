use serialport;
use std::f64::consts::PI;
use std::time::{Instant, Duration};
use std::io;
use std::sync::{Arc, Mutex};
use std::thread;

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
                                        
    //used by PID
    let mut last_command_time: f64 = 0.0;

    let mut pre_val :[f64; 3] = [0.0, 0.0, 0.0];

    let mut error :[[f64; 3]; 2] = [
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ];

    let mut integral :[f64; 3] = [0.0, 0.0, 0.0];

    const KP :f64 = 0.2;
    const KI :f64 = 0.2;
    const KD :f64 = 0.2;

    let from_controller_params: Arc<Mutex<(u16, u8)>> = Arc::new(Mutex::new((0, 0)));

    let from_controller_params_clone = Arc::clone(&from_controller_params);

    let handle = thread::spawn(move || {
        loop {
            let mut from_controller_str = String::new();
            io::stdin().read_line(&mut from_controller_str).unwrap();
            let mut from_controller = from_controller_str.split(' ');
            let direction_sceta :u16 = from_controller.next().unwrap().parse::<u16>().unwrap();
            let power_u8: u8 = from_controller.next().unwrap().trim_matches('\n').parse::<u8>().unwrap();
            let mut params = from_controller_params_clone.lock().unwrap();
            *params = (direction_sceta, power_u8);
        }
    });


    let now = Instant::now();
    loop {
        let (direction_sceta, power_u8) = *(from_controller_params.lock().unwrap());
        let power: f64 = power_u8 as f64 / 255.0 as f64;

        //println!("{}, {}", direction_sceta, power);

        /*
        let time :f64 = now.elapsed().as_secs_f64();
        if time >= 4.0 {
            break;
        }

        let mut sceta: f64 = 0.0;
        
        if time <= 2.0 {
            sceta = (360.0 / 2.0) * time
        } else {
            sceta = 360.0 - ((360.0 / 2.0) * (time - 2.0));
        }

        let direction_sceta :u16 = sceta as u16;// 0 ~ 360
        */

        let direction_sceta_dig : f64 = (direction_sceta as f64) / (360 as f64) * 2.0 * PI;
        let direction_sceta_dig_x_y : [f64; 2] = [direction_sceta_dig.cos(), direction_sceta_dig.sin()];

        /*
           let motor1 :f64 = (direction_sceta_dig - MOTOR1_DIR).cos();
           let motor2 :f64 = (direction_sceta_dig - MOTOR2_DIR).cos();
           let motor3 :f64 = (direction_sceta_dig - MOTOR3_DIR).cos();
           */

        let mut motor1 :f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[0][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[0][1];
        let mut motor2 :f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[1][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[1][1];
        let mut motor3 :f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[2][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[2][1];

        motor1 = motor1 * power;
        motor2 = motor2 * power;
        motor3 = motor3 * power;

        //PID
        let time_after_command: f64 = now.elapsed().as_secs_f64() - last_command_time;

        error[0] = error[1];
        error[1] = [
            motor1 - pre_val[0],
            motor2 - pre_val[1],
            motor3 - pre_val[2],
        ];
        integral = [
            integral[0] + ((error[0][0] + error[1][0])/2.0 * time_after_command),
            integral[1] + ((error[0][1] + error[1][1])/2.0 * time_after_command),
            integral[2] + ((error[0][2] + error[1][2])/2.0 * time_after_command),
        ];
        let delta_motion :[f64; 3] = [
            KP*error[1][0] + KI*integral[0] + (KD*((error[1][0] - error[0][0]) / time_after_command)*0.0000001),
            KP*error[1][1] + KI*integral[1] + (KD*((error[1][1] - error[0][1]) / time_after_command)*0.0000001),
            KP*error[1][2] + KI*integral[2] + (KD*((error[1][2] - error[0][2]) / time_after_command)*0.0000001),
        ];

        motor1 = motor1 + delta_motion[0];
        motor2 = motor2 + delta_motion[1];
        motor3 = motor3 + delta_motion[2];
        
        //log scale
        motor1 = if motor1 >= 0.0 { (motor1 + 1.0).log2() } else { (motor1.abs() + 1.0).log2() * -1.0 };
        motor2 = if motor2 >= 0.0 { (motor2 + 1.0).log2() } else { (motor2.abs() + 1.0).log2() * -1.0 };
        motor3 = if motor3 >= 0.0 { (motor3 + 1.0).log2() } else { (motor3.abs() + 1.0).log2() * -1.0 };

        // clamp
        if motor1 > 1.0 { motor1  = 1.0; }
        if motor2 > 1.0 { motor2  = 1.0; }
        if motor3 > 1.0 { motor3  = 1.0; }
        if motor1 < -1.0 { motor1  = -1.0; }
        if motor2 < -1.0 { motor2  = -1.0; }
        if motor3 < -1.0 { motor3  = -1.0; }

        let motor :[i8; 3] = [(motor1*-50.0) as i8, (motor2*-50.0) as i8, (motor3*-50.0) as i8]; //should -100 to 100
        let mut cmd_str =  String::from("1F000"); //unused motor channel 1
        let cmd = generate_cmd(&motor, &mut cmd_str);

        //println!("start motor(byte): {:?}", cmd);

        port.write(cmd).unwrap();
        port.flush().unwrap();

        // used by PID
        last_command_time = now.elapsed().as_secs_f64();
        pre_val = [ motor1, motor2, motor3 ];
    }
    handle.join().unwrap();

    let poweroff_all_motor: &[u8; 21] = b"1F0002F0003F0004F000\n";
    //let force_stop_motor: &[u8; 21] = b"1R0002R0003R0004R000\n";
    match port.write(poweroff_all_motor)
    {
        Ok(_) => println!("poweroff motor"),
        Err(e) => eprintln!("{:?}", e),
    }
    port.flush().unwrap();
}
