use serialport;
use std::f64::consts::PI;
use std::time::{Instant, Duration};
use std::io;
use std::sync::{Arc, Mutex};
use std::thread;
use rppal::spi;
use rppal::spi::{Spi, Bus, SlaveSelect};
use rppal::gpio::{Gpio, Level};

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

    let mut error :[f64; 3] = [0.0, 0.0, 0.0];
    let mut integral :f64 = 0.0;

    const KP :f64 = 0.6;
    const KI :f64 = 0.00005;
    const KD :f64 = 0.00002;

    let from_controller_params :Arc<Mutex<(u16, u8)>> = Arc::new(Mutex::new((0, 0)));
    let from_controller_params_clone = Arc::clone(&from_controller_params);

    let _handle = thread::spawn(move || {
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

    let from_sensor_dir :Arc<Mutex<u16>> = Arc::new(Mutex::new(0));
    let from_sensor_dir_clone = Arc::clone(&from_sensor_dir);

    let _handle2 = thread::spawn(move || {
        //let ten_micros = time::Duration::from_micros(10);
        let mut spi = Spi::new( Bus::Spi0, SlaveSelect::Ss0, 1_000_000, spi::Mode::Mode0 ).expect( "Failed Spi::new" ); //1MHz
        let write_data :Vec<u8> = vec![0x20];
        let mut read_data1 :Vec<u8> = vec![0];
        let mut read_data2 :Vec<u8> = vec![0];
        loop{
            let _ret = spi.write( &write_data );
            //thread::sleep(ten_micros);
            let _ret = spi.read( &mut read_data1 ).expect("Failed Spi::read");
            //thread::sleep(ten_micros);
            let _ret = spi.read( &mut read_data2 ).expect("Failed Spi::read");
            //thread::sleep(ten_micros);

            let dir: u16 = ((read_data1[0] as u16) << 8 ) | read_data2[0] as u16;
            //println!("{}", dir);
            let mut param = from_sensor_dir_clone.lock().unwrap();
            *param = dir;
        }
    });

    let program_switch :Arc<Mutex<bool>> = Arc::new(Mutex::new(false));
    let program_switch_clone = Arc::clone(&program_switch);
    let _handle3 = thread::spawn(move || {
        let gpio = Gpio::new().unwrap();
        let pin = gpio.get(21).unwrap().into_input();
        loop{
            thread::sleep(Duration::from_millis(50));
            let val :bool = match pin.read() { Level::High => true, Level::Low => false, };
            let mut param = program_switch_clone.lock().unwrap();
            *param = val;
        }
    });
    
    thread::sleep(Duration::from_millis(100));

    loop {
        let pin_val = *(program_switch.lock().unwrap());
        if pin_val == false { break; }
    }

    loop {
        let pin_val = *(program_switch.lock().unwrap());
        if pin_val == true { break; }
    }

    let now = Instant::now();
    'outer: loop {
        let pin_val = *(program_switch.lock().unwrap());
        if pin_val == false { break 'outer; }

        let (direction_sceta, power_u8) = *(from_controller_params.lock().unwrap());
        let power: f64 = power_u8 as f64 / 255.0 as f64;

        let robot_dir :u16 = 90; //controll

        let sensor_dir :u16 = 360 - *(from_sensor_dir.lock().unwrap()); //0~360, reverse
        let mod_robot_dir :u16 = if robot_dir <= 180 {180 + robot_dir} else {robot_dir - 180}; //centelyzed by 180
        let mod_sensor_dir :u16 = if sensor_dir <= 180 {180 + sensor_dir} else {sensor_dir - 180}; //centelyzed by 180

        let direction_sceta_dig : f64 = (direction_sceta as f64) / (360 as f64) * 2.0 * PI;
        let direction_sceta_dig_x_y : [f64; 2] = [direction_sceta_dig.cos(), direction_sceta_dig.sin()];

        let mut motor1 :f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[0][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[0][1];
        let mut motor2 :f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[1][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[1][1];
        let mut motor3 :f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[2][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[2][1];

        motor1 = motor1 * power;
        motor2 = motor2 * power;
        motor3 = motor3 * power;
        
        //senosrs dir feedback
        let dir_diff_angle :f64 = (mod_robot_dir as f64) - (mod_sensor_dir as f64); //-180~180 //mod is centelyzed by 180
        let mut dir_diff :f64 = dir_diff_angle / 180.0; // -1.0~1.0

        dir_diff = dir_diff.clamp(-0.5, 0.5);

        //PID
        let time_after_command: f64 = now.elapsed().as_secs_f64() - last_command_time;

        error[0] = error[1];
        error[1] = dir_diff;
        integral += (error[0] + error[1])/2.0 * time_after_command;
        let delta_dir_motion :f64 = KP*error[1] + KI*integral + (KD*((error[1] - error[0]) / time_after_command));

        motor1 += delta_dir_motion;
        motor2 += delta_dir_motion;
        motor3 += delta_dir_motion;
        
        //log scale
        /*
        motor1 = if motor1 >= 0.0 { (motor1 + 1.0).log2() } else { (motor1.abs() + 1.0).log2() * -1.0 };
        motor2 = if motor2 >= 0.0 { (motor2 + 1.0).log2() } else { (motor2.abs() + 1.0).log2() * -1.0 };
        motor3 = if motor3 >= 0.0 { (motor3 + 1.0).log2() } else { (motor3.abs() + 1.0).log2() * -1.0 };
        */

        // clamp
        motor1 = motor1.clamp(-1.0, 1.0);
        motor2 = motor2.clamp(-1.0, 1.0);
        motor3 = motor3.clamp(-1.0, 1.0);

        //save motor
        if motor1.abs() < 0.1 && motor2.abs() < 0.1 && motor3.abs() < 0.1 { motor1 = 0.0; motor2 = 0.0; motor3 = 0.0; }

        let motor :[i8; 3] = [(motor1*-50.0) as i8, (motor2*-50.0) as i8, (motor3*-50.0) as i8]; //should -100 to 100
        let mut cmd_str =  String::from("1F000"); //unused motor channel 1
        let cmd = generate_cmd(&motor, &mut cmd_str);

        //println!("start motor(byte): {:?}", cmd);

        port.write(cmd).unwrap();
        port.flush().unwrap();

        // used by PID
        last_command_time = now.elapsed().as_secs_f64();
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
