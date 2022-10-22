use serialport;
use std::f64::consts::PI;
use core::f64::MAX;
use std::time::{Instant, Duration};
use std::io;
use std::sync::{Arc, Mutex};
use std::thread;
use rppal::spi;
use rppal::spi::{Spi, Bus, SlaveSelect};
use rppal::gpio::{Gpio, Level};
use evdev::{Device, RelativeAxisType, InputEventKind};

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

struct CrossPoint {
    first: [f64; 2], // x, y
    second: [f64; 2], // x, y
}

fn circle_cross_point(circle1: &[f64; 3], circle2: &[f64; 3]) -> Option<CrossPoint>
{
    let x1: f64 = circle2[0] - circle1[0];
    let y1: f64 = circle2[1] - circle1[1];
    let r1: f64 = circle1[2];
    let r2: f64 = circle2[2];

    let x1_pow2: f64 = x1.powi(2);
    let y1_pow2: f64 = y1.powi(2);
    let r1_pow2: f64 = r1.powi(2);
    let r2_pow2: f64 = r2.powi(2);

    let a: f64 = (x1_pow2 + y1_pow2 + r1_pow2 - r2_pow2 ) / 2.0;

    let d: f64 = ((x1_pow2 + y1_pow2)*r1_pow2) - a.powi(2);
    if d < 0.0 {
        return Option::None;
    } else {
        let d_sqrt: f64 = d.sqrt();
        let first: [f64; 2] = [
            ((a*x1 + y1*d_sqrt) / (x1_pow2 + y1_pow2) + circle1[0]),
            ((a*y1 - x1*d_sqrt) / (x1_pow2 + y1_pow2) + circle1[1]),
        ];
        let second: [f64; 2] = [
            ((a*x1 - y1*d_sqrt) / (x1_pow2 + y1_pow2) + circle1[0]),
            ((a*y1 + x1*d_sqrt) / (x1_pow2 + y1_pow2) + circle1[1]),
        ];
        return Option::Some(CrossPoint{ first, second });
    }

}

fn three_point_circle(point1: &[f64; 2], point2: &[f64; 2], point3: &[f64; 2]) -> [f64; 3]  // return
                                                                                            // circle
                                                                                            // x, y, r
{
    let x1: f64 = point1[0];
    let y1: f64 = point1[1];
    let x2: f64 = point2[0];
    let y2: f64 = point2[1];
    let x3: f64 = point3[0];
    let y3: f64 = point3[1];

    let a: f64 = 2.0 * ((x1 - x2)*(y1 - y3) - (x1 - x3)*(y1 - y2));
    let c1: f64 = x2.powi(2) - x1.powi(2) + y2.powi(2) - y1.powi(2);
    let c2: f64 = x3.powi(2) - x1.powi(2) + y3.powi(2) - y1.powi(2);

    let xp: f64 = ((y1 - y2)*c2 - (y1 - y3)*c1) / a;
    let yp: f64 = ((x1 - x3)*c1 - (x1 - x2)*c2) / a;

    let r = ((xp - x1).powi(2) + (yp - y1).powi(2)).sqrt();

    return [xp, yp, r];
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

    const MOTOR1_DIR: f64 = ((-60 + 90) as f64) / (360 as f64) * 2.0 * PI;
    const MOTOR2_DIR: f64 = ((60 + 90) as f64) / (360 as f64) * 2.0 * PI;
    const MOTOR3_DIR: f64 = ((180 + 90) as f64) / (360 as f64) * 2.0 * PI;

    let motor_dir_x_y: [[f64; 2]; 3] = [[MOTOR1_DIR.cos(), MOTOR1_DIR.sin()],
                                        [MOTOR2_DIR.cos(), MOTOR2_DIR.sin()],
                                        [MOTOR3_DIR.cos(), MOTOR3_DIR.sin()]];
                                        
    //let mut preval: [f64; 3] = [0.0, 0.0, 0.0];
    //used by PID
    let mut last_command_time: f64 = 0.0;

    let mut error: [f64; 3] = [0.0, 0.0, 0.0];
    let mut integral: f64 = 0.0;

    const KP: f64 = 0.3;
    const KI: f64 = 0.0;//10.0;
    const KD: f64 = 0.2;

    /*
    let from_controller_params: Arc<Mutex<(u16, u8)>> = Arc::new(Mutex::new((0, 0)));
    let from_controller_params_clone = Arc::clone(&from_controller_params);

    let _handle = thread::spawn(move || {
        loop {
            let mut from_controller = from_controller_str.split(' ');
            let direction_sceta: u16 = from_controller.next().unwrap().parse::<u16>().unwrap();
            let power_u8: u8 = from_controller.next().unwrap().trim_matches('\n').parse::<u8>().unwrap();
            let mut params = from_controller_params_clone.lock().unwrap();
            *params = (direction_sceta, power_u8);
        }
    });
    */

    let from_sensor_dir: Arc<Mutex<u16>> = Arc::new(Mutex::new(0));
    let from_sensor_dir_clone = Arc::clone(&from_sensor_dir);

    let dir_sensor_latency: Arc<Mutex<f64>> = Arc::new(Mutex::new(0.0));
    let dir_sensor_latency_clone = Arc::clone(&dir_sensor_latency);

    let _handle2 = thread::spawn(move || {
        //let ten_micros = time::Duration::from_micros(10);
        let mut spi0_0 = Spi::new( Bus::Spi0, SlaveSelect::Ss0, 1_000_000, spi::Mode::Mode0 ).expect( "Failed Spi::new" ); //1MHz
        let write_data: Vec<u8> = vec![0x20];
        let mut read_data1: Vec<u8> = vec![0];
        let mut read_data2: Vec<u8> = vec![0];
        //center dir first pos
        let _ret = spi0_0.write( &write_data );
        //thread::sleep(ten_micros);
        let _ret = spi0_0.read( &mut read_data1 ).expect("Failed Spi::read");
        //thread::sleep(ten_micros);
        let _ret = spi0_0.read( &mut read_data2 ).expect("Failed Spi::read");
        //thread::sleep(ten_micros);

        let center_dir: u16 = ((read_data1[0] as u16) << 8 ) | read_data2[0] as u16;
        let center_dir: u16 = center_dir;
        //println!("center_dir: {}", center_dir);

        let dir_sensor_now = Instant::now();
        let mut last_now_time: f64 = 0.0;

        loop{
            let one_cycle_latency: f64 = dir_sensor_now.elapsed().as_secs_f64() - last_now_time;
            last_now_time = dir_sensor_now.elapsed().as_secs_f64();

            let _ret = spi0_0.write( &write_data );
            //thread::sleep(ten_micros);
            let _ret = spi0_0.read( &mut read_data1 ).expect("Failed Spi::read");
            //thread::sleep(ten_micros);
            let _ret = spi0_0.read( &mut read_data2 ).expect("Failed Spi::read");
            //thread::sleep(ten_micros);

            let dir: u16 = ((read_data1[0] as u16) << 8 ) | read_data2[0] as u16;
            let dir_diff: i16 = dir as i16 - center_dir as i16;
            let centelyzed_dir: u16 = if dir_diff < 0 { (360 + dir_diff) as u16 } else { dir_diff as u16 }; //centelyzed by center_dir
            //println!("centelyzed_dir: {}", centelyzed_dir);
            let mut param = from_sensor_dir_clone.lock().unwrap();
            *param = centelyzed_dir;
            
            let mut latency = dir_sensor_latency_clone.lock().unwrap();
            *latency = one_cycle_latency;
        }
    });

    let program_switch: Arc<Mutex<bool>> = Arc::new(Mutex::new(false));
    let program_switch_clone = Arc::clone(&program_switch);
    let _handle3 = thread::spawn(move || {
        let gpio = Gpio::new().unwrap();
        let pin = gpio.get(6).unwrap().into_input();
        loop{
            thread::sleep(Duration::from_millis(50));
            let val :bool = match pin.read() { Level::High => true, Level::Low => false, };
            let mut param = program_switch_clone.lock().unwrap();
            *param = val;
        }
    });
   
    const SQRT_3: f64 = 1.73205080757;
    const CIRCLE_SENSORS_POS: [[f64; 3]; 12] = [ //[sensor_pos(x, y), sensor_dir(max:2*pi)]
        [0.0            , 5.0           , 2.0*PI*0.0/360.0], // 0

        [-5.0/(SQRT_3)  , 5.0           , 2.0*PI*30.0/360.0], // 1
        [-5.0*(SQRT_3)  , 5.0           , 2.0*PI*60.0/360.0], // 2
        [-10.0          , 0.0           , 2.0*PI*90.0/360.0], // 3
        [-5.0*(SQRT_3)  , -5.0          , 2.0*PI*120.0/360.0], // 4
        [-5.0           , -5.0*(SQRT_3) , 2.0*PI*150.0/360.0], // 5

        [0.0            , -10.0         , 2.0*PI*180.0/360.0], // 6

        [5.0            , -5.0*(SQRT_3) , 2.0*PI*210.0/360.0], // 7
        [5.0*(SQRT_3)   , -5.0          , 2.0*PI*240.0/360.0], // 8
        [10.0           , 0.0           , 2.0*PI*270.0/360.0], // 9
        [5.0*(SQRT_3)   , 5.0           , 2.0*PI*300.0/360.0], // 10
        [5.0/(SQRT_3)   , 5.0           , 2.0*PI*330.0/360.0], // 11
    ];

    let ball_pos_relative: Arc<Mutex<Option<[f64; 2]>>> = Arc::new(Mutex::new(Option::None));
    let ball_pos_relative_clone = Arc::clone(&ball_pos_relative);

    const BALL_MAX_DIST: f64 = 70.0;

    let ball_sensor_latency: Arc<Mutex<f64>> = Arc::new(Mutex::new(0.0));
    let ball_sensor_latency_clone = Arc::clone(&ball_sensor_latency);

    let _handle4 = thread::spawn(move || {
        //let lpc1114_wait = Duration::from_micros(10000);
        let mut spi1_0 = Spi::new( Bus::Spi1, SlaveSelect::Ss0, 1_000_000, spi::Mode::Mode0 ).expect( "Failed Spi::new" ); //1MHz
        let mut spi1_1 = Spi::new( Bus::Spi1, SlaveSelect::Ss1, 1_000_000, spi::Mode::Mode0 ).expect( "Failed Spi::new" ); //1MHz
        let mut spi1_2 = Spi::new( Bus::Spi1, SlaveSelect::Ss2, 1_000_000, spi::Mode::Mode0 ).expect( "Failed Spi::new" ); //1MHz
        let write_data: Vec<u8> = vec![0x40];
        let mut read_first_seg: Vec<u8> = vec![0];

        let mut read_data1_h: Vec<u8> = vec![0];
        let mut read_data1_l: Vec<u8> = vec![0];
        let mut read_data2_h: Vec<u8> = vec![0];
        let mut read_data2_l: Vec<u8> = vec![0];
        let mut read_data3_h: Vec<u8> = vec![0];
        let mut read_data3_l: Vec<u8> = vec![0];
        let mut read_data4_h: Vec<u8> = vec![0];
        let mut read_data4_l: Vec<u8> = vec![0];

        let mut sensor_val_from_slave0: [u16; 4] = [0; 4];
        let mut sensor_val_from_slave1: [u16; 4] = [0; 4];
        let mut sensor_val_from_slave2: [u16; 4] = [0; 4];

        let ball_sensor_now = Instant::now();
        let mut last_now_time: f64 = 0.0;

        loop{
            let one_cycle_latency: f64 = ball_sensor_now.elapsed().as_secs_f64() - last_now_time;
            last_now_time = ball_sensor_now.elapsed().as_secs_f64();

            let _ret = spi1_0.write( &write_data );
            //thread::sleep(lpc1114_wait);
            let _ret = spi1_0.read( &mut read_first_seg ).expect("Failed Spi::read");
            if read_first_seg[0] == 0x40 {

                let _ret = spi1_0.read( &mut read_data1_h ).expect("Failed Spi::read");
                let _ret = spi1_0.read( &mut read_data1_l ).expect("Failed Spi::read");

                let _ret = spi1_0.read( &mut read_data2_h ).expect("Failed Spi::read");
                let _ret = spi1_0.read( &mut read_data2_l ).expect("Failed Spi::read");

                let _ret = spi1_0.read( &mut read_data3_h ).expect("Failed Spi::read");
                let _ret = spi1_0.read( &mut read_data3_l ).expect("Failed Spi::read");

                let _ret = spi1_0.read( &mut read_data4_h ).expect("Failed Spi::read");
                let _ret = spi1_0.read( &mut read_data4_l ).expect("Failed Spi::read");

                let sensor_val1: u16 = ((read_data1_h[0] as u16) << 8 ) | read_data1_l[0] as u16;
                let sensor_val2: u16 = ((read_data2_h[0] as u16) << 8 ) | read_data2_l[0] as u16;
                let sensor_val3: u16 = ((read_data3_h[0] as u16) << 8 ) | read_data3_l[0] as u16;
                let sensor_val4: u16 = ((read_data4_h[0] as u16) << 8 ) | read_data4_l[0] as u16;
                sensor_val_from_slave0 = [sensor_val1, sensor_val2, sensor_val3, sensor_val4];
            }

            let _ret = spi1_1.write( &write_data );
            //thread::sleep(lpc1114_wait);
            let _ret = spi1_1.read( &mut read_first_seg ).expect("Failed Spi::read");
            if read_first_seg[0] == 0x40 {

                let _ret = spi1_1.read( &mut read_data1_h ).expect("Failed Spi::read");
                let _ret = spi1_1.read( &mut read_data1_l ).expect("Failed Spi::read");

                let _ret = spi1_1.read( &mut read_data2_h ).expect("Failed Spi::read");
                let _ret = spi1_1.read( &mut read_data2_l ).expect("Failed Spi::read");

                let _ret = spi1_1.read( &mut read_data3_h ).expect("Failed Spi::read");
                let _ret = spi1_1.read( &mut read_data3_l ).expect("Failed Spi::read");

                let _ret = spi1_1.read( &mut read_data4_h ).expect("Failed Spi::read");
                let _ret = spi1_1.read( &mut read_data4_l ).expect("Failed Spi::read");

                let sensor_val1: u16 = ((read_data1_h[0] as u16) << 8 ) | read_data1_l[0] as u16;
                let sensor_val2: u16 = ((read_data2_h[0] as u16) << 8 ) | read_data2_l[0] as u16;
                let sensor_val3: u16 = ((read_data3_h[0] as u16) << 8 ) | read_data3_l[0] as u16;
                let sensor_val4: u16 = ((read_data4_h[0] as u16) << 8 ) | read_data4_l[0] as u16;
                sensor_val_from_slave1 = [sensor_val1, sensor_val2, sensor_val3, sensor_val4];
            }

            let _ret = spi1_2.write( &write_data );
            //thread::sleep(lpc1114_wait);
            let _ret = spi1_2.read( &mut read_first_seg ).expect("Failed Spi::read");
            if read_first_seg[0] == 0x40 {

                let _ret = spi1_2.read( &mut read_data1_h ).expect("Failed Spi::read");
                let _ret = spi1_2.read( &mut read_data1_l ).expect("Failed Spi::read");

                let _ret = spi1_2.read( &mut read_data2_h ).expect("Failed Spi::read");
                let _ret = spi1_2.read( &mut read_data2_l ).expect("Failed Spi::read");

                let _ret = spi1_2.read( &mut read_data3_h ).expect("Failed Spi::read");
                let _ret = spi1_2.read( &mut read_data3_l ).expect("Failed Spi::read");

                let _ret = spi1_2.read( &mut read_data4_h ).expect("Failed Spi::read");
                let _ret = spi1_2.read( &mut read_data4_l ).expect("Failed Spi::read");

                let sensor_val1: u16 = ((read_data1_h[0] as u16) << 8 ) | read_data1_l[0] as u16;
                let sensor_val2: u16 = ((read_data2_h[0] as u16) << 8 ) | read_data2_l[0] as u16;
                let sensor_val3: u16 = ((read_data3_h[0] as u16) << 8 ) | read_data3_l[0] as u16;
                let sensor_val4: u16 = ((read_data4_h[0] as u16) << 8 ) | read_data4_l[0] as u16;
                sensor_val_from_slave2 = [sensor_val1, sensor_val2, sensor_val3, sensor_val4];
            }

            let sensor_val_circle = [
                sensor_val_from_slave0[0], sensor_val_from_slave1[0], sensor_val_from_slave2[0],
                sensor_val_from_slave0[1], sensor_val_from_slave1[1], sensor_val_from_slave2[1],
                sensor_val_from_slave0[2], sensor_val_from_slave1[2], sensor_val_from_slave2[2],
                sensor_val_from_slave0[3], sensor_val_from_slave1[3], sensor_val_from_slave2[3],
            ];
            // not need sort, max is enough
            let mut max_sensor_num: usize = 0;
            let mut max_sensor_val: u16 = 0;
            for i in 0..sensor_val_circle.len()
            {
                if max_sensor_val < sensor_val_circle[i] {
                    max_sensor_val = sensor_val_circle[i];
                    max_sensor_num = i;
                }
            }

            let three_sensor_point: [usize; 3] = match max_sensor_num 
            {
                0 => [11, 0, 1],
                11 => [10, 11, 0],
                _ => [max_sensor_num - 1 ,max_sensor_num, max_sensor_num + 1],
            };
            //println!("{:?}", three_sensor_point);

            let mut r_double: [f64; 3] = [0.0; 3];
            let mut r: [f64; 3] = [0.0; 3];

            let mut circles: [[f64; 3]; 3] = [[0.0, 0.0, 0.0]; 3]; // x, y, r

            for i in 0..3 {
                r_double[i] = 3000.0 / ((sensor_val_circle[three_sensor_point[i]] as f64).sqrt());
                r[i] = r_double[i] / 2.0;
                circles[i] = [
                    CIRCLE_SENSORS_POS[three_sensor_point[i]][0] // sensor_pos_x
                        +
                        (CIRCLE_SENSORS_POS[three_sensor_point[i]][2].sin() * -1.0 * r[i]) // sensor_dir.sin() * -1.0 * r
                    ,
                    CIRCLE_SENSORS_POS[three_sensor_point[i]][1] // sensor_pos_y
                        +
                        (CIRCLE_SENSORS_POS[three_sensor_point[i]][2].cos() * r[i]) // sensor_dir.cos() * r

                    , r[i] ];
            }
            //println!("circles1: 2*r (dist when dir(ball-sensor).cos == 1): {}", circles[1][2] * 2.0);
            //println!("circles1: pos: x:{}, y:{}", circles[1][0], circles[1][1]);

            let cross_point_a = match circle_cross_point(&circles[0], &circles[1])
            {
                Some(cross_point) => cross_point,
                None => { // no cross point , then return half point both circle center
                    let half_point_x: f64 = (circles[0][0] + circles[1][0]) / 2.0;
                    let half_point_y: f64 = (circles[0][1] + circles[1][1]) / 2.0;
                    CrossPoint { first: [half_point_x, half_point_y], second: [half_point_x, half_point_y] } // return first second same point
                },
            };

            let cross_point_b = match circle_cross_point(&circles[1], &circles[2])
            {
                Some(cross_point) => cross_point,
                None => { // no cross point , then return half point both circle center
                    let half_point_x: f64 = (circles[1][0] + circles[2][0]) / 2.0;
                    let half_point_y: f64 = (circles[1][1] + circles[2][1]) / 2.0;
                    CrossPoint { first: [half_point_x, half_point_y], second: [half_point_x, half_point_y] } // return first second same point
                },
            };

            let cross_point_c = match circle_cross_point(&circles[2], &circles[0])
            {
                Some(cross_point) => cross_point,
                None => { // no cross point , then return half point both circle center
                    let half_point_x: f64 = (circles[2][0] + circles[0][0]) / 2.0;
                    let half_point_y: f64 = (circles[2][1] + circles[0][1]) / 2.0;
                    CrossPoint { first: [half_point_x, half_point_y], second: [half_point_x, half_point_y] } // return first second same point
                },
            };

            // calc 3point-include circle radius in 8 pattern // 2^(3C2) = 8
            // choise minimum one and the circle center is ball pos
            // but, when circle-crosspoint <-> sensor dist is close, 3point circle radius is close
            // too.
            // so, can't detect long dist case.
            // now we try to choise most long dist one instead.
            // but not works well at small dist.
            // try avg 8 circle center.
            //let mut minimum_circle_r = MAX;
            //let mut most_long_dist = 0.0;
            let mut ball_pos: [f64; 2] = [0.0 as f64, 0.0 as f64];

            for i in [cross_point_a.first, cross_point_a.second] {
                for j in [cross_point_b.first, cross_point_b.second] {
                    for k in [cross_point_c.first, cross_point_c.second] {
                        let circle_p: [f64; 3] = three_point_circle(&i, &j, &k);
                        ball_pos = [
                            ball_pos[0] + circle_p[0]*(1.0/8.0),
                            ball_pos[1] + circle_p[1]*(1.0/8.0)
                        ]; //x, y

                        /*
                        if circle_p[2] < minimum_circle_r { //r
                            minimum_circle_r = circle_p[2];
                            ball_pos = [circle_p[0], circle_p[1]]; //x, y
                        }
                        */
                        /*
                        if most_long_dist < (circle_p[0].powi(2) * circle_p[1].powi(2)).sqrt() {
                            most_long_dist = (circle_p[0].powi(2) * circle_p[1].powi(2)).sqrt();
                            ball_pos = [circle_p[0], circle_p[1]]; //x, y
                        }
                        */
                    }
                }
            }

            let mut ball_pos_option: Option<[f64; 2]> = Option::None;

            let ball_dist: f64 = (ball_pos[0].powi(2) + ball_pos[1].powi(2)).sqrt(); //ball dist
            if ball_dist >= BALL_MAX_DIST {
                // not found
                ball_pos_option = Option::None;
                //println!("BALL not found (too long dist)");
            }  else {
                ball_pos_option = Option::Some([ball_pos[0], ball_pos[1]]);
            }
            //println!("ball_pos_option: {:?}", ball_pos_option);
            //println!("ball_dist: {:?}", ball_dist);
            let mut param = ball_pos_relative_clone.lock().unwrap();
            *param = ball_pos_option;

            let mut latency = ball_sensor_latency_clone.lock().unwrap();
            *latency = one_cycle_latency;
        }
    });

    let machine_pos: Arc<Mutex<[f64; 2]>> = Arc::new(Mutex::new([0.0; 2]));
    let machine_pos_clone = Arc::clone(&machine_pos);

    let from_sensor_dir_clone2 = Arc::clone(&from_sensor_dir);

    let usb_mouse_latency: Arc<Mutex<f64>> = Arc::new(Mutex::new(0.0));
    let usb_mouse_latency_clone = Arc::clone(&usb_mouse_latency);

    // usb-mouse
    let _handle5 = thread::spawn(move || {
        let mut device = Device::open("/dev/input/by-id/usb-Avago_USB_LaserStream_TM__Mouse-event-mouse").unwrap();
        let mut accum_val :[f64; 2] = [0.0; 2];

        let usb_mouse_now = Instant::now();
        let mut last_now_time: f64 = 0.0;

        loop{
            let one_cycle_latency: f64 = usb_mouse_now.elapsed().as_secs_f64() - last_now_time;
            last_now_time = usb_mouse_now.elapsed().as_secs_f64();

            let mut val :[i32; 2] = [0; 2];
            for ev in device.fetch_events().unwrap() {
                match ev.kind() {
                    InputEventKind::RelAxis(axis) => {
                        match axis {
                            RelativeAxisType::REL_X => val[1] = ev.value(),
                            RelativeAxisType::REL_Y => val[0] = ev.value(),
                            _ => (),
                        }
                    },
                    _ => (),
                }
            }

            let sensor_dir: u16 = 360 - *(from_sensor_dir_clone2.lock().unwrap()); //0~360, reverse
            let sensor_dir_dig: f64 = 2.0 * PI *(sensor_dir as f64 / 360.0);
            let sensor_dir_sin: f64 = sensor_dir_dig.sin();
            let sensor_dir_cos: f64 = sensor_dir_dig.cos();

            //rotate
            let rotate_x: f64 = (val[0] as f64 * sensor_dir_cos) + (val[1] as f64 * (-1.0 * sensor_dir_sin));
            let rotate_y: f64 = (val[0] as f64 * sensor_dir_sin) + (val[1] as f64 * sensor_dir_cos);

            //scale
            let rotate_x: f64 = rotate_x * 0.003;
            let rotate_y: f64 = rotate_y * 0.003;

            accum_val[0] += rotate_x;
            accum_val[1] += rotate_y;

            //println!("{:?}", accum_val);

            let mut param = machine_pos_clone.lock().unwrap();
            *param = accum_val;
            
            let mut latency = usb_mouse_latency_clone.lock().unwrap();
            *latency = one_cycle_latency;
        }
    });

    let target_pos_relative: Arc<Mutex<Option<[f64; 2]>>> = Arc::new(Mutex::new(Option::None));
    let target_pos_relative_clone = Arc::clone(&target_pos_relative);

    const BALL_R: f64 = 3.75;
    const MACHINE_R: f64 = 11.0;
    const MARGIN: f64 = 20.0; //wrapround magin;
    const C_R: f64 = BALL_R + MACHINE_R + MARGIN;

    let from_sensor_dir_clone3 = Arc::clone(&from_sensor_dir);
    let machine_pos_clone2 = Arc::clone(&machine_pos);

    let calc_target_latency: Arc<Mutex<f64>> = Arc::new(Mutex::new(0.0));
    let calc_target_latency_clone = Arc::clone(&calc_target_latency);

    // calc target_pos
    let _handle6 = thread::spawn(move || {
        const PRE_SAMPLE_SIZE: usize = 1000;
        const MACHIME_SPEED: f64 = 100.0;

        let mut previous_ball_pos: [[f64; 2]; PRE_SAMPLE_SIZE] = [[0.0; 2]; PRE_SAMPLE_SIZE]; // need 3 times avg and ball tracking more.
        let mut previous_target_pos: [f64; 2] = [0.0; 2];

        let calc_target_now = Instant::now();
        let mut last_now_time: f64 = 0.0;

        loop{
            let one_cycle_latency: f64 = calc_target_now.elapsed().as_secs_f64() - last_now_time;
            last_now_time = calc_target_now.elapsed().as_secs_f64();

            let mut target_pos_relative_option: Option<[f64; 2]> = Option::None;

            let ball_pos: Option<[f64; 2]> = *(ball_pos_relative.lock().unwrap());
            match ball_pos {
                Some([x, y]) => { 
                    if x.is_normal() && y.is_normal() {
                        let sensor_dir: u16 = 360 - *(from_sensor_dir_clone3.lock().unwrap()); //0~360, reverse
                        let sensor_dir_dig: f64 = 2.0 * PI *(sensor_dir as f64 / 360.0);
                        let sensor_dir_sin: f64 = sensor_dir_dig.sin();
                        let sensor_dir_cos: f64 = sensor_dir_dig.cos();

                        let rotate_x:f64 = (x * sensor_dir_cos) + (y * (-1.0 * sensor_dir_sin));
                        let rotate_y:f64 = (x * sensor_dir_sin) + (y * sensor_dir_cos);

                        let relative_ball_pos_now: [f64; 2] = [rotate_x, rotate_y];
                        //println!("relative ball_pos: {:?}", ball_pos_now);

                        // calc absolute_ball_pos
                        let machine_pos: [f64; 2] = *(machine_pos_clone2.lock().unwrap());
                        let absolute_ball_pos: [f64; 2] = [ machine_pos[0] + relative_ball_pos_now[0],
                                                            machine_pos[1] + relative_ball_pos_now[1], ];
                        // record previous absolute ball pos
                        for i in (1..PRE_SAMPLE_SIZE).rev()
                        {
                            previous_ball_pos[i] = previous_ball_pos[i - 1];
                        }
                        previous_ball_pos[0] = absolute_ball_pos;

                        let absolute_ball_pos_now: [f64; 2] = [ // three times average
                            (previous_ball_pos[2][0] + previous_ball_pos[1][0] + previous_ball_pos[0][0]) / 3.0,
                            (previous_ball_pos[2][1] + previous_ball_pos[1][1] + previous_ball_pos[0][1]) / 3.0,
                        ];
                        //println!("absolute_ball_pos_now: {:?}", absolute_ball_pos_now);
                        
                        // calc relative pos
                        let relative_ball_pos_now: [f64; 2] =  [
                            absolute_ball_pos_now[0] - machine_pos[0],
                            absolute_ball_pos_now[1] - machine_pos[1], 
                        ];
                        
                        //uniform linear mortion trajectory
                        let mut b1: [f64; 2] = [0.0; 2];
                        for i in 0..PRE_SAMPLE_SIZE
                        {
                            b1[0] += previous_ball_pos[i][0];
                            b1[1] += previous_ball_pos[i][1];
                        }
                        b1[0] = b1[0] / (PRE_SAMPLE_SIZE as f64); // b1 sample avg
                        b1[1] = b1[1] / (PRE_SAMPLE_SIZE as f64); // b1 sample avg
                        let b1_sample_point: usize = (PRE_SAMPLE_SIZE / 2) as usize;
                        let b1_to_b2_time: f64 = (b1_sample_point as f64) * one_cycle_latency;
                        let b2: [f64; 2] = absolute_ball_pos_now;//previous_ball_pos[0];

                        let ball_motion_vec: [f64; 2] = [
                            b2[0] - b1[0],
                            b2[1] - b1[1], 
                        ];
                        let ball_motion_vec_length: f64 = (ball_motion_vec[0].powi(2) + ball_motion_vec[1].powi(2)).sqrt();
                        let ball_speed: f64 = ball_motion_vec_length / b1_to_b2_time;
                        //println!("ball_motion_vec: {:?}, ball_speed: {}", ball_motion_vec, ball_speed);
                        let ball_motion_vec_norm: [f64; 2] = [
                            ball_motion_vec[0] / ball_motion_vec_length,
                            ball_motion_vec[1] / ball_motion_vec_length,
                        ];
                        
                        let ball_motion_vec_norm_and_rev_ball_vec_cross: f64 = 
                            (ball_motion_vec_norm[0] * -1.0 * relative_ball_pos_now[0]) + (ball_motion_vec_norm[1] * -1.0 * relative_ball_pos_now[1]);

                        let relative_ball_dist_pow2: f64 = relative_ball_pos_now[0].powi(2) + relative_ball_pos_now[1].powi(2);
 
                        let a: f64 = ball_speed.powi(2) - MACHIME_SPEED.powi(2);
                        let b: f64 = -2.0 * ball_speed * ball_motion_vec_norm_and_rev_ball_vec_cross;
                        let c: f64 = relative_ball_dist_pow2;

                        let mut skip_flag: bool = false;
                        let d: f64 = b.powi(2) - (4.0 * a * c);
                        let mut t: f64 = 0.0;
                        if d >= 0.0
                        {
                            let t1 = ( (-1.0 * b) + d.sqrt() ) / (2.0 * a);
                            let t2 = ( (-1.0 * b) - d.sqrt() ) / (2.0 * a);

                            if t1 < 0.0 && t2 < 0.0 {
                                // same as below skip state
                                skip_flag = true;
                            } else if t1 < 0.0 {
                                t = t2;
                            } else if t2 < 0.0 {
                                t = t1;
                            } else { // t1 >=0.0 && t2 >= 0.0
                                if t1 > t2 {
                                    t = t2;
                                } else {
                                    t = t1;
                                }
                            }
                        } else {
                            // if d < 0 or not exist t >= 0 val,
                            //
                            // should skip target_ball_pos, set target_poit to [0,0] + ball_motion_vec dir ?
                            skip_flag = true;
                        }

                        if skip_flag == true {
                            // ignore t
                            let target_point: [f64; 2]= ball_motion_vec; //should use norm val? //use only dir

                            let target_pos_rotate_x:f64 = (target_point[0] * sensor_dir_cos) + (target_point[1] * -1.0 * (-1.0 * sensor_dir_sin));
                            let target_pos_rotate_y:f64 = (target_point[0] * -1.0 * sensor_dir_sin) + (target_point[1] * sensor_dir_cos);

                            let target_pos_rotate: [f64; 2] = [target_pos_rotate_x, target_pos_rotate_y];
                            //println!("target_pos_rotate: {:?}", target_pos_rotate);
                            target_pos_relative_option = Option::Some(target_pos_rotate);
                            previous_target_pos = target_pos_rotate;
                        } else {
                            // calc based t
                            let mut target_ball_pos: [f64; 2] =  relative_ball_pos_now;
                            target_ball_pos = [
                                target_ball_pos[0] + (ball_motion_vec_norm[0] * t),
                                target_ball_pos[1] + (ball_motion_vec_norm[1] * t),
                            ];

                            let ball_dir: f64 = (2.0 * PI) - (target_ball_pos[0].atan2(target_ball_pos[1]));
                            let ball_dist: f64 = (target_ball_pos[0].powi(2) + target_ball_pos[1].powi(2)).sqrt();

                            let enemy_goal_dir: f64 = 0.0; // relative to ball coordinates

                            // calc target pos for football game
                            if (ball_dir - enemy_goal_dir).cos()  > 0.5 {
                                //target enemy goal
                                target_pos_relative_option = Option::Some(target_ball_pos);
                            } else {
                                // wraparound
                                let own_goal_dir: f64 = PI; // relative to ball coordinates

                                let own_goal_vec: [f64; 2] = [own_goal_dir.sin(), own_goal_dir.cos()];
                                let enemy_goal_vec: [f64; 2] = [enemy_goal_dir.sin(), enemy_goal_dir.cos()];

                                let ball_dist_clamp: f64 = ball_dist.clamp(C_R+1.0, BALL_MAX_DIST);
                                let target_ball_pos: [f64; 2] = [
                                    target_ball_pos[0]*ball_dist_clamp/ball_dist, 
                                    target_ball_pos[1]*ball_dist_clamp/ball_dist, 
                                ];

                                let machine_c_r: f64 = (ball_dist_clamp.powi(2) - C_R.powi(2)).sqrt();

                                let circle_a: [f64; 3] = [0.0, 0.0, machine_c_r]; //machine relative position 
                                let circle_b: [f64; 3] = [target_ball_pos[0], target_ball_pos[1], C_R]; //ball relative position

                                match circle_cross_point(&circle_a, &circle_b) {
                                    Some(cross_point) => {
                                        let mut target_point: [f64; 2] = [0.0; 2];
                                        let first_point: [f64; 2] = cross_point.first;
                                        let second_point: [f64; 2] = cross_point.second;

                                        let ball_first_point: [f64; 2] = [target_ball_pos[0] - first_point[0], target_ball_pos[1] - first_point[1]];
                                        let ball_first_point_normal: [f64; 2] = [ball_first_point[0] / C_R, ball_first_point[1] / C_R];
                                        let cos_ball_first_point_own_goal: f64 =  ball_first_point_normal[0]*own_goal_vec[0] + ball_first_point_normal[1]*own_goal_vec[1];

                                        let ball_second_point: [f64; 2] = [target_ball_pos[0] - second_point[0], target_ball_pos[1] - second_point[1]];
                                        let ball_second_point_normal: [f64; 2] = [ball_second_point[0] / C_R, ball_second_point[1] / C_R];
                                        let cos_ball_second_point_own_goal: f64 =  ball_second_point_normal[0]*own_goal_vec[0] + ball_second_point_normal[1]*own_goal_vec[1];

                                        //avoid own goal
                                        if cos_ball_first_point_own_goal < cos_ball_second_point_own_goal {
                                            target_point = first_point;
                                        } else {
                                            target_point = second_point;
                                        }

                                        let target_pos_rotate_x:f64 = (target_point[0] * sensor_dir_cos) + (target_point[1] * -1.0 * (-1.0 * sensor_dir_sin));
                                        let target_pos_rotate_y:f64 = (target_point[0] * -1.0 * sensor_dir_sin) + (target_point[1] * sensor_dir_cos);

                                        let target_pos_rotate: [f64; 2] = [target_pos_rotate_x, target_pos_rotate_y];
                                        //println!("target_pos_rotate: {:?}", target_pos_rotate);
                                        target_pos_relative_option = Option::Some(target_pos_rotate);
                                        previous_target_pos = target_pos_rotate;
                                    },
                                    None => {
                                        // not expect in this case;
                                        println!("some thing wrong");
                                        println!("circle_a: {:?}", circle_a);
                                        println!("circle_b: {:?}", circle_b);
                                    },
                                };
                            }
                        }
                    } else { //ball_pos x, y is not normal
                        //target_pos_relative_option = Option::None;
                        target_pos_relative_option = Option::Some(previous_target_pos);
                    }
                },
                None => {
                    target_pos_relative_option = Option::None;
                },
            };

            //println!("target_point_relative_option: {:?}", target_pos_relative_option);
            let mut param = target_pos_relative_clone.lock().unwrap();
            *param = target_pos_relative_option;

            let mut latency = calc_target_latency_clone.lock().unwrap();
            *latency = one_cycle_latency;
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
    let mut cycle_num: u8 = 0;

    // start main loop
    'outer: loop {
        let pin_val = *(program_switch.lock().unwrap());
        if pin_val == false { break 'outer; }

        /*
        let (direction_sceta, power_u8) = *(from_controller_params.lock().unwrap());
        let power: f64 = power_u8 as f64 / 255.0 as f64;
        */

        let read_target_pos_relative: Option<[f64; 2]> = *(target_pos_relative.lock().unwrap());


        let machine_pos: [f64; 2] = *(machine_pos.lock().unwrap());
        //println!("{:?}", machine_pos);

        let mut direction_sceta_dig: f64 = 0.0;
        let mut power: f64 = 0.0;

        match read_target_pos_relative{
            Some([x, y]) => {
                direction_sceta_dig = (2.0 * PI) - (x.atan2(y));
                power = 0.8;// ball_dist / 100.0;
            },
            None => {
                // when ball_not found, return first pos(0.0, 0.0)
                /*
                direction_sceta_dig = (2.0 * PI) - ((-1.0 * machine_pos[0]).atan2((-1.0 * machine_pos[1])));
                power = (machine_pos[0].powi(2) + machine_pos[1].powi(2)).sqrt() / 10.0;
                if power >= 0.2 {
                    power = power.clamp(0.0, 0.6);
                } else {
                    power = 0.0;
                }
                */

                //power = 0.0;
            },
        }


        let robot_dir: u16 = 0; //controll

        let sensor_dir: u16 = 360 - *(from_sensor_dir.lock().unwrap()); //0~360, reverse
        let sensor_dir_diff: i16 = sensor_dir as i16 - robot_dir as i16;
        let mod_robot_dir: u16 = if sensor_dir_diff < 0 { (360 + sensor_dir_diff) as u16 } else { sensor_dir_diff as u16 }; //centelyzed by robot_dir

        //let direction_sceta_dig: f64 = (direction_sceta as f64) / (360 as f64) * 2.0 * PI;
        let direction_sceta_dig_x_y: [f64; 2] = [direction_sceta_dig.cos(), direction_sceta_dig.sin()];

        let mut motor1: f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[0][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[0][1];
        let mut motor2: f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[1][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[1][1];
        let mut motor3: f64 = direction_sceta_dig_x_y[0] * motor_dir_x_y[2][0] + direction_sceta_dig_x_y[1] * motor_dir_x_y[2][1];

        let max_num_rev: f64 = 1.0 / ( motor1.abs().max(motor2.abs()).max(motor3.abs()) );

        motor1 = motor1 * max_num_rev;
        motor2 = motor2 * max_num_rev;
        motor3 = motor3 * max_num_rev;
        
        motor1 = motor1 * power;
        motor2 = motor2 * power;
        motor3 = motor3 * power;
        
        //senosrs dir feedback
        let dir_diff_angle: f64 = if mod_robot_dir <= 180 { -1.0 * (mod_robot_dir as f64) } else { 360.0 - (mod_robot_dir as f64) }; //-180~180 //mod is centelyzed by robot_dir
        let mut dir_diff: f64 = dir_diff_angle / 180.0; // -1.0~1.0

        dir_diff = dir_diff.clamp(-0.5, 0.5);

        //PID
        let time_after_command: f64 = now.elapsed().as_secs_f64() - last_command_time;
        last_command_time = now.elapsed().as_secs_f64();
        //println!("time_after_command: {}", time_after_command);

        let dir_sensor_latency_val: f64= *(dir_sensor_latency.lock().unwrap());
        let ball_sensor_latency_val: f64 = *(ball_sensor_latency.lock().unwrap());
        let usb_mouse_latency_val: f64 = *(usb_mouse_latency.lock().unwrap());
        let calc_target_latency_val: f64 = *(calc_target_latency.lock().unwrap());

        //print max latency sum that all sensors;
        let max_latency: f64 = dir_sensor_latency_val + ball_sensor_latency_val + usb_mouse_latency_val + calc_target_latency_val + time_after_command;

        //println!("all max latency: {}, dir_sesnsor: {}, ball_sensor: {}, usb_mouse: {}, calc_target: {}, main_loop: {}",
        //        max_latency, dir_sensor_latency_val, ball_sensor_latency_val, usb_mouse_latency_val, calc_target_latency_val, time_after_command);

        cycle_num += 1;
        if cycle_num > 100 {
            cycle_num = 0;
            integral = 0.0;
        }

        error[0] = error[1];
        error[1] = dir_diff;
        integral += (error[0] + error[1])/2.0 * time_after_command;
        let delta_dir_motion: f64 = KP*error[1] + KI*integral + (KD*((error[1] - error[0]) / time_after_command));

        //println!("integral: {}", integral);
        //println!("delta_dir_motion: {}", delta_dir_motion);

        motor1 += delta_dir_motion;
        motor2 += delta_dir_motion;
        motor3 += delta_dir_motion;
        
        //log scale
        /*
        motor1 = if motor1 >= 0.0 { (motor1 + 1.0).log2() } else { (motor1.abs() + 1.0).log2() * -1.0 };
        motor2 = if motor2 >= 0.0 { (motor2 + 1.0).log2() } else { (motor2.abs() + 1.0).log2() * -1.0 };
        motor3 = if motor3 >= 0.0 { (motor3 + 1.0).log2() } else { (motor3.abs() + 1.0).log2() * -1.0 };
        */

        /*
        if preval[0] == 0.0 { motor1 += 0.05; }
        if preval[1] == 0.0 { motor2 += 0.05; }
        if preval[2] == 0.0 { motor3 += 0.05; }
        */

        // clamp
        motor1 = motor1.clamp(-1.0, 1.0);
        motor2 = motor2.clamp(-1.0, 1.0);
        motor3 = motor3.clamp(-1.0, 1.0);

        //save motor
        if motor1.abs() < 0.1 && motor2.abs() < 0.1 && motor3.abs() < 0.1 { motor1 = 0.0; motor2 = 0.0; motor3 = 0.0; }

        let motor: [i8; 3] = [(motor1*40.0) as i8, (motor2*40.0) as i8, (motor3*40.0) as i8]; //should -100 to 100
        let mut cmd_str =  String::from("1F000"); //unused motor channel 1
        let cmd = generate_cmd(&motor, &mut cmd_str);

        //println!("start motor(byte): {:?}", cmd);

        port.write(cmd).unwrap();
        port.flush().unwrap();

        //preval = [ motor1, motor2, motor3 ];
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
