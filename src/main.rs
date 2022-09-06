use serialport;
use std::time::Duration;
use std::thread::sleep;

fn main() {
    /*
    let mut port = serialport::new("/dev/tty1", 115200)
        .stop_bits(serialport::StopBits::One)
        .data_bits(serialport::DataBits::Eight)
        .timeout(Duration::from_millis(10))
        .open()
        .unwrap_or_else(|e| {
            eprintln!("Failed to open \"{}\". Error: {}", "/dev/tty1", e);
            ::std::process::exit(1);
    });
    */

    let motor :[i8; 3] = [20, 20, 20]; //should -100 to 100

    let mut cmd_str = String::from("1F000"); //unused motor channel 1

    for i in 0..3
    {
        cmd_str += (i + 2).to_string().as_ref();
        if motor[i] >= 0 { //at 0 meaning not force stop command here
            cmd_str += "F";
        } else {
            cmd_str += "R";
        }

        let param: u8 = motor[i].abs() as u8;
        assert_eq!(param <= 100, true); // should remove at release

        let param_str: String = format!("{:>03}", param);
        cmd_str += param_str.as_str();

    }

    cmd_str.push_str("\n");
    let cmd = cmd_str.as_bytes();
    println!("start motor: {}", cmd_str);
    println!("start motor(byte): {:?}", cmd);
    /*
    match port.write(cmd)
    {
        Ok(_) => println!("start motor: {:?}", cmd_str),
        Err(e) => eprintln!("{:?}", e),
    }

    port.flush().unwrap();

    sleep(Duration::from_secs(10));

    let poweroff_all_motor: &[u8; 21] = b"1F0002F0003F0004F000\n";
    //let force_stop_motor: &[u8; 21] = b"1R0002R0003R0004R000\n";
    match port.write(poweroff_all_motor)
    {
        Ok(_) => println!("poweroff motor"),
        Err(e) => eprintln!("{:?}", e),
    }

    port.flush().unwrap();
    */
}
