use serialport;
use std::time::Duration;
use std::thread::sleep;

fn main() {
    let mut port = serialport::new("/dev/ttyS1", 115200)
        .stop_bits(serialport::StopBits::One)
        .data_bits(serialport::DataBits::Eight)
        .timeout(Duration::from_millis(10))
        .open()
        .unwrap_or_else(|e| {
            eprintln!("Failed to open \"{}\". Error: {}", "/dev/ttyAMA0", e);
            ::std::process::exit(1);
    });

    let cmd: &[u8; 21] = b"1F0202F0203F0204F020\n";

    match port.write(cmd)
    {
        Ok(_) => println!("start motor"),
        Err(e) => eprintln!("{:?}", e),
    }

    port.flush();

    sleep(Duration::from_secs(10));

    let poweroff_all_motor: &[u8; 21] = b"1F0002F0003F0004F000\n";
    //let force_stop_motor: &[u8; 21] = b"1R0002R0003R0004R000\n";
    match port.write(poweroff_all_motor)
    {
        Ok(_) => println!("poweroff motor"),
        Err(e) => eprintln!("{:?}", e),
    }

    port.flush();
}
