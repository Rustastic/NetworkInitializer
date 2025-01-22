use crossbeam_channel::{unbounded, Sender};
use std::{collections::HashMap, fs};

use wg_2024::{
    config::{Config, Drone as ConfigDrone},
    controller::{DroneCommand, DroneEvent},
    drone::Drone,
    network::NodeId,
    packet::Packet,
};

fn open(path: &str) -> Config {
    // Read content of file src/config.toml
    let config_data = fs::read_to_string(path).expect("Unable to read config file");
    // Parse previously created string
    toml::from_str(&config_data).expect("Unable to parse TOML")
}

pub fn run() {
    // Open and read File
    let config = open("src/config.toml");

    if config.drone.len() < 10 {
        panic!("ERROR");
    }

    // simulation controller channels
    let (_, controller_recv) = unbounded::<DroneCommand>();
    let (controller_send, _) = unbounded::<DroneEvent>();

    // hashmap sender channels
    let mut packet_send = HashMap::<NodeId, Sender<Packet>>::new();

    // loop to fill packet_send
    for drone in &config.drone {
        let (pkt_send, _) = unbounded::<Packet>();
        packet_send.insert(drone.id, pkt_send);
    }

    // vector of drones
    let mut drones: Vec<Box<dyn Drone>> = Vec::new();

    //
    let drone_factories: Vec<Box<dyn Fn(&ConfigDrone) -> Box<dyn Drone>>> = vec![
        Box::new(|drone| {
            Box::new(rusty_drones::RustyDrone::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
        Box::new(|drone| {
            Box::new(LeDron_James::Drone::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
        Box::new(|drone| {
            Box::new(dr_ones::Drone::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
        Box::new(|drone| {
            Box::new(skylink::SkyLinkDrone::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
        Box::new(|drone| {
            Box::new(rustbusters_drone::RustBustersDrone::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
        Box::new(|drone| {
            Box::new(rust_roveri::RustRoveri::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
        Box::new(|drone| {
            Box::new(rust_do_it::RustDoIt::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
        Box::new(|drone| {
            Box::new(wg_2024_rust::drone::RustDrone::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
        Box::new(|drone| {
            Box::new(null_pointer_drone::MyDrone::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
        Box::new(|drone| {
            Box::new(lockheedrustin_drone::LockheedRustin::new(
                drone.id,
                controller_send.clone(),
                controller_recv.clone(),
                unbounded::<Packet>().1,
                packet_send.clone(),
                drone.pdr,
            ))
        }),
    ];

    for (n, drone) in config.drone.iter().enumerate() {
        if let Some(factory) = drone_factories.get(n) {
            drones.push(factory(drone));
        } else {
            panic!("No factory defined for drone index {}", n);
        }
    }

    for drone in &mut drones {
        drone.run()
    }
}
