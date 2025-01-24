use crossbeam_channel::{unbounded, Receiver, Sender};
use std::{collections::HashMap, fs, thread};

use wg_2024::{
    config::{Config, Drone as ConfigDrone},
    controller::{DroneCommand, DroneEvent},
    drone::Drone,
    network::NodeId,
    packet::Packet,
};

use simulation_controller::SimulationController;

fn open(path: &str) -> Config {
    // Read content of file src/config.toml
    let config_data = fs::read_to_string(path).expect("Unable to read config file");
    // Parse previously created string
    toml::from_str(&config_data).expect("Unable to parse TOML")
}

fn create_factory<D>() -> Box<
    dyn Fn(
        &ConfigDrone,
        &Sender<DroneEvent>,
        &Receiver<DroneCommand>,
        &HashMap<NodeId, Sender<Packet>>,
        &HashMap<NodeId, Receiver<Packet>>,
    ) -> Box<dyn Drone>,
>
where
    D: Drone + 'static,
{
    Box::new(
        |drone, event_send, command_recv, all_packet_send, all_packet_recv| {
            let packet_recv = all_packet_recv.get(&drone.id).unwrap().clone();

            let mut packet_send = HashMap::<NodeId, Sender<Packet>>::new();

            for neighbor in &drone.connected_node_ids {
                all_packet_send
                    .iter()
                    .filter(|(node_id, _)| *node_id == neighbor)
                    .for_each(|(node_id, channel)| {
                        packet_send.insert(*node_id, channel.clone());
                    });
            }

            Box::new(D::new(
                drone.id,
                event_send.clone(),
                command_recv.clone(),
                packet_recv,
                packet_send,
                drone.pdr,
            ))
        },
    )
}

pub fn run() {
    // Open and read File
    let config = open("src/config.toml");

    if config.drone.len() < 10 {
        panic!("ERROR: At least 10 drones are required");
    }

    // Simulation controller channels
    let (command_send, command_recv) = unbounded::<DroneCommand>();
    let (event_send, event_recv) = unbounded::<DroneEvent>();

    // HashMap for packet channels
    let mut all_packet_send = HashMap::<NodeId, Sender<Packet>>::new();
    let mut all_packet_recv = HashMap::<NodeId, Receiver<Packet>>::new();

    for drone in &config.drone {
        let (pkt_send, pkt_recv) = unbounded::<Packet>();
        all_packet_send.insert(drone.id, pkt_send);
        all_packet_recv.insert(drone.id, pkt_recv);
    }

    // Vector of drones and hashmap for the simulation controller
    let mut drones: Vec<Box<dyn Drone>> = Vec::new();
    let mut drones_hashmap = HashMap::<NodeId, (Sender<DroneCommand>, Sender<Packet>)>::new();

    // Factory mapping (uses a common helper function)
    let drone_factories = vec![
        create_factory::<rusty_drones::RustyDrone>(),
        create_factory::<LeDron_James::Drone>(),
        create_factory::<dr_ones::Drone>(),
        create_factory::<skylink::SkyLinkDrone>(),
        create_factory::<rustbusters_drone::RustBustersDrone>(),
        create_factory::<rust_roveri::RustRoveri>(),
        create_factory::<rust_do_it::RustDoIt>(),
        create_factory::<wg_2024_rust::drone::RustDrone>(),
        create_factory::<null_pointer_drone::MyDrone>(),
        create_factory::<lockheedrustin_drone::LockheedRustin>(),
    ];

    // Generate drones using factories
    for (n, drone) in config.drone.iter().enumerate() {
        if let Some(factory) = drone_factories.get(n) {
            let new_drone = factory(
                drone,
                &event_send,
                &command_recv,
                &all_packet_send,
                &all_packet_recv,
            );

            // Add the new drone to the list and hashmap
            drones.push(new_drone);

            if let Some(pkt_send) = all_packet_send.get(&drone.id) {
                drones_hashmap.insert(drone.id, (command_send.clone(), pkt_send.clone()));
            } else {
                panic!("Packet sender not found for drone {}", drone.id);
            }
        } else {
            panic!("No factory defined for [ Drone {} ]", drone.id);
        }
    }

    // Create and initialize simulation controller
    let mut simulation_controller = SimulationController::new(drones_hashmap, event_recv);

    let controller_handle = thread::spawn(move || {
        simulation_controller.run();
    });

    let mut drone_handles = Vec::new();
    // Run drones
    for mut drone in drones.into_iter() {
        let handle = thread::spawn(move || {
            drone.run();
        });
        drone_handles.push(handle);
    }

    controller_handle.join().unwrap();

    for handle in drone_handles {
        handle.join().unwrap();
    }
}
