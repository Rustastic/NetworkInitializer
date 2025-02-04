//! This file contains the Rustastic Network Initializer
//!
//! File:   network_initializer.rs
//!
//! Brief:  Main file for the Rustastic Network Initializer, containing the necessary step to initialize the drones, simulation controller, server, client
//!
//! Author: Alessandro Busola

use colored::Colorize;
use crossbeam_channel::{unbounded, Receiver, Sender};
use log::info;
use std::{collections::HashMap, fs, thread};

use wg_2024::{
    config::{Config, Drone as ConfigDrone},
    controller::{DroneCommand, DroneEvent},
    drone::Drone,
    network::NodeId,
    packet::Packet,
};

use simulation_controller::SimulationController;
use gui::{
    commands::{GUICommands, GUIEvents},
    SimCtrlGUI,
};
use chat_client::ChatClient;
use media_client::MediaClient;
use messages::client_commands::{ChatClientCommand, ChatClientEvent, MediaClientCommand, MediaClientEvent};

/// Creates a factory function for generating drones of a specified type.
///
/// # Type Parameters
/// * `T` - The type of drone to be created. Must implement the `Drone` trait.
///
/// # Returns
/// * A boxed closure that takes the following arguments and returns a boxed drone:
///     - `ConfigDrone`: Configuration for the drone.
///     - `Sender<DroneEvent>`: Sender channel for drone events.
///     - `HashMap<NodeId, Receiver<DroneCommand>>`: Map of command receiver channels for each node.
///     - `HashMap<NodeId, Sender<Packet>>`: Map of packet sender channels for each node.
///     - `HashMap<NodeId, Receiver<Packet>>`: Map of packet receiver channels for each node.
///
/// # Panics
/// * If the required channels for a drone are not found in the provided hashmaps.
fn drone_factory<T>() -> Box<
    dyn Fn(
        &ConfigDrone,
        &Sender<DroneEvent>,
        &HashMap<NodeId, Receiver<DroneCommand>>,
        &HashMap<NodeId, Sender<Packet>>,
        &HashMap<NodeId, Receiver<Packet>>,
    ) -> Box<dyn Drone>,
>
where
    T: Drone + 'static,
{
    Box::new(
        |drone, event_send, command_recv_hashmap, packet_send_hashmap, packet_recv_hashmap| {
            // Get drone's packet receiver channel
            if let Some(packet_recv) = packet_recv_hashmap.get(&drone.id) {
                // Create packet send hashmap
                let mut packet_send = HashMap::<NodeId, Sender<Packet>>::new();
                // Fill hashmap with only neighbor
                for neighbor in &drone.connected_node_ids {
                    packet_send_hashmap
                        .iter()
                        .filter(|(node_id, _)| *node_id == neighbor)
                        .for_each(|(node_id, channel)| {
                            packet_send.insert(*node_id, channel.clone());
                        });
                }

                // Get drone's command receiver channel
                if let Some(command_recv) = command_recv_hashmap.get(&drone.id) {
                    return Box::new(T::new(
                        drone.id,
                        event_send.clone(),
                        command_recv.clone(),
                        packet_recv.clone(),
                        packet_send,
                        drone.pdr,
                    ));
                } else {
                    panic!("Command receiver not found for drone {}", drone.id);
                }
            } else {
                panic!("Packet receiver not found for drone {}", drone.id);
            }
        },
    )
}

/// Opens a configuration file and parses it into a `Config` object.
///
/// # Arguments
/// * `path` - A string slice representing the path to the configuration file.
///
/// # Returns
/// * `Config` - The parsed configuration object.
///
/// # Panics
/// * If the file cannot be read.
/// * If the TOML data cannot be parsed.
fn open(path: &str) -> Config {
    // Read content of file src/config.toml
    let config_data = fs::read_to_string(path).expect("Unable to read config file");
    // Parse previously created string
    toml::from_str(&config_data).expect("Unable to parse TOML")
}

/// Initializes and runs the drone simulation.
///
/// This function performs the following steps:
/// 1. Opens and parses the configuration file.
/// 2. Creates communication channels for events, packets, and commands.
/// 3. Instantiates drones using factory functions.
/// 4. Creates a simulation controller and initializes it with drones and neighbors.
/// 5. Runs the simulation controller and drones on separate threads.
/// 6. Joins all threads to ensure proper execution.
///
/// # Panics
/// * If the configuration contains fewer than 10 drones.
/// * If required channels or factories are missing during drone initialization.
pub fn run() {
    info!(
        "[ {} ] Starting Network Initializer",
        "Network Initializer".green()
    );
    // Open and read File
    let config = open("src/config.toml");

    // Event channels
    let (event_send, event_recv) = unbounded::<DroneEvent>();

    // Packet channels
    let mut packet_send = HashMap::<NodeId, Sender<Packet>>::new();
    let mut packet_recv = HashMap::<NodeId, Receiver<Packet>>::new();

    // Command channels
    let mut command_send = HashMap::<NodeId, Sender<DroneCommand>>::new();
    let mut command_recv = HashMap::<NodeId, Receiver<DroneCommand>>::new();

    // Create channels
    for drone in &config.drone {
        let id = drone.id;

        // Packet channel
        let (pkt_send, pkt_recv) = unbounded::<Packet>();
        packet_send.insert(id, pkt_send);
        packet_recv.insert(id, pkt_recv);

        // Command channel
        let (cmd_send, cmd_recv) = unbounded::<DroneCommand>();
        command_send.insert(id, cmd_send);
        command_recv.insert(id, cmd_recv);
    }

    // Vector of drones
    let mut drones: Vec<Box<dyn Drone>> = Vec::new();

    // Hashmap of sender channel of drones
    let mut drones_hashmap = HashMap::<NodeId, (Sender<DroneCommand>, Sender<Packet>)>::new();

    // Create vector containing all the drones' function
    let drone_factories = vec![
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
    ];

    info!("[ {} ] Creating Drones", "Network Initializer".green());
    // Generate drones using factories
    for (n, drone) in config.drone.iter().enumerate() {
        // Get right function
        if let Some(factory) = drone_factories.get(n) {
            // create drone
            let new_drone = factory(
                drone,
                &event_send,
                &command_recv,
                &packet_send,
                &packet_recv,
            );

            // Add the new drone to the list and hashmap
            drones.push(new_drone);

            // Fill drone Hashmap
            if let Some(pkt_send) = packet_send.get(&drone.id) {
                if let Some(cmd_send) = command_send.get(&drone.id) {
                    drones_hashmap.insert(drone.id, (cmd_send.clone(), pkt_send.clone()));
                } else {
                    panic!("Command sender not found for drone {}", drone.id);
                }
            } else {
                panic!("Packet sender not found for drone {}", drone.id);
            }
        } else {
            panic!("No factory defined for [ Drone {} ]", drone.id);
        }
    }

    // Create an hashmap containing neighbors
    let mut neighbor = HashMap::<NodeId, Vec<NodeId>>::new();

    // Fill the hashmap
    for drone in &config.drone {
        neighbor.insert(drone.id, drone.connected_node_ids.clone());
    }

    // Client
    let half = config.client.len() / 2; // number of chat clients
    let mut count = 0;

    info!(
        "[ {} ] Creating ChatClient and MediaClient",
        "Network Initializer".green()
    );
    // Create vectors to save clients
    let chat_clients = Vec::<ChatClient>::new();
    let media_clients = Vec::<MediaClient>::new();

    // create hashmap to pass to clients
    let cclient_send = HashMap::<NodeId, Sender<ChatClientCommand>>::new();
    let mclient_send = HashMap::<NodeId, Sender<MediaClientCommand>>::new();

    // generate
    for client in &config.client {
        if count <= half {
            // Create ChatClient channels
            let (cclient_command_send, cclient_command_recv) = unbounded::<ChatClientCommand>();
            let (cclient_event_send, cclient_event_recv) = unbounded::<ChatClientEvent>();

            // Create ChatClient
            let cclient = ChatClient::new(
                client.id,
                cclient_event_send.clone(),
                cclient_command_recv,
                packet_recv,
                packet_send,
            );

            // Init ChatClient
            cclient_command_send.send(ChatClientCommand::StartChatClient);

            // Add to structures
            chat_clients.push(cclient);
            cclient_send.insert(client.id, cclient_command_send);
        } else {
            // Create MediaClient channels
            let (mclient_command_send, mclient_command_recv) = unbounded::<MediaClientCommand>();
            let (mclient_event_send, mclient_event_recv) = unbounded::<MediaClientEvent>();

            // Create MediaClient
            let mclient = MediaClient::new(
                client.id,
                mclient_event_send.clone(),
                mclient_command_recv,
                packet_recv,
                packet_send,
            );

            // Add to structures
            chat_clients.push(mclient);
            mclient_send.insert(client.id, mclient_command_send);
        }
        // Add client to neighbor vec
        neighbor.insert(client.id, client.connected_drone_ids.clone());
        
        count += 1;
    } 

    // Create Channels for the GUI
    let (gui_command_send, gui_command_recv) = unbounded::<GUICommands>();
    let (gui_event_send, gui_event_recv) = unbounded::<GUIEvents>();
    let gui_send = gui_event_send.clone();

    // Create and initialize the Simulation Controller
    info!(
        "[ {} ] Creating Simulation Controller",
        "Network Initializer".green()
    );
    let mut simulation_controller = SimulationController::new(
        drones_hashmap,
        event_recv,
        neighbor,
        event_send,
        gui_event_send,
        gui_command_recv,
        cclient_send,
        cclient_event_recv,
        mclient_send,
        mclient_command_recv,
    );

    // Run simulation controller on different tread
    let controller_handle = thread::spawn(move || {
        simulation_controller.run();
    });

    let mut drone_handles = Vec::new();
    // Run drones on different threads
    for mut drone in drones.into_iter() {
        let handle = thread::spawn(move || {
            drone.run();
        });
        drone_handles.push(handle);
    }

    let mut cclient_handles = Vec::new();
    // Run chat clients on different threads
    for mut client in chat_clients.into_iter() {
        let handle = thread::spawn(move || {
            client.run();
        });
        cclient_handles.push(handle);
    }

    let mut mclient_handles = Vec::new();
    // Run media client on different threads
    for mut mclient in media_clients.into_iter() {
        let handle = thread::spawn(move || {
            mclient.run();
        });
        mclient_handles.push(handle);
    }

    // launch GUI
    info!("[ {} ] Creating GUI", "Network Initializer".green());
    let gui = SimCtrlGUI::new(gui_command_send, gui_event_recv);
    gui_send.send(GUIEvents::Topology(config.drone)).unwrap();

    let options = eframe::NativeOptions::default();
    let _ = eframe::run_native(
        "Drone Simulation GUI",
        options,
        Box::new(|_cc| Ok(Box::new(gui))),
    );

    // Join all threads
    controller_handle.join().unwrap();

    for handle in drone_handles {
        handle.join().unwrap();
    }

    for handle in cclient_handles {
        handle.join().unwrap();
    }

    for handle in mclient_handles {
        handle.join().unwrap();
    }
}
