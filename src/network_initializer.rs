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

use chat_client::ChatClient;
use communication_server::{communication_server::CommunicationServer, server::Server};
use gui::{
    commands::{GUICommands, GUIEvents},
    SimCtrlGUI,
};
use media_client::media_client::MediaClient;
use messages::{
    client_commands::{ChatClientCommand, ChatClientEvent, MediaClientCommand, MediaClientEvent},
    server_commands::{CommunicationServerCommand, CommunicationServerEvent},
};
use simulation_controller::SimulationController;

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

    // Create a vector to save servers
    let mut communication_servers = Vec::<CommunicationServer>::new();
    let mut comm_server_send =
        HashMap::<NodeId, (Sender<CommunicationServerCommand>, Sender<Packet>)>::new();
    let mut comm_server_recv = HashMap::<NodeId, Receiver<CommunicationServerEvent>>::new();

    let (comm_server_event_send, comm_server_event_recv) = unbounded::<CommunicationServerEvent>();

    let third = config.server.len() / 3;
    let mut count = config.server.len();
    for server in &config.server {
        /*if count > (third * 2) {
            // content-text
        } else if count > third {
            // content-media
        } else {*/
        let (comm_server_command_send, comm_server_command_recv) =
            unbounded::<CommunicationServerCommand>();
        let (pkt_send, pkt_recv) = unbounded::<Packet>();

        packet_send.insert(server.id, pkt_send.clone());
        packet_recv.insert(server.id, pkt_recv);

        comm_server_recv.insert(server.id, comm_server_event_recv.clone());
        comm_server_send.insert(server.id, (comm_server_command_send, pkt_send));
        //}

        count += 1;
    }

    // Create vectors to save clients
    let mut chat_clients = Vec::<ChatClient>::new();
    let mut media_clients = Vec::<MediaClient>::new();

    // create hashmap to pass to clients
    let mut cclient_send = HashMap::<NodeId, (Sender<ChatClientCommand>, Sender<Packet>)>::new();
    let mut cclient_recv = HashMap::<NodeId, Receiver<ChatClientCommand>>::new();
    let mut mclient_send = HashMap::<NodeId, (Sender<MediaClientCommand>, Sender<Packet>)>::new();
    let mut mclient_recv = HashMap::<NodeId, Receiver<MediaClientCommand>>::new();

    let (cclient_event_send, cclient_event_recv) = unbounded::<ChatClientEvent>();
    let (mclient_event_send, mclient_event_recv) = unbounded::<MediaClientEvent>();

    let half = config.client.len() / 2; // Number of chat clients
    count = 0;
    for client in &config.client {
        if count < half {
            let (cclient_command_send, cclient_command_recv) = unbounded::<ChatClientCommand>();
            let (pkt_send, pkt_recv) = unbounded::<Packet>();

            packet_send.insert(client.id, pkt_send.clone());
            packet_recv.insert(client.id, pkt_recv);

            cclient_recv.insert(client.id, cclient_command_recv);
            cclient_send.insert(client.id, (cclient_command_send, pkt_send));
        } else {
            let (mclient_command_send, mclient_command_recv) = unbounded::<MediaClientCommand>();
            let (pkt_send, pkt_recv) = unbounded::<Packet>();

            packet_send.insert(client.id, pkt_send.clone());
            packet_recv.insert(client.id, pkt_recv);

            mclient_recv.insert(client.id, mclient_command_recv);
            mclient_send.insert(client.id, (mclient_command_send, pkt_send));
        }

        count += 1;
    }

    // Vector of drones
    let mut drones: Vec<Box<dyn Drone>> = Vec::new();

    // Hashmap of sender channel of drones
    let mut drones_hashmap = HashMap::<NodeId, (Sender<DroneCommand>, Sender<Packet>)>::new();

    // Create vector containing all the drones' function
    let drone_factories = vec![
        /*drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),*/

        /* rusty_drones: OK
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<rusty_drones::RustyDrone>(),
        drone_factory::<rusty_drones::RustyDrone>(),*/

        /* LeDron_James: Subtracts with overflow
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<LeDron_James::Drone>(),
        drone_factory::<LeDron_James::Drone>(),
        */

        /* dr_ones: Unwrap a None value
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<dr_ones::Drone>(),
        drone_factory::<dr_ones::Drone>(),*/

        /* skylink: Infinite Loop of FloodResponse
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<skylink::SkyLinkDrone>(),
        drone_factory::<skylink::SkyLinkDrone>(),*/

        /* rustbusters_drone: OK
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),
        drone_factory::<rustbusters_drone::RustBustersDrone>(),*/

        /* rust_roveri: OK*/
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        drone_factory::<rust_roveri::RustRoveri>(),
        /* rust_do_it: Loops infinite Nack saying destination is Drone
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        drone_factory::<rust_do_it::RustDoIt>(),
        */

        /* RustTheGroup: Panics because no path_trace
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),
        drone_factory::<wg_2024_rust::drone::RustDrone>(),*/


        /* null_pointer_drone: Panics because no path_trace
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),
        drone_factory::<null_pointer_drone::MyDrone>(),*/

        /* lockheedrustin_drone: Infinite FloodResponse
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),
        drone_factory::<lockheedrustin_drone::LockheedRustin>(),*/
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
    info!(
        "[ {} ] Creating ChatClient and MediaClient",
        "Network Initializer".green()
    );

    // Generate clients
    count = 0;
    for client in &config.client {
        let mut cpkt_send = HashMap::<u8, Sender<Packet>>::new();
        for neighbor in client.connected_drone_ids.iter() {
            cpkt_send.insert(*neighbor, packet_send.get(neighbor).unwrap().clone());
        }

        if count < half {
            // Create ChatClient
            let cclient = ChatClient::new(
                client.id,
                cclient_event_send.clone(),
                cclient_recv.get(&client.id).unwrap().clone(),
                packet_recv.get(&client.id).unwrap().clone(),
                cpkt_send,
            );

            // Add to structures
            chat_clients.push(cclient);
        } else {
            // Create MediaClient
            let mclient = MediaClient::new(
                client.id,
                mclient_event_send.clone(),
                mclient_recv.get(&client.id).unwrap().clone(),
                packet_recv.get(&client.id).unwrap().clone(),
                cpkt_send,
            );

            // Add to structures
            media_clients.push(mclient);
        }
        // Add client to neighbor vec
        neighbor.insert(client.id, client.connected_drone_ids.clone());

        count += 1;
    }

    // Server
    info!(
        "[ {} ] Creating Communication and Content Servers",
        "Network Initializer".green()
    );

    count = 0;
    for server in &config.server {
        let mut spkt_send = HashMap::<u8, Sender<Packet>>::new();
        for neighbor in server.connected_drone_ids.iter() {
            spkt_send.insert(*neighbor, packet_send.get(neighbor).unwrap().clone());
        }

        /*if count > (third * 2) {
            // content-text
        } else if count > third {
            // content-media
        } else {*/
        let comm_server = CommunicationServer::new(
            server.id,
            packet_recv.get(&server.id).unwrap().clone(),
            spkt_send,
            comm_server_event_send.clone(),
            comm_server_recv.get(&server.id).unwrap().clone(),
        );

        communication_servers.push(comm_server);
        //}
        neighbor.insert(server.id, server.connected_drone_ids.clone());

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
        mclient_event_recv,
        comm_server_send,
        comm_server_event_recv,
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

    let mut comm_server_handles = Vec::new();
    // Run Servers
    for mut server in communication_servers.into_iter() {
        let handle = thread::spawn(move || {
            server.run();
        });
        comm_server_handles.push(handle);
    }

    // Run GUI on main thread
    info!("[ {} ] Creating GUI", "Network Initializer".green());
    let gui = SimCtrlGUI::new(gui_command_send, gui_event_recv);
    gui_send
        .send(GUIEvents::Topology(
            config.drone,
            config.client,
            config.server,
        ))
        .unwrap();

    let options = eframe::NativeOptions::default();
    let _ = eframe::run_native(
        "Simulation Controller GUI",
        options,
        Box::new(|_cc| Ok(Box::new(gui))),
    );

    // Join all threads
    for handle in drone_handles {
        handle.join().unwrap();
    }

    for handle in cclient_handles {
        handle.join().unwrap();
    }

    for handle in mclient_handles {
        handle.join().unwrap();
    }

    for handle in comm_server_handles {
        handle.join().unwrap();
    }

    controller_handle.join().unwrap();
}
