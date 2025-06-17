# NetworkInitializer

## Overview
The **NetworkInitializer** is the entry point of the simulation for the drone-based network developed by the **Rustastic** group.

## Role in the Simulation
The **NetworkInitializer** is responsible for setting up and launching the entire simulation environment. It reads the network configuration from a TOML file, constructs the network topology, and initializes all required components including drones, clients, servers, and the GUI.

### Responsibilities
* **Configuration Parsing**: Loads the network configuration from a TOML file. This includes information such as:

  * Node type (drone, client, server)
  * Unique identifiers
  * Connectivity (neighbors)
* **Network Construction**: Builds the network graph based on the parsed configuration, defining how nodes are connected.
* **Component Initialization**: Starts all nodes in the simulation:

  * Launches drone, client, and server instances
  * Boots up the GUI for interactive control
* **Simulation Bootstrap**: Triggers the simulation by initializing the necessary runtime environment and passing control to the Simulation Controller.

### Features
* Reads structured network configuration from TOML files.
* Dynamically builds and deploys the full network topology.
* Launches all core components of the simulation automatically.
* Serves as the single entry point for running the simulation.
