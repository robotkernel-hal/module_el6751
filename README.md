# module_el6751

**Robotkernel handler module for the Beckhoff EL6751 EtherCAT-to-CANopen Terminal**

This module provides a high-level interface for the [**Beckhoff EL6751 EtherCAT Terminal**](https://www.beckhoff.de/default.asp?ethercat/el6751.htm), enabling integration of CAN and CANopen devices within an EtherCAT-based automation system. It is designed for use with the [Robotkernel HAL](https://github.com/robotkernel-hal) framework.

The EL6751 operates as a **CAN/CANopen master**, making it possible to connect arbitrary CANopen slaves or exchange raw CAN frames without requiring the user to handle low-level frame details.

## ✨ Features

- Supports both **CANopen** and **raw CAN** message handling
- Fully integrated with the robotkernel processing model
- Provides seamless communication with CAN/CANopen slave modules
- Automatic process data handling via input/output triggers
- Modular design — easily extendable for custom CAN handlers

## 🧩 Configuration

Below is an example configuration snippet used to instantiate and integrate the `module_el6751` handler:

```yaml
name: my_el6751_terminal
so_file: libmodule_el6751.so
config:
  pd_inputs_device: ecat.slave_2.inputs.pd      # EL6751 process data inputs
  pd_outputs_device: ecat.slave_2.outputs.pd    # EL6751 process data outputs
  slave_modules: [ module_1, module_2 ]         # CAN slave module names
depends: [ ecat, module_1, module_2 ]           # Declare dependencies
```

### Parameters

| Key                | Description                                                                 |
|---------------------|-----------------------------------------------------------------------------|
| `pd_inputs_device` | Path to the process data input device of the EL6751 EtherCAT slave           |
| `pd_outputs_device`| Path to the process data output device of the EL6751 EtherCAT slave          |
| `slave_modules`    | List of CAN/CANopen handler module names (e.g., slave devices)              |

## ⚙️ Runtime Behavior

- The module registers callback routines that are automatically invoked when new process data is written to `pd_inputs_device`.
- After processing the incoming data, the module triggers updates to `pd_outputs_device`.
- Internally, it dispatches received CAN frames to registered slave modules and collects outgoing messages from them.

This design allows for **modular extension of CAN/CANopen logic**, where each slave device is represented by a dedicated module (e.g., `module_cia402` for a CiA 402 device).

## 🔗 Dependencies

- e.g. `module_ethercat` module (EtherCAT master interface)
- One or more CAN/CANopen slave handler modules (e.g., `module_cia402`, `module_dummy_slave`, etc.)

## 📦 Build & Installation

Please make sure that the prerequisites are installed. These are:
- robotkernel

Then you should be able to build module_el6751 with:

```bash
git clone https://github.com/robotkernel-hal/module_el6751.git
cd module_el6751
./bootstrap.sh
mkdir build && cd build
../configure
make
sudo make install
```

## 📚 Related Modules

- [`module_ethercat`](https://github.com/robotkernel-hal/module_ethercat): EtherCAT master interface
- `module_cia402`: CANopen slave module for motion control
- `module_dummy_slave`: Simple reference module for CANopen simulation

## 🧪 Testing

Ensure that the EtherCAT master and CANopen slave modules are correctly mocked or simulated if hardware is not available.

## 🤝 Contributing

Issues and pull requests are welcome. Please ensure:

- Your code builds cleanly 
- You adhere to robotkernel coding conventions (clang-format)
- Tests are provided for new functionality

## 📄 License

This project is licensed under the [LGPL-V3 License](LICENSE).

---

**Robotkernel HAL Project** – Real-time robotics infrastructure powered by modular, modern C++
