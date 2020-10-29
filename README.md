In this folder, the Arduino sketches used in all the set-ups used in our experiment may be found.
Below is a basic description of the general purpose of each of these scripts.
Information on how to connect the corresponding circuits, as well as the purpose of these cicuits in more detail, including expected outcomes, may be found in the [reproduction guide](https://git.science.uu.nl/ued2020/experiment-design-2020/-/blob/master/projects/Hardware_Johanna_Floris_Frank/Documentation/reproduction_guide.md).

**digital_master and digital_slave**

Simple scripts for I2C communication between a master and slave Arduino via the digital pins.

**master_reader and slave_reader**

Simple scripts for I2C communication between a master and slave Arduino via the analog pins.

**BME**

BME.ino can be uploaded to a master Arduino connected to a single (BME) sensor and possibly a slave Arduino. The master Arduino then reads the sensor output and, if present, the text-based message the slave sends directly. Should you want to use the second Arduino, the slave_reader.ino sketch should be uploaded to the slave device.

**Hybrid_Master and Hybrid_Slave**

Hybrid_Master.ino can be uploaded to an Arduino, which then acts as the master Arduino requesting data from a subscriber (slave) Arduino and shows this data. Hybrid_Slave.ino can be uploaded to Arduinos connected to (BME) sensors. This subscriber then reads the sensor values and give the data to the master upon request. 

**Hybrid_Master_MultiSlave, Hybrid_Slave and Hybrid_Slave_Clock**

Generalization of the previously described set-up. Hybrid_Master_MultiSlave may be uploaded to an Arduino, which then acts as the master Arduino requesting data from multiple subscriber (slave) Arduinos and shows this data. Hybrid_Slave.ino can be uploaded to an Arduino connected to (BME) sensors, Hybrid_Slave_Clock.ino can be uploaded to an Arduino connected to an MH DS3231 external clock. These subscribers then read the corresponding sensor outputs and transfer the data to the master upon request. 