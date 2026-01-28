# Gazebo RFID Scanner Plugin
Implementation of an RFID scanner in Gazebo, with tag and antenna models.

**Features**
- Service for adding and removing tags from the simulation.
- Models of RFID Antenna and RFID tags.
- Realistic RFID scanning model based on Friis free-space-path-loss (FSPL).

**RFID Model**
The model is based on the Friis transmission equation with FSPL. 


$$P_{in} = P_{tx} + G_{r}(\mathbf{\theta_{r}}, \mathbf{x_{r}}, \mathbf{x_{t}})+G_{t} - L_{path}(\mathbf{x_{r}}, \mathbf{x_{t}}) - L_{pol}(\mathbf{\theta_{r}}, \mathbf{\theta_{r}})$$
$$P_{rx} = P_{tx} + G_{r}(\mathbf{\theta_{r}}, \mathbf{x_{r}}, \mathbf{x_{t}})+G_{t} - 2(L_{path}(\mathbf{x_{r}}, \mathbf{x_{t}}) - L_{pol}(\mathbf{\theta_{r}}, \mathbf{\theta_{r}}))$$
$$\text{RSSI}=P_{rx}+\eta_{meas}, \eta_{meas} \sim N(0, \sigma_{rssi}^{2})$$

- $P_{tx}$ is transmit power (dBm).
- $G_{r}$ and $G_{t}$ are antenna and tag gains (dBi).
- $L_{path}(d)$ and $L_{pol}$ are power loss (dB) from difference in distance and polarization between tag and antenna.

We sample from this distribution to determine if the tag is successfully read. 

$$P_{read,t}=\sigma\left(\frac{P_{in}-P_{in,\text{ offset}}}{k_{in}}\right)$$
$$P_{read,r}=\sigma\left(\frac{P_{rx}-P_{rx,\text{ offset}}}{k_{rx}}\right)$$

$$P_{read}=P_{read,t}\cdot P_{read,r}$$

<img src="img/bsangle0-readprob.png" width="300" height="200">

**See Also**
- A detailed tutorial based on this plugin, available [here]().
- Usage of this RFID plugin in an actual Gazebo simulation environment, [here]().


