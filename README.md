### Gazebo RFID Scanner Plugin

**Features**
- Service for adding and removing tags from the simulation.
- Models of RFID Antenna and RFID tags.
- Realistic RFID scanning model based on Friis free-space-path-loss (FSPL).

**RFID Model**
The model is based on the Friis transmission equation with FSPL. 
$$P_{in} = P_{tx} + G_{r}(\mathbf{\theta_{r}}, \mathbf{x_{r}}, \mathbf{x_{t}})+G_{t} - L_{path}(\mathbf{x_{r}}, \mathbf{x_{t}}) - L_{pol}(\mathbf{\theta_{r}}, \mathbf{\theta_{r}})$$
$$P_{rx} = P_{tx} + G_{r}(\mathbf{\theta_{r}}, \mathbf{x_{r}}, \mathbf{x_{t}})+G_{t} - 2(L_{path}(\mathbf{x_{r}}, \mathbf{x_{t}}) - L_{pol}(\mathbf{\theta_{r}}, \mathbf{\theta_{r}}))$$
$$\text{RSSI}=P_{rx}+\eta_{meas}, \eta_{meas} \sim N(0, \sigma_{rssi}^{2})$$

- $P_{tx}$ is transmit power (dBm). For our reader, the nominal value is 30dBm.
- $G_{r}$ is antenna gain (dBi) and represents changes in power from the direction that the antenna is facing (propogating the signal).
- $G_{t}$ is the tag gain (dBi).
- $L_{path}(d)$ represents power loss (dB) arising from the distance between the antenna and the tag.
- $L_{pol}$ represents power loss (dB) from difference in orientation between the antenna and the tag.
- $\theta$ is the tag and receiver/antenna orientations.
- $\mathbf{x}$ is the tag and receiver positions.

We sample from this distribution to determine if the tag is successfully read. 
$$P_{read,t}=\sigma\left(\frac{P_{in}-P_{in,\text{ offset}}}{k_{in}}\right)$$
$$P_{read,r}=\sigma\left(\frac{P_{rx}-P_{rx,\text{ offset}}}{k_{rx}}\right)$$

$$P_{read}=P_{read,t}\cdot P_{read,r}$$

![bsangle0](img/bsangle0-readprob.png)

**See Also**
- A detailed tutorial based on this plugin, available [here]().
- Usage of this RFID plugin in an actual Gazebo simulation environment, [here]().


