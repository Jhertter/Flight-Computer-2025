# SAM M8Q
Information gathered from *SAM M8Q* datasheet, *u-blox protocol manual* and some usefull sites. 

## Collaborators 
- [Ezequiel Diaz Guzman](ediazguzman@itba.edu.ar)
- [José Iván Hertter](jhertter@itba.edu.ar)

## Max Navigation Update Rate

| GPS & GLONASS | GPS  | GLONASS |
| ------------- | ---- | ------- |
| 10Hz          | 18Hz | 18Hz    |

- Rates with SBAS and QZSS enabled for > 98% fix report rate under typical conditions

### Augmentation Systems

> SBAS and QZSS are satellite systems that enhance GNSS (Global Navigation Satellite System) performance. So, enabling SBAS and QZSS helps the u-blox achieve higher update rates and more reliable fixes by improving positioning conditions.

By default, the M8 receivers are configured for concurrent GPS and GLONASS, including SBAS and QZSS reception. If power consumption is a key factor, then the receiver should be configured for a single GNSS operation using GPS, Galileo or GLONASS and disabling QZSS and SBAS. The module can also be configured to receive any single GNSS constellation

#### SBAS (Satellite-Based Augmentation System)

These systems supplement GNSS data with additional regional or wide area GPS augmentation data.

Regional satellites:
- USA $\to$ WAAS
- Europe $\to$ EGNOS 
- MSAS $\to$ Japan 
- GAGAN $\to$ India
 
#### QZSS (Quasi-Zenith Satellite System)

The Quasi-Zenith Satellite System (QZSS) is a regional navigation satellite system that transmits additional GPS L1 C/A signals for the Pacific region covering **Japan** and **Australia**.