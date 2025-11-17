# BotWave Custom

A custom FM-RDS transmitter using the Raspberry Pi's PWM, based on [PiFmRds](https://github.com/ChristopheJacquet/PiFmRds)
  
This software has been made to suit the needs of [BotWave](https://github.com/dpipstudio/botwave).

> [!DANGER]
> Running this software without knowledge can lead in a broken system.
> We do not accept any liability or responsibility for anything that happens as a result of you running this code. If it breaks, you get to keep all the pieces

> [!WARNING]
> Running this may be illegal in your country.

---

This program generates an FM modulation, with RDS (Radio Data System) data generated in real time. It can include monophonic or stereophonic audio.

## Installation & Usage

First, install required packages and clone the repository

```bash
sudo apt install libsndfile1-dev build-essential git -y
git clone https://github.com/dpipstudio/bw_custom.git
cd bw_custom/src/
```

Then, build `bw_custom`: `make`.

This should have built an executable named `bw_custom`

Run `bw_custom -h` to know any argument you need.

---
For more informations, please reffer to [this README](https://github.com/ChristopheJacquet/PiFmRds/blob/master/README.md)

## License
As for the original code, this code is released under the GPL. See [LICENSE](LICENSE)

![madebydouxx](https://madeby.douxx.tech)
![adpipstudioproject](https://madeby.dpip.lol)