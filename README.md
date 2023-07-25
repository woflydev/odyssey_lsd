# Project Odyssey 2023: REDEMPTION AT LAST! (Odyssey LSD)

Our hard work finally paid off, as we were crowned the **Best Overall** and **1st Place** champions of the Droid Racing Challenge 2023! Thank you so much to the [QUT Robotics Club](https://www.instagram.com/p/Cu9Vgmuro8u/?img_index=1) for hosting such an awesome event and for allowing us to attend!
- [Instagram](https://www.instagram.com/p/Cu8mdIEtegS/?img_index=1)
- [Twitter](https://twitter.com/QUT/status/1679271733004578818)

For a video demonstration of the robot running with our code, please visit here: [Google Photos](https://photos.app.goo.gl/apWUW5tePbo4QnF27)

![team photo](https://github.com/woflydev/odyssey_cnn/blob/main/readme/team.jpg)

## Sister repositories
Check ``odyssey_cnn`` for more detailed information about us and the team!
| Repo | Description |
| ---- | --- |
| [woflydev/odyssey_cnn](https://github.com/woflydev/odyssey_cnn) | Main root repository for the Odyssey project. |
| [woflydev/odyssey_nnn](https://github.com/woflydev/odyssey_nnn) | New Neural Network implementation for the Odyssey project. |
| [woflydev/odyssey_data](https://github.com/woflydev/odyssey_data) | Unity simulation to generate virutal road scenes to train AI |
| [woflydev/odyssey_img](https://github.com/woflydev/odyssey_img) | Data exported from woflydev/odyssey_data |
| [woflydev/odyssey_docs](https://github.com/woflydev/odyssey_docs) | Upcoming documentation for Project Odyssey files and regular progress updates. |

## Installation

> **⚠ Deprecated API Documentation**
> ---
> If you plan on using our API and code, beware that:
> - some documentation below is outdated
> - we do not plan on updating the documentation here
> - instead, check [woflydev/odyssey_docs](https://github.com/woflydev/odyssey_docs) for future reference.

Git (recommended):
```bash
git clone https://github.com/woflydev/odyssey_lsd.git
cd odyssey_lsd
pip install -r requirements.txt
sudo chmod +x permissions.sh     # required for Arduino port access
```

GitHub CLI:
```bash
gh repo clone woflydev/odyssey_lsd
cd odyssey_cnn
pip install -r requirements.txt
sudo chmod +x permissions.sh     # required for Arduino port access
```

## Usage / Examples

Please note that it is recommended to install the [SSH Extension for Visual Studio Code](https://code.visualstudio.com/docs/remote/ssh-tutorial) so you can have a nice development UI remotely from the Jetson.

```python
sudo python3 line_v7_qut_track_1_variable_boost.py
```

```python
sudo python3 line_v7_qut_track_2_variable_boost.py
```

Both of these files are identical in central processing. However, there are minor tweaks with how the robot behaves around obstacles and curves. For ``track_1``, due to a higher number of straight sections along the track, the robot handles curves much more agressively and will aim to match top speed as quickly as possible to maximize time savings. However, ``track_2`` focused on manoeuvrability and consistency. As a result, the we set the robot to dampen turns and use smoother acceleration curves.

The program will automatically initialize motors and camera equipment. When it's done, it will ask for you to press the ``enter`` key to begin! We also implemented automatic stopping when the robot sees the finish line. On seeing the line, the robot will go into 'mad dash' mode, and sprint towards the finish line. On stopping, the robot will prompt for another input to start the next lap straight away. It will also output a ``time`` in seconds for track completion.

## Authors

- [@woflydev](https://www.github.com/woflydev)
- [@AwesomeGuy000](https://github.com/awesomeguy000)
- [@xdBeanjo](https://github.com/xdBeanjo)
- [@hashtable2020](https://github.com/hashtable2020)
- [@kelco-chan](https://github.com/kelco-chan)

## Contributing, Forking, and Support
Please refer to our root repository at [woflydev/odyssey_cnn](https://github.com/woflydev/odyssey_cnn/#contributing-and-forking) for contribution information.
